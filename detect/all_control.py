import cv2
import time
import logging
import sys
import os
import signal

# 添加模块路径
sys.path.append('/root/square/detect')
sys.path.append('/root/square/all_duoji')

# 添加GPIO控制
try:
    import Hobot.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    logging.warning("GPIO模块不可用，激光控制功能将被禁用")

# 导入模块化组件
from vision_pipeline import VisionPipeline, VisionConfig
from coordinate_mapper_square import SquareCoordinateMapper
from control import ServoController
from ui import SquareTrackerUI, UIStateManager

# 导入功能函数模块
from all_control_function import (
    load_default_config, load_config_from_file, save_config_to_file,
    merge_configs, validate_config, setup_gpio_laser, control_gpio_laser,
    cleanup_gpio, parse_command_line_args, apply_command_line_args,
    log_configuration_info, format_status_info, calculate_fps_stats,
    create_signal_handler, validate_servo_parameters, ensure_integer_speed_params
)

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='[%(name)s] [%(asctime)s] [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("SquareTracker")

class SquareTrackerCore:
    """方块跟踪核心模块 - 纯YOLO检测方案"""
    
    def __init__(self, config: dict):
        """初始化跟踪核心 - 纯检测模式"""
        self.config = ensure_integer_speed_params(config)
        
        # 运行控制标志
        self.should_stop = False
        self.external_control = False
        self.external_laser_force_high = False
        
        # 初始化纯YOLO视觉处理管道
        logger.info("初始化纯YOLO视觉处理管道...")
        vision_config = self._create_vision_config()
        self.vision_pipeline = VisionPipeline(vision_config)
        
        # 强制禁用OpenCV精细化
        self.vision_pipeline.set_opencv_enabled(False)
        
        # 初始化UI模块
        self.ui = SquareTrackerUI(config)
        
        # 极性配置参数
        self.x_polarity = config.get('x_polarity', -1)
        self.y_polarity = config.get('y_polarity', -1)
        
        # 初始化增量角度映射控制器
        logger.info("初始化增量角度映射控制器...")
        calibration_file = config.get('calibration_file', '/root/square/detect/square_servo_calibration.json')
        self.coordinate_mapper = SquareCoordinateMapper(calibration_file)
        
        # 设置坐标映射器的极性
        if hasattr(self.coordinate_mapper, 'set_polarity'):
            self.coordinate_mapper.set_polarity(self.x_polarity, self.y_polarity)
            logger.info(f"极性配置已设置: X轴={'正常' if self.x_polarity == 1 else '反向'}, Y轴={'正常' if self.y_polarity == 1 else '反向'}")
        
        # 初始化串行总线舵机控制器
        self.servo_controller = ServoController(
            port=config.get('servo_port', '/dev/ttyACM0'),
            baudrate=config.get('servo_baudrate', 1000000)
        )
        self.servo_connected = False
        
        # 串行总线舵机状态跟踪
        self.x_servo_id = config.get('x_servo_id', 1)
        self.y_servo_id = config.get('y_servo_id', 2)
        self.current_x_position = 0
        self.current_y_position = 0
        self.center_x_position = 0
        self.center_y_position = 0
        
        # 串行总线舵机参数
        self.servo_resolution = 4096
        self.angle_per_unit = 360.0 / self.servo_resolution
        
        # 初始化激光控制
        self.laser_pin = config.get('laser_pin', 16)
        self.laser_enabled = config.get('laser_enabled', True) and GPIO_AVAILABLE
        self.laser_current_state = GPIO.LOW if GPIO_AVAILABLE else False
        
        if self.laser_enabled:
            self.laser_enabled = setup_gpio_laser(self.laser_pin)
        
        # 摄像头
        self.cap = None
        
        # 状态变量
        self.is_tracking = False
        self.target_found = False
        self.last_detection_time = 0
        self.detection_timeout = 2.0
        
        # 统计信息
        self.frame_count = 0
        self.detection_count = 0
        self.control_count = 0
        self.laser_shot_count = 0
        self.mapping_success_count = 0
        self.mapping_failure_count = 0
        
        # 帧率控制
        self.target_fps = 60
        self.frame_time = 1.0 / self.target_fps
        self.last_frame_time = 0
        
        # 控制频率参数
        self.control_interval = 0.02
        self.last_control_time = 0
        
        # 简化舵机速度控制参数 - 支持多种速度模式
        self.servo_speed_normal = 30000      # 正常模式速度（2号、3号指令）
        self.servo_speed_high = 32000        # 高速模式速度（4号、5号指令）
        self.current_servo_speed = self.servo_speed_normal  # 当前使用的速度
        self.servo_speed_mode = 'normal'     # 当前速度模式
        
        # 保留原有参数以兼容现有代码
        self.servo_speed = self.servo_speed_normal
        self.min_servo_speed = 30000
        self.max_servo_speed = 32000
        self.speed_step = 0  # 不再需要速度调整
        
        # 移除自适应速度控制
        self.adaptive_speed = False
        self.high_speed_threshold = 0
        self.low_speed_threshold = 0
        self.high_speed = 30000
        self.medium_speed = 30000
        self.low_speed = 30000
        
        # 等待式控制状态管理
        self.servo_moving = False
        self.move_start_time = 0
        self.move_timeout = config.get('move_timeout', 3.0)
        self.position_tolerance = config.get('position_tolerance', 2.0)
        self.target_x_angle = None
        self.target_y_angle = None
        
        # 位置检查频率
        self.position_check_interval = 0.02
        self.last_position_check = 0
        
        # 快速响应模式参数
        self.fast_response_mode = config.get('fast_response_mode', True)
        self.position_tolerance = config.get('position_tolerance', 1.0)
        self.move_timeout = config.get('move_timeout', 1.5)
        
        # 在子线程运行时不设置信号处理器
        try:
            signal_handler = create_signal_handler(self)
            signal.signal(signal.SIGINT, signal_handler)
            signal.signal(signal.SIGTERM, signal_handler)
            logger.debug("信号处理器设置成功")
        except ValueError as e:
            logger.debug(f"信号处理器设置失败（可能在子线程中）: {e}")
        except Exception as e:
            logger.warning(f"信号处理器设置异常: {e}")
        
        # 记录检测方法的变量
        self.last_detection_method = 'Unknown'
        
        # 搜索模式相关变量
        self.search_mode_active = False
        self.search_direction = 0
        self.search_target_angle = None
        self.search_start_angle_x = None
        self.search_current_angle = None
        self.search_speed = config.get('search_speed', 32000)  # 使用30000速度
        self.search_step_angle = config.get('search_step_angle', 5.0) # 
        self.search_pause_time = config.get('search_pause_time', 0.3) # 
        self.search_last_step_time = 0
        self.search_max_range = config.get('search_max_range', 90.0)
        
        # **新增：D控制（微分控制）相关变量**
        self.d_control_enabled = False           # D控制启用状态
        self.d_control_acceleration_threshold = config.get('d_control_acceleration_threshold', 50)
        self.d_control_prediction_factor = config.get('d_control_prediction_factor', 0.3)
        self.d_control_min_delta_angle = config.get('d_control_min_delta_angle', 0.5)
        self.d_control_max_delta_angle = config.get('d_control_max_delta_angle', 3.0)
        self.d_control_velocity_smooth_factor = config.get('d_control_velocity_smooth_factor', 0.7)
        self.d_control_debug = config.get('d_control_debug', False)
        
        # 目标位置历史记录（用于计算速度和加速度）
        self.target_position_history = []  # [(time, x, y), ...]
        self.max_history_length = 5  # 保持最近5个位置记录
        
        # 当前速度和加速度
        self.current_velocity_x = 0.0  # 像素/秒
        self.current_velocity_y = 0.0  # 像素/秒
        self.current_acceleration_x = 0.0  # 像素/秒²
        self.current_acceleration_y = 0.0  # 像素/秒²
        self.last_velocity_x = 0.0
        self.last_velocity_y = 0.0
        self.last_velocity_time = 0.0
        
        # D控制预测状态
        self.d_control_prediction_active = False
        self.d_control_last_prediction_time = 0.0
        self.d_control_prediction_cooldown = 0.1  # 预测冷却时间（秒）
        
        logger.info("纯YOLO方块跟踪系统初始化完成 - 高性能检测方案")
        logger.info("🚀 性能优化: 跳过OpenCV后处理，直接使用YOLO检测框中心点")
        logger.info(f"⚡ 舵机速度配置: 正常模式={self.servo_speed_normal}, 高速模式={self.servo_speed_high}")
        if self.d_control_enabled:
            logger.info(f"🎯 D控制已启用: 加速度阈值={self.d_control_acceleration_threshold}, 预测因子={self.d_control_prediction_factor}")

    def _create_vision_config(self) -> dict:
        """根据主配置创建视觉处理配置 - 纯检测版本"""
        vision_config = VisionConfig.create_default_config()
        
        # 更新配置
        if 'yolo' in self.config:
            vision_config['yolo'].update(self.config['yolo'])
        if 'camera' in self.config:
            vision_config['camera'].update(self.config['camera'])
        
        # 强制设置为纯检测模式
        vision_config['enable_opencv_refinement'] = False
        vision_config['fallback_to_yolo'] = True
        vision_config['bbox_expansion'] = 0
        
        return vision_config

    def set_external_control(self, enabled: bool):
        """设置外部控制模式"""
        self.external_control = enabled
        if enabled:
            logger.info("🎮 切换到外部控制模式")
        else:
            logger.info("🎮 切换到独立运行模式")
    
    def set_laser_force_high(self, force_high: bool):
        """外部设置激光强制高电平"""
        self.external_laser_force_high = force_high
        if force_high:
            logger.info("🔴 激光强制高电平模式已启用")
        else:
            logger.info("🔵 激光强制高电平模式已禁用")
    
    def control_laser(self, should_fire: bool):
        """控制激光射击"""
        if not self.laser_enabled:
            return
        
        # 检查外部强制高电平模式
        if hasattr(self, 'external_laser_force_high') and self.external_laser_force_high:
            should_fire = True
            force_mode = True
        else:
            force_mode = False
        
        success, new_state = control_gpio_laser(self.laser_pin, should_fire, self.laser_current_state)
        
        if success and new_state != self.laser_current_state:
            self.laser_current_state = new_state
            if new_state == GPIO.HIGH:
                self.laser_shot_count += 1
                if force_mode:
                    logger.debug(f"🔴 激光强制亮起 - 外部控制模式")
                else:
                    logger.info(f"🔴 激光射击 - 第{self.laser_shot_count}次")
            else:
                logger.debug("🔵 激光关闭")
    
    def cleanup_and_exit(self):
        """清理资源并退出"""
        try:
            # 关闭激光
            if self.laser_enabled:
                self.control_laser(False)
                cleanup_gpio()
            
            # 清理UI资源
            self.ui.cleanup()
            
            # 断开设备连接
            self.disconnect_devices()
            
        except Exception as e:
            logger.error(f"清理资源时出错: {e}")
        finally:
            sys.exit(0)

    def connect_devices(self) -> bool:
        """连接所有设备"""
        logger.info("正在连接设备...")
        
        # 连接串行总线舵机
        if not self.servo_controller.connect():
            logger.error("串行总线舵机连接失败")
            return False
        
        self.servo_connected = True
        
        # 测试舵机连接
        if not self.servo_controller.ping_servo(self.x_servo_id):
            logger.error(f"X轴舵机 {self.x_servo_id} 无响应")
            return False
        
        if not self.servo_controller.ping_servo(self.y_servo_id):
            logger.error(f"Y轴舵机 {self.y_servo_id} 无响应")
            return False
        
        logger.info("串行总线舵机连接成功")
        
        # 连接摄像头
        self.cap = cv2.VideoCapture(self.config['camera_id'])
        if not self.cap.isOpened():
            logger.error("摄像头连接失败")
            return False
        
        # 设置摄像头参数
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['camera_width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['camera_height'])
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 初始化舵机位置
        self.setup_coordinate_mapping_mode()
        
        logger.info("所有设备连接成功")
        return True
    
    def setup_coordinate_mapping_mode(self):
        """设置增量角度映射控制模式"""
        logger.info("设置增量角度映射控制模式...")
        
        try:
            # 初始化舵机到安全位置
            try:
                current_x_angle = self.servo_controller.read_servo_position(self.x_servo_id)
                current_y_angle = self.servo_controller.read_servo_position(self.y_servo_id)
                
                if current_x_angle is not None and current_y_angle is not None:
                    self.current_x_position = int(current_x_angle / self.angle_per_unit)
                    self.current_y_position = int(current_y_angle / self.angle_per_unit)
                    logger.info(f"读取到舵机当前角度: X={current_x_angle:.1f}°, Y={current_y_angle:.1f}°")
                else:
                    safe_x_angle = 90.0
                    safe_y_angle = 45.0
                    
                    logger.warning("无法读取舵机当前位置，设置到安全位置")
                    self.servo_controller.write_servo_position(self.x_servo_id, safe_x_angle)
                    self.servo_controller.write_servo_position(self.y_servo_id, safe_y_angle)
                    time.sleep(1.0)
                    
                    self.current_x_position = int(safe_x_angle / self.angle_per_unit)
                    self.current_y_position = int(safe_y_angle / self.angle_per_unit)
                    logger.info(f"舵机已设置到安全位置: X={safe_x_angle:.1f}°, Y={safe_y_angle:.1f}°")
                
            except Exception as e:
                logger.error(f"舵机初始化失败: {e}")
                self.current_x_position = 0
                self.current_y_position = 0
            
            # 设置当前位置为中心位置
            self.center_x_position = self.current_x_position
            self.center_y_position = self.current_y_position
            
            x_angle = self.current_x_position * self.angle_per_unit
            y_angle = self.current_y_position * self.angle_per_unit
            
            logger.info(f"舵机中心位置: X={self.center_x_position}({x_angle:.1f}°), Y={self.center_y_position}({y_angle:.1f}°)")
                
        except Exception as e:
            logger.error(f"增量角度映射控制模式设置失败: {e}")
    
    def disconnect_devices(self):
        """断开所有设备连接"""
        if self.cap:
            self.cap.release()
        
        if self.servo_controller and self.servo_connected:
            self.servo_controller.disconnect()
            self.servo_connected = False
            logger.info("串行总线舵机已断开")
        
        logger.info("设备连接已断开")
    
    def process_frame(self, frame):
        """处理单帧图像 - 纯YOLO检测版本"""
        self.frame_count += 1
        
        # 使用纯YOLO视觉处理管道
        vision_result = self.vision_pipeline.process_frame(frame, enable_debug=(self.frame_count % 30 == 0))
        
        if vision_result['success']:
            self.target_found = True
            self.last_detection_time = time.time()
            self.detection_count += 1
            
            # 记录检测方法
            self.last_detection_method = vision_result.get('method', 'pure_yolo')
            
            # 获取YOLO检测框中心点
            center_x, center_y = vision_result['center']
            
            # 执行增量角度映射控制
            if self.is_tracking:
                self.process_coordinate_mapping_control(center_x, center_y, vision_result)
                self.control_count += 1
            
            # 使用视觉管道绘制检测结果
            if self.ui.is_enabled():
                frame = self.vision_pipeline.visualize_result(frame, vision_result)
        
        else:
            # 记录检测失败
            self.last_detection_method = "No Detection"
            
            # 没有检测到目标时，关闭激光
            if self.is_tracking:
                self.control_laser(False)
            
            # 检查检测超时
            if time.time() - self.last_detection_time > self.detection_timeout:
                if self.target_found:
                    logger.warning("目标丢失，停止跟踪")
                    self.target_found = False
                    self.control_laser(False)
        
        # 使用UI模块绘制界面元素
        if self.ui.is_enabled():
            tracker_state = UIStateManager.create_tracker_state(self)
            frame = self.ui.draw_ui_elements(frame, tracker_state)
        
        return frame

    def update_target_position_history(self, center_x: float, center_y: float):
        """**新增：更新目标位置历史记录**"""
        current_time = time.time()
        
        # 添加新的位置记录
        self.target_position_history.append((current_time, center_x, center_y))
        
        # 保持历史记录长度
        if len(self.target_position_history) > self.max_history_length:
            self.target_position_history.pop(0)
        
        # 计算速度和加速度
        self.calculate_velocity_and_acceleration()

    def calculate_velocity_and_acceleration(self):
        """**新增：计算目标移动速度和加速度**"""
        if len(self.target_position_history) < 2:
            return
        
        try:
            # 获取最近两个位置点
            time1, x1, y1 = self.target_position_history[-2]
            time2, x2, y2 = self.target_position_history[-1]
            
            dt = time2 - time1
            if dt <= 0:
                return
            
            # 计算瞬时速度（像素/秒）
            velocity_x = (x2 - x1) / dt
            velocity_y = (y2 - y1) / dt
            
            # 速度平滑滤波
            smooth_factor = self.d_control_velocity_smooth_factor
            self.current_velocity_x = smooth_factor * self.current_velocity_x + (1 - smooth_factor) * velocity_x
            self.current_velocity_y = smooth_factor * self.current_velocity_y + (1 - smooth_factor) * velocity_y
            
            # 计算加速度（像素/秒²）
            if self.last_velocity_time > 0:
                dt_accel = time2 - self.last_velocity_time
                if dt_accel > 0:
                    self.current_acceleration_x = (self.current_velocity_x - self.last_velocity_x) / dt_accel
                    self.current_acceleration_y = (self.current_velocity_y - self.last_velocity_y) / dt_accel
            
            # 更新上次速度记录
            self.last_velocity_x = self.current_velocity_x
            self.last_velocity_y = self.current_velocity_y
            self.last_velocity_time = time2
            
            # 调试输出
            if self.d_control_debug and self.frame_count % 30 == 0:
                logger.debug(f"D控制状态: 速度=({self.current_velocity_x:.1f},{self.current_velocity_y:.1f})px/s, "
                           f"加速度=({self.current_acceleration_x:.1f},{self.current_acceleration_y:.1f})px/s²")
                
        except Exception as e:
            if self.d_control_debug:
                logger.error(f"计算速度和加速度失败: {e}")

    def calculate_d_control_prediction(self) -> tuple:
        """**新增：计算D控制预测角度增量**"""
        if not self.d_control_enabled:
            return 0.0, 0.0
        
        try:
            # 检查加速度是否达到阈值
            accel_magnitude = (self.current_acceleration_x**2 + self.current_acceleration_y**2)**0.5
            
            if accel_magnitude < self.d_control_acceleration_threshold:
                return 0.0, 0.0
            
            # 检查预测冷却时间
            current_time = time.time()
            if current_time - self.d_control_last_prediction_time < self.d_control_prediction_cooldown:
                return 0.0, 0.0
            
            # 计算预测角度增量（基于加速度方向）
            prediction_x = self.current_acceleration_x * self.d_control_prediction_factor
            prediction_y = self.current_acceleration_y * self.d_control_prediction_factor
            
            # 将像素加速度转换为角度增量（简化转换）
            # 假设图像中心到边缘约对应45度角度范围
            image_width = self.config.get('camera_width', 640)
            image_height = self.config.get('camera_height', 480)
            
            # 像素到角度的转换因子
            pixel_to_angle_factor_x = 90.0 / (image_width / 2)  # 度/像素
            pixel_to_angle_factor_y = 90.0 / (image_height / 2)  # 度/像素
            
            # 计算预测角度增量
            delta_angle_x = prediction_x * pixel_to_angle_factor_x * 0.01  # 缩放因子
            delta_angle_y = prediction_y * pixel_to_angle_factor_y * 0.01  # 缩放因子
            
            # 限制预测角度范围
            delta_angle_x = max(-self.d_control_max_delta_angle, 
                               min(self.d_control_max_delta_angle, delta_angle_x))
            delta_angle_y = max(-self.d_control_max_delta_angle, 
                               min(self.d_control_max_delta_angle, delta_angle_y))
            
            # 应用最小角度阈值
            if abs(delta_angle_x) < self.d_control_min_delta_angle:
                delta_angle_x = 0.0
            if abs(delta_angle_y) < self.d_control_min_delta_angle:
                delta_angle_y = 0.0
            
            # 更新预测时间
            if delta_angle_x != 0.0 or delta_angle_y != 0.0:
                self.d_control_last_prediction_time = current_time
                
                if self.d_control_debug:
                    logger.info(f"🎯 D控制预测: 加速度=({self.current_acceleration_x:.1f},{self.current_acceleration_y:.1f})px/s², "
                              f"预测角度=({delta_angle_x:.2f},{delta_angle_y:.2f})°")
            
            return delta_angle_x, delta_angle_y
            
        except Exception as e:
            if self.d_control_debug:
                logger.error(f"D控制预测计算失败: {e}")
            return 0.0, 0.0

    def process_coordinate_mapping_control(self, center_x, center_y, vision_result):
        """纯YOLO检测的增量角度映射控制逻辑"""
        current_time = time.time()
        
        # **新增：更新目标位置历史（仅在D控制启用时）**
        if self.d_control_enabled:
            self.update_target_position_history(center_x, center_y)
        
        # 在快速响应模式下，允许舵机移动中进行控制计算
        if self.servo_moving and not self.fast_response_mode:
            logger.debug("舵机正在移动中，跳过控制计算")
            return
        
        # 提高控制频率
        if current_time - self.last_control_time < self.control_interval:
            return
        
        try:
            # 使用纯YOLO检测的中心点进行映射
            delta_x_angle, delta_y_angle = self.coordinate_mapper.detection_center_to_delta_angle(center_x, center_y)
            
            if delta_x_angle is None or delta_y_angle is None:
                logger.warning(f"纯YOLO中心点映射失败: 检测中心({center_x}, {center_y})")
                self.mapping_failure_count += 1
                return
            
            # **新增：应用D控制预测（仅在启用时）**
            d_delta_x, d_delta_y = 0.0, 0.0
            if self.d_control_enabled:
                d_delta_x, d_delta_y = self.calculate_d_control_prediction()
                
                # 将D控制预测加入到基础控制中
                delta_x_angle += d_delta_x
                delta_y_angle += d_delta_y
                
                if d_delta_x != 0.0 or d_delta_y != 0.0:
                    logger.info(f"🎯 D控制激活: 基础=({delta_x_angle-d_delta_x:.2f},{delta_y_angle-d_delta_y:.2f})°, "
                              f"预测=({d_delta_x:.2f},{d_delta_y:.2f})°, 总计=({delta_x_angle:.2f},{delta_y_angle:.2f})°")
            
            self.mapping_success_count += 1
            
            # 快速获取当前舵机角度
            if self.fast_response_mode:
                # 快速模式：使用估算值，避免串口读取延时
                current_x_angle = self.current_x_position * self.angle_per_unit
                current_y_angle = self.current_y_position * self.angle_per_unit
            else:
                # 精确模式：读取真实角度
                current_x_angle = self.servo_controller.read_servo_position(self.x_servo_id)
                current_y_angle = self.servo_controller.read_servo_position(self.y_servo_id)
                
                if current_x_angle is None or current_y_angle is None:
                    logger.warning("无法读取当前舵机角度，使用估算值")
                    current_x_angle = self.current_x_position * self.angle_per_unit
                    current_y_angle = self.current_y_position * self.angle_per_unit
            
            # 计算目标角度
            target_x_angle = current_x_angle + delta_x_angle
            target_y_angle = current_y_angle + delta_y_angle
            
            # 更小的角度死区，提高精度
            angle_deadzone_x = self.config.get('angle_deadzone_x', 0.3)
            angle_deadzone_y = self.config.get('angle_deadzone_y', 0.3)
            
            angle_diff_x = abs(target_x_angle - current_x_angle)
            angle_diff_y = abs(target_y_angle - current_y_angle)
            
            if angle_diff_x <= angle_deadzone_x and angle_diff_y <= angle_deadzone_y:
                # 在死区内，检查是否应该射击
                total_angle_error = (abs(delta_x_angle)**2 + abs(delta_y_angle)**2)**0.5
                should_fire = total_angle_error <= self.config.get('laser_angle_threshold', 0.8)
                self.control_laser(should_fire)
                
                logger.debug(f"在角度死区内: ΔX={angle_diff_x:.2f}°, ΔY={angle_diff_y:.2f}°, 激光={'射击' if should_fire else '待机'}")
                self.last_control_time = current_time
                return
            
            # 快速舵机移动
            logger.debug(f"🎯 舵机移动指令: 当前({current_x_angle:.1f}°,{current_y_angle:.1f}°) -> 目标({target_x_angle:.1f}°,{target_y_angle:.1f}°)")
            
            success = self.move_servo_to_angle_fast(target_x_angle, target_y_angle)
            
            if success:
                # 快速响应模式下减少状态管理开销
                if self.fast_response_mode:
                    # 快速模式：立即更新位置，不等待舵机到位
                    self.current_x_position = int(target_x_angle / self.angle_per_unit)
                    self.current_y_position = int(target_y_angle / self.angle_per_unit)
                    self.servo_moving = False
                else:
                    # 精确模式：设置移动状态等待
                    self.servo_moving = True
                    self.move_start_time = current_time
                    self.target_x_angle = target_x_angle
                    self.target_y_angle = target_y_angle
                    self.current_x_position = int(target_x_angle / self.angle_per_unit)
                    self.current_y_position = int(target_y_angle / self.angle_per_unit)
                
                # 移动时关闭激光
                self.control_laser(False)
                
                # 减少日志输出频率
                if self.frame_count % 10 == 0:
                    vision_method = vision_result.get('method', 'pure_yolo')
                    target_info = self.coordinate_mapper.get_target_info()
                    target_center = target_info['target_center']
                    offset_x = center_x - target_center[0]
                    offset_y = center_y - target_center[1]
                    
                    log_msg = f"舵机移动: 视觉={vision_method}, 检测=({center_x:.0f},{center_y:.0f}), 偏差=({offset_x:+.0f},{offset_y:+.0f}), 增量=(Δ{delta_x_angle:.2f}°,Δ{delta_y_angle:.2f}°), 角度差=({angle_diff_x:.2f}°,{angle_diff_y:.2f}°)"
                    
                    if self.d_control_enabled and (d_delta_x != 0.0 or d_delta_y != 0.0):
                        log_msg += f", D控制=({d_delta_x:.2f}°,{d_delta_y:.2f}°)"
                    
                    logger.info(log_msg)
            else:
                logger.error("❌ 舵机移动指令发送失败")
            
            self.last_control_time = current_time
            
        except Exception as e:
            logger.error(f"纯YOLO中心点映射控制更新失败: {e}")
            self.mapping_failure_count += 1

    def move_servo_to_angle_fast(self, target_x_angle, target_y_angle):
        """简化：快速舵机移动方法 - 使用当前速度模式"""
        if not (self.servo_controller and self.servo_connected):
            logger.debug("虚拟舵机模式，立即返回成功")
            return True
        
        try:
            # **关键修改：使用当前速度模式的速度**
            selected_speed = self.current_servo_speed
            
            # 发送移动指令
            try:
                x_angle = float(target_x_angle)
                y_angle = float(target_y_angle)
                speed = int(selected_speed)
                
                success_x = self.servo_controller.write_servo_position(self.x_servo_id, x_angle, speed)
                success_y = self.servo_controller.write_servo_position(self.y_servo_id, y_angle, speed)
                
            except Exception as param_error:
                logger.error(f"❌ 参数转换错误: {param_error}")
                return False
            
            if success_x and success_y:
                logger.debug(f"✓ 舵机移动: X={target_x_angle:.1f}°, Y={target_y_angle:.1f}°, 速度={selected_speed} ({self.servo_speed_mode}模式)")
                return True
            else:
                logger.warning(f"❌ 舵机移动失败: X成功={success_x}, Y成功={success_y}")
                return False
                
        except Exception as e:
            logger.error(f"舵机移动异常: {e}")
            return False

    def move_servo_to_angle_with_wait(self, target_x_angle, target_y_angle):
        """简化：移动舵机到指定角度（带等待逻辑）- 使用当前速度模式"""
        if not (self.servo_controller and self.servo_connected):
            logger.debug("虚拟舵机模式，立即返回成功")
            return True
        
        try:
            # **关键修改：使用当前速度模式的速度**
            selected_speed = self.current_servo_speed
            
            # 发送移动指令
            try:
                x_angle = float(target_x_angle)
                y_angle = float(target_y_angle)
                speed = int(selected_speed)
                
                success_x = self.servo_controller.write_servo_position(self.x_servo_id, x_angle, speed)
                success_y = self.servo_controller.write_servo_position(self.y_servo_id, y_angle, speed)
                
            except Exception as param_error:
                logger.error(f"❌ 参数转换错误: {param_error}")
                return False
            
            if success_x and success_y:
                logger.debug(f"✓ 舵机指令: X={target_x_angle:.1f}°, Y={target_y_angle:.1f}°, 速度={selected_speed} ({self.servo_speed_mode}模式)")
                return True
            else:
                logger.warning(f"❌ 舵机指令失败: X成功={success_x}, Y成功={success_y}")
                return False
                
        except Exception as e:
            logger.error(f"舵机移动异常: {e}")
            return False

    def set_servo_speed_mode(self, mode: str):
        """**修改：设置舵机速度模式 - 同时控制D控制状态**"""
        if mode == 'normal':
            self.current_servo_speed = self.servo_speed_normal
            self.servo_speed_mode = 'normal'
            # 2号3号指令禁用D控制
            self.enable_d_control(False)
            logger.info(f"🔧 舵机速度模式: 正常模式 ({self.servo_speed_normal}) - D控制已禁用")
        elif mode == 'high_speed':
            self.current_servo_speed = self.servo_speed_high
            self.servo_speed_mode = 'high_speed'
            # 4号5号指令启用D控制
            self.enable_d_control(True)
            logger.info(f"🔧 舵机速度模式: 高速模式 ({self.servo_speed_high}) - D控制已启用")
        else:
            logger.warning(f"⚠️ 未知速度模式: {mode}，保持当前模式")
        
        # 更新兼容参数
        self.servo_speed = self.current_servo_speed

    def enable_d_control(self, enabled: bool):
        """**新增：启用/禁用D控制**"""
        self.d_control_enabled = enabled
        if enabled:
            logger.info("🎯 D控制已启用 - 基于加速度的提前预测")
            # 清空历史记录
            self.target_position_history.clear()
            self.current_velocity_x = 0.0
            self.current_velocity_y = 0.0
            self.current_acceleration_x = 0.0
            self.current_acceleration_y = 0.0
        else:
            logger.info("🎯 D控制已禁用")

    def show_fps_stats(self):
        """显示FPS统计信息"""
        # ...existing code...
        
        # **新增：显示D控制状态**
        if self.d_control_enabled:
            logger.info("D控制状态:")
            logger.info(f"  当前速度: ({self.current_velocity_x:.1f}, {self.current_velocity_y:.1f}) px/s")
            logger.info(f"  当前加速度: ({self.current_acceleration_x:.1f}, {self.current_acceleration_y:.1f}) px/s²")
            logger.info(f"  加速度阈值: {self.d_control_acceleration_threshold} px/s²")
            logger.info(f"  预测因子: {self.d_control_prediction_factor}")
            logger.info(f"  位置历史长度: {len(self.target_position_history)}")

    def cleanup_gpio_resources(self):
        """清理GPIO资源"""
        try:
            if self.laser_enabled:
                self.control_laser(False)
                cleanup_gpio()
        except Exception as e:
            logger.error(f"清理GPIO资源时出错: {e}")

    def get_latest_frame(self):
        """获取最新帧，清空缓存中的旧帧"""
        if not self.cap:
            return None, None
        
        # 清空摄像头缓冲区，获取最新帧
        frame_count = 0
        ret, frame = None, None
        
        # 快速读取多帧，丢弃缓存中的旧帧
        for _ in range(5):  # 最多清空5帧缓存
            temp_ret, temp_frame = self.cap.read()
            if temp_ret:
                ret, frame = temp_ret, temp_frame
                frame_count += 1
            else:
                break
        
        if frame_count > 1:
            logger.debug(f"清空了{frame_count-1}帧缓存，获取最新帧")
        
        return ret, frame
    
    def wait_for_stable_image(self, max_wait_time=0.1):
        """优化：快速等待图像稳定"""
        if not self.fast_response_mode:
            return True  # 快速响应模式下跳过图像稳定等待
        
        start_time = time.time()
        stable_count = 0
        required_stable_frames = 1  # 优化：从3帧减少到1帧
        
        while time.time() - start_time < max_wait_time:
            ret, current_frame = self.get_latest_frame()
            if ret:
                stable_count += 1
                if stable_count >= required_stable_frames:
                    logger.debug(f"图像已稳定({stable_count}帧)")
                    return True
            time.sleep(0.01)  # 优化：从50ms减少到10ms
        
        logger.debug(f"快速模式：跳过图像稳定等待")
        return True
    
    def run_single_cycle(self):
        """执行单次处理循环 - 供外部调用"""
        try:
            # 修正：优先处理搜索模式
            if self.search_mode_active:
                # 搜索模式下的处理
                search_continue = self.process_search_step()
                
                if not search_continue:
                    # 搜索结束，转入正常跟踪模式
                    logger.info("🔍 搜索阶段结束，转入正常跟踪模式")
                
                # 搜索模式下仍需要进行图像处理以检测目标
                ret, frame = self.get_latest_frame()
                if not ret:
                    logger.error("无法读取摄像头帧")
                    return False
                
                # 处理图像（主要用于目标检测）
                processed_frame = self.process_frame(frame)
                
                # 显示图像
                if self.ui.is_enabled():
                    if self.config.get('display_fps_counter', True):
                        processed_frame = self.ui.add_fps_overlay(processed_frame)
                    
                    # 修正：在搜索模式下显示搜索状态
                    if self.search_mode_active:
                        if hasattr(self, 'angle_increment') and self.angle_increment is not None:
                            # 固定角度模式
                            direction_text = 'LEFT' if self.search_direction == 0 else 'RIGHT'
                            search_info = f"SEARCH: {direction_text} {self.angle_increment}° -> {self.search_target_angle:.1f}°"
                            if hasattr(self, 'search_angle_reached') and self.search_angle_reached:
                                search_info += " (REACHED)"
                        else:
                            # 持续搜索模式
                            direction_text = 'LEFT' if self.search_direction == 0 else 'RIGHT'
                            search_info = f"SEARCHING: {self.search_current_angle:.1f}° ({direction_text})"
                        
                        cv2.putText(processed_frame, search_info, (10, processed_frame.shape[0] - 60), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    
                    key = self.ui.display_frame(processed_frame)
                    
                    # 搜索模式下也可以响应某些按键
                    if not self.external_control:
                        if key == ord('q') or key == 27:  # 允许退出
                            return False
                        elif key == ord('s'):  # 显示状态
                            self.show_status()
                        elif key == ord(' '):  # 空格键可以停止搜索
                            if self.search_mode_active:
                                logger.info("用户手动停止搜索")
                                self.stop_search_mode()
                
                return True
            
            # 优化：简化舵机状态检查
            if self.servo_moving and not self.fast_response_mode:
                # 只在精确模式下检查舵机状态
                if self.check_servo_arrived():
                    logger.debug("✓ 舵机已到达目标位置")
                    self.servo_moving = False
                    self.wait_for_stable_image(0.05)
                    
                elif time.time() - self.move_start_time > self.move_timeout:
                    logger.warning("⚠️ 舵机移动超时，强制继续")
                    self.servo_moving = False
                else:
                    time.sleep(0.01)
                    return True  # 继续等待舵机移动
            
            # 获取最新图像帧
            ret, frame = self.get_latest_frame()
            if not ret:
                logger.error("无法读取摄像头帧")
                return False
            
            # 处理最新图像
            processed_frame = self.process_frame(frame)
            
            # 显示图像和处理用户输入
            if self.ui.is_enabled():
                # 添加FPS显示
                if self.config.get('display_fps_counter', True):
                    processed_frame = self.ui.add_fps_overlay(processed_frame)
                
                # 显示图像并获取按键
                key = self.ui.display_frame(processed_frame)
                
                # 只在非外部控制模式下处理键盘输入
                if not self.external_control:
                    if self.handle_keyboard_input(key):
                        return False  # 用户请求退出
            else:
                # 无显示模式 - 自动开始跟踪
                if not self.is_tracking and not self.external_control:
                    self.is_tracking = True
                    logger.info("自动开始跟踪（无显示模式）")
            
            return True
            
        except Exception as e:
            logger.error(f"单次处理循环异常: {e}")
            return False
    
    def run_continuous(self):
        """连续运行模式 - 纯检测版本"""
        logger.info("启动纯YOLO方块跟踪系统...")
        
        try:
            # 连接设备
            if not self.connect_devices():
                logger.error("设备连接失败，退出程序")
                return
            
            logger.info("设备连接成功，开始跟踪...")
            logger.info(f"🚀 纯YOLO检测模式: 跳过OpenCV后处理，直接使用检测框中心点")
            logger.info(f"响应模式: {'快速响应' if self.fast_response_mode else '精确控制'}")
            logger.info(f"控制频率: {1.0/self.control_interval:.0f}Hz")
            logger.info(f"目标帧率: {self.target_fps}fps")
            
            # 初始化UI窗口
            if self.ui.is_enabled():
                logger.info("🖥️ 图像显示已启用 - 将显示纯YOLO检测结果")
                logger.info(f"   窗口名称: {self.config.get('display_window_name', 'Square Tracker')}")
                
                # 初始化显示窗口
                if self.ui.initialize_display():
                    logger.info(f"✓ 图像显示窗口已创建")
                else:
                    logger.warning("图像显示窗口创建失败，切换到无头模式")
                    self.ui.enabled = False
            else:
                logger.info("🖥️ 图像显示已禁用 - 无头模式运行")
            
            logger.info("按键说明:")
            logger.info("  空格键 - 开始/停止跟踪")
            logger.info("  s - 显示系统状态")
            logger.info("  x - 切换X轴极性")
            logger.info("  y - 切换Y轴极性")
            logger.info("  r - 重置极性为默认值")
            # 移除速度调整按键说明
            if self.ui.is_enabled():
                logger.info("  d - 切换显示模式")
                logger.info("  f - 显示FPS统计")
            logger.info("  q/ESC - 退出程序")
            
            # 预热摄像头
            logger.info("预热摄像头...")
            for _ in range(5):
                self.cap.read()
            
            # 初始化UI窗口
            if self.ui.is_enabled():
                self.ui.initialize_display()
                logger.info(f"✓ 图像显示窗口已创建")
            
            # 高频率主循环
            frame_start_time = time.time()
            
            while not self.should_stop:
                if not self.run_single_cycle():
                    break
                
                # 状态显示
                if time.time() - frame_start_time >= 60:
                    if not self.external_control:
                        self.show_status()
                    frame_start_time = time.time()
                
                # 帧率控制
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            logger.info("接收到键盘中断")
        except Exception as e:
            logger.error(f"运行时异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            logger.info("正在清理资源...")
            self.cleanup_and_exit()

    def start_tracking(self):
        """启动跟踪"""
        if not self.is_tracking:
            self.is_tracking = True
            logger.info("🎯 开始方块跟踪")
    
    def stop_tracking(self):
        """停止跟踪"""
        if self.is_tracking:
            self.is_tracking = False
            logger.info("⏹️ 停止方块跟踪")
    
    def stop_core(self):
        """停止核心模块"""
        self.should_stop = True
        logger.info("🛑 核心模块停止信号已发送")
    
    def handle_keyboard_input(self, key):
        """简化：处理键盘输入，移除速度调整按键"""
        if key == ord(' '):  # 空格键 - 开始/停止跟踪
            self.is_tracking = not self.is_tracking
            status = "开启" if self.is_tracking else "关闭"
            logger.info(f"跟踪状态: {status}")
        elif key == ord('s'):  # s键 - 显示状态
            self.show_status()
        elif key == ord('x') or key == ord('X'):  # X键 - 切换X轴极性
            self.toggle_x_polarity()
        elif key == ord('y') or key == ord('Y'):  # Y键 - 切换Y轴极性
            self.toggle_y_polarity()
        elif key == ord('r') or key == ord('R'):  # R键 - 重置极性
            self.reset_polarity()
        elif key == ord('d') or key == ord('D'):  # D键 - 切换显示模式
            if self.ui.is_enabled():
                self.ui.toggle_display_mode()
        elif key == ord('f') or key == ord('F'):  # F键 - 显示FPS统计
            self.show_fps_stats()
        elif key == ord('q') or key == 27:  # q键或ESC - 退出
            logger.info("用户请求退出")
            return True
        
        return False

    def _execute_fixed_angle_move(self):
        """简化：执行固定角度移动 - 统一30000速度"""
        try:
            logger.info(f"🔄 执行1号舵机移动: {self.search_start_angle_x:.1f}° → {self.search_target_angle:.1f}°")
            
            # 使用固定30000速度移动1号舵机
            success = self.servo_controller.write_servo_position(
                self.x_servo_id, 
                float(self.search_target_angle), 
                30000
            )
            
            if success:
                self.current_x_position = int(self.search_target_angle / self.angle_per_unit)
                logger.info(f"✅ 1号舵机移动指令已发送，速度: 30000")
                self.search_angle_reached = True
                return True
            else:
                logger.error("❌ 1号舵机移动指令发送失败")
                return False
                
        except Exception as e:
            logger.error(f"执行固定角度移动失败: {e}")
            return False
    
    def start_search_mode(self, search_params: dict) -> bool:
        """**新增：启动搜索模式接口**"""
        try:
            logger.info(f"🔍 启动搜索模式: {search_params}")
            
            # 这个方法主要是为了UART控制器接口兼容
            # 实际的搜索逻辑在UART控制器中实现
            return True
            
        except Exception as e:
            logger.error(f"启动搜索模式失败: {e}")
            return False

    def stop_search_mode(self):
        """停止搜索模式"""
        if self.search_mode_active:
            self.search_mode_active = False
            logger.info("🔍 搜索模式已停止")
    
    def check_servo_arrived(self):
        """优化：快速检查舵机是否到达目标位置"""
        if self.fast_response_mode:
            return True  # 快速模式下假定立即到达
        
        if self.target_x_angle is None or self.target_y_angle is None:
            return True
        
        try:
            # 优化：减少串口读取频率
            current_time = time.time()
            if current_time - self.last_position_check < self.position_check_interval:
                return False  # 还没到检查时间
            
            self.last_position_check = current_time
            
            # 读取当前舵机角度
            current_x_angle = self.servo_controller.read_servo_position(self.x_servo_id)
            current_y_angle = self.servo_controller.read_servo_position(self.y_servo_id)
            
            if current_x_angle is None or current_y_angle is None:
                logger.warning("无法读取舵机位置，假定已到达")
                return True
            
            # 计算误差
            error_x = abs(current_x_angle - self.target_x_angle)
            error_y = abs(current_y_angle - self.target_y_angle)
            
            # 检查是否在容差范围内
            arrived = (error_x <= self.position_tolerance) and (error_y <= self.position_tolerance)
            
            if arrived:
                logger.debug(f"✓ 舵机到达目标: 目标({self.target_x_angle:.1f}°,{self.target_y_angle:.1f}°), 当前({current_x_angle:.1f}°,{current_y_angle:.1f}°), 误差({error_x:.2f}°,{error_y:.2f}°)")
            
            return arrived
            
        except Exception as e:
            logger.error(f"检查舵机到位状态失败: {e}")
            return True  # 出错时假定已到达，避免死锁

    def run(self):
        """兼容性接口 - 叫用连续运行模式"""
        self.run_continuous()

    def cleanup_gpio_resources(self):
        """清理GPIO资源"""
        try:
            if self.laser_enabled:
                self.control_laser(False)
                cleanup_gpio()
        except Exception as e:
            logger.error(f"清理GPIO资源时出错: {e}")
