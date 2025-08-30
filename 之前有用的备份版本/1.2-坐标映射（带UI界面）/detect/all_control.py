import cv2
import time
import logging
import argparse
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
from ui import SquareTrackerUI, UIStateManager  # **新增：导入UI模块**

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='[%(name)s] [%(asctime)s] [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("SquareTracker")

class ModularSquareTracker:
    """模块化方块跟踪控制系统 - 专用于增量角度映射控制"""
    
    def __init__(self, config: dict):
        """初始化跟踪系统"""
        self.config = config
        
        # 初始化模块化视觉处理管道
        logger.info("初始化模块化视觉处理管道...")
        vision_config = self._create_vision_config()
        self.vision_pipeline = VisionPipeline(vision_config)
        
        # **新增：初始化UI模块**
        self.ui = SquareTrackerUI(config)
        
        # **新增：极性配置参数**
        self.x_polarity = config.get('x_polarity', -1)  # X轴极性：1=正常，-1=反向
        self.y_polarity = config.get('y_polarity', -1)  # Y轴极性：1=正常，-1=反向
        
        # 初始化增量角度映射控制器
        logger.info("初始化增量角度映射控制器...")
        calibration_file = config.get('calibration_file', '/root/square/detect/square_servo_calibration.json')
        logger.info(f"标定文件路径: {calibration_file}")
        
        self.coordinate_mapper = SquareCoordinateMapper(calibration_file)
        logger.info("坐标映射器初始化完成")
        
        # **设置坐标映射器的极性**
        if hasattr(self.coordinate_mapper, 'set_polarity'):
            self.coordinate_mapper.set_polarity(self.x_polarity, self.y_polarity)
            logger.info(f"极性配置已设置: X轴={'正常' if self.x_polarity == 1 else '反向'}, Y轴={'正常' if self.y_polarity == 1 else '反向'}")
        
        # **修复：初始化串行总线舵机控制器**
        self.servo_controller = ServoController(
            port=config.get('servo_port', '/dev/ttyACM0'),
            baudrate=config.get('servo_baudrate', 1000000)
        )
        self.servo_connected = False  # **新增：舵机连接状态标志**
        
        # 串行总线舵机状态跟踪
        self.x_servo_id = config.get('x_servo_id', 1)
        self.y_servo_id = config.get('y_servo_id', 2)
        self.current_x_position = 0  # 当前X轴位置
        self.current_y_position = 0  # 当前Y轴位置
        self.center_x_position = 0   # 中心X轴位置
        self.center_y_position = 0   # 中心Y轴位置
        
        # 串行总线舵机参数
        self.servo_resolution = 4096
        self.angle_per_unit = 360.0 / self.servo_resolution
        
        # 初始化激光控制GPIO
        self.laser_pin = config.get('laser_pin', 16)
        self.laser_enabled = config.get('laser_enabled', True) and GPIO_AVAILABLE
        self.laser_current_state = GPIO.LOW if GPIO_AVAILABLE else False
        
        if self.laser_enabled:
            self.setup_laser_gpio()
        
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
        self.target_fps = 30
        self.frame_time = 1.0 / self.target_fps
        self.last_frame_time = 0
        
        # 控制频率限制
        self.control_interval = 0.05
        self.last_control_time = 0
        
        # **新增：舵机速度控制参数**
        self.servo_speed = config.get('servo_speed', 200)  # 默认速度
        self.min_servo_speed = config.get('min_servo_speed', 100)    # 最小速度
        self.max_servo_speed = config.get('max_servo_speed', 200)   # 最大速度
        self.speed_step = config.get('speed_step', 100)              # 速度调整步长
        
        # **新增：自适应速度控制**
        self.adaptive_speed = config.get('adaptive_speed', True)     # 是否启用自适应速度
        self.high_speed_threshold = config.get('high_speed_threshold', 5.0)  # 大角度移动阈值(度)
        self.low_speed_threshold = config.get('low_speed_threshold', 1.0)    # 小角度移动阈值(度)
        self.high_speed = config.get('high_speed', 200)            # 大角度移动速度
        self.medium_speed = config.get('medium_speed', 200)        # 中等角度移动速度
        self.low_speed = config.get('low_speed', 200)               # 精细调整速度
        
        # **新增：等待式控制状态管理**
        self.servo_moving = False      # 舵机是否正在移动
        self.move_start_time = 0       # 移动开始时间
        self.move_timeout = 3.0        # 移动超时时间（秒）
        self.position_tolerance = 2.0  # 位置到达容差（度）
        self.target_x_angle = None     # 目标X轴角度
        self.target_y_angle = None     # 目标Y轴角度
        
        # **修改：大幅减少控制频率**
        self.control_interval = 0.2    # 增加到200ms，给舵机充足时间
        
        # **新增：位置检查频率**
        self.position_check_interval = 0.1  # 每100ms检查一次位置
        self.last_position_check = 0
        
        # **优化：大幅提高控制频率**
        self.control_interval = 0.02    # 从200ms减少到20ms，50Hz控制频率
        
        # **优化：提高位置检查频率**
        self.position_check_interval = 0.02  # 从100ms减少到20ms，50Hz检查频率
        self.last_position_check = 0
        
        # **新增：快速响应模式参数**
        self.fast_response_mode = config.get('fast_response_mode', True)
        self.position_tolerance = config.get('position_tolerance', 1.0)  # 从2.0度减少到1.0度
        self.move_timeout = config.get('move_timeout', 1.5)  # 从3.0秒减少到1.5秒
        
        # **优化：减少帧时间，提高整体响应**
        self.target_fps = 60  # 从30fps提高到60fps
        self.frame_time = 1.0 / self.target_fps
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # **新增：记录检测方法的变量**
        self.last_detection_method = 'Unknown'
        
        logger.info("模块化方块跟踪系统初始化完成 - 增量角度映射控制")
    
    def _create_vision_config(self) -> dict:
        """根据主配置创建视觉处理配置"""
        vision_config = VisionConfig.create_accuracy_config()
        
        # 更新YOLO配置
        if 'yolo' in self.config:
            vision_config['yolo'].update(self.config['yolo'])
        
        # 更新OpenCV配置
        if 'opencv' in self.config:
            vision_config['opencv'].update(self.config['opencv'])
        
        # 更新摄像头配置
        if 'camera' in self.config:
            vision_config['camera'].update(self.config['camera'])
        
        # 设置处理模式
        vision_config['enable_opencv_refinement'] = self.config.get('enable_opencv_refinement', True)
        vision_config['fallback_to_yolo'] = self.config.get('fallback_to_yolo', True)
        
        return vision_config
    
    def setup_laser_gpio(self):
        """设置激光控制GPIO"""
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.laser_pin, GPIO.OUT, initial=GPIO.LOW)
            self.laser_current_state = GPIO.LOW
            logger.info(f"激光控制GPIO初始化成功 - 引脚{self.laser_pin}")
        except Exception as e:
            logger.error(f"激光控制GPIO初始化失败: {e}")
            self.laser_enabled = False
    
    def control_laser(self, should_fire: bool):
        """控制激光射击"""
        if not self.laser_enabled:
            return
        
        try:
            new_state = GPIO.HIGH if should_fire else GPIO.LOW
            
            if new_state != self.laser_current_state:
                GPIO.output(self.laser_pin, new_state)
                self.laser_current_state = new_state
                
                if should_fire:
                    self.laser_shot_count += 1
                    logger.info(f"🔴 激光射击 - 第{self.laser_shot_count}次")
                else:
                    logger.debug("🔵 激光关闭")
        except Exception as e:
            logger.error(f"激光控制失败: {e}")
    
    def signal_handler(self, signum, frame):
        """信号处理器 - 用于优雅退出"""
        logger.info(f"接收到信号 {signum}，正在优雅退出...")
        self.cleanup_and_exit()
    
    def cleanup_and_exit(self):
        """清理资源并退出"""
        try:
            # 关闭激光
            if self.laser_enabled:
                self.control_laser(False)
                GPIO.cleanup()
                logger.info("GPIO资源已清理")
            
            # **新增：清理UI资源**
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
        
        # **修复：连接串行总线舵机并设置连接状态**
        if not self.servo_controller.connect():
            logger.error("串行总线舵机连接失败")
            return False
        
        self.servo_connected = True  # **设置连接状态标志**
        
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
            # **新增：初始化舵机到安全位置**
            try:
                # 尝试读取当前舵机位置
                current_x_angle = self.servo_controller.read_servo_position(self.x_servo_id)
                current_y_angle = self.servo_controller.read_servo_position(self.y_servo_id)
                
                if current_x_angle is not None and current_y_angle is not None:
                    # 转换角度为位置值
                    self.current_x_position = int(current_x_angle / self.angle_per_unit)
                    self.current_y_position = int(current_y_angle / self.angle_per_unit)
                    logger.info(f"读取到舵机当前角度: X={current_x_angle:.1f}°, Y={current_y_angle:.1f}°")
                else:
                    # 读取失败，设置到安全中心位置
                    safe_x_angle = 90.0  # 安全的X轴角度
                    safe_y_angle = 45.0  # 安全的Y轴角度
                    
                    logger.warning("无法读取舵机当前位置，设置到安全位置")
                    self.servo_controller.write_servo_position(self.x_servo_id, safe_x_angle)
                    self.servo_controller.write_servo_position(self.y_servo_id, safe_y_angle)
                    time.sleep(1.0)  # 等待舵机移动到位
                    
                    self.current_x_position = int(safe_x_angle / self.angle_per_unit)
                    self.current_y_position = int(safe_y_angle / self.angle_per_unit)
                    logger.info(f"舵机已设置到安全位置: X={safe_x_angle:.1f}°, Y={safe_y_angle:.1f}°")
                
            except Exception as e:
                logger.error(f"舵机初始化失败: {e}")
                # 使用默认位置值
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
        
        # **删除：移除OpenCV窗口清理代码，由UI模块处理**
        
        logger.info("设备连接已断开")
    
    def process_frame(self, frame):
        """**增强：处理单帧图像 - 记录检测方法**"""
        self.frame_count += 1
        
        # 使用视觉处理管道进行检测和精细化处理
        vision_result = self.vision_pipeline.process_frame(frame, enable_debug=(self.frame_count % 30 == 0))
        
        if vision_result['success']:
            self.target_found = True
            self.last_detection_time = time.time()
            self.detection_count += 1
            
            # **新增：记录检测方法**
            self.last_detection_method = vision_result.get('method', 'Unknown')
            
            # 获取精确的目标中心点
            center_x, center_y = vision_result['center']
            
            # 执行增量角度映射控制
            if self.is_tracking:
                self.process_coordinate_mapping_control(center_x, center_y, vision_result)
                self.control_count += 1
            
            # **修改：使用视觉管道绘制检测结果**
            if self.ui.is_enabled():
                frame = self.vision_pipeline.visualize_result(frame, vision_result)
        
        else:
            # **新增：记录检测失败**
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
        
        # **修改：使用UI模块绘制界面元素**
        if self.ui.is_enabled():
            tracker_state = UIStateManager.create_tracker_state(self)
            frame = self.ui.draw_ui_elements(frame, tracker_state)
        
        return frame
    
    def get_latest_frame(self):
        """**新增：获取最新帧，清空缓存中的旧帧**"""
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
    
    def wait_for_stable_image(self, max_wait_time=0.1):  # **优化：从0.5秒减少到0.1秒**
        """**优化：快速等待图像稳定**"""
        if not self.fast_response_mode:
            return True  # 快速响应模式下跳过图像稳定等待
        
        start_time = time.time()
        stable_count = 0
        required_stable_frames = 1  # **优化：从3帧减少到1帧**
        
        while time.time() - start_time < max_wait_time:
            ret, current_frame = self.get_latest_frame()
            if ret:
                stable_count += 1
                if stable_count >= required_stable_frames:
                    logger.debug(f"图像已稳定({stable_count}帧)")
                    return True
            time.sleep(0.01)  # **优化：从50ms减少到10ms**
        
        logger.debug(f"快速模式：跳过图像稳定等待")
        return True

    def process_coordinate_mapping_control(self, center_x, center_y, vision_result):
        """**优化：高频率增量角度映射控制逻辑**"""
        current_time = time.time()
        
        # **优化：在快速响应模式下，允许舵机移动中进行控制计算**
        if self.servo_moving and not self.fast_response_mode:
            logger.debug("舵机正在移动中，跳过控制计算")
            return
        
        # **优化：提高控制频率**
        if current_time - self.last_control_time < self.control_interval:
            return
        
        try:
            # 使用YOLO+OpenCV检测的中心点进行映射
            delta_x_angle, delta_y_angle = self.coordinate_mapper.detection_center_to_delta_angle(center_x, center_y)
            
            if delta_x_angle is None or delta_y_angle is None:
                logger.warning(f"YOLO+OpenCV中心点映射失败: 检测中心({center_x}, {center_y})")
                self.mapping_failure_count += 1
                return
            
            self.mapping_success_count += 1
            
            # **优化：快速获取当前舵机角度**
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
            
            # **优化：更小的角度死区，提高精度**
            angle_deadzone_x = self.config.get('angle_deadzone_x', 0.3)  # 从0.5度减少到0.3度
            angle_deadzone_y = self.config.get('angle_deadzone_y', 0.3)  # 从0.5度减少到0.3度
            
            angle_diff_x = abs(target_x_angle - current_x_angle)
            angle_diff_y = abs(target_y_angle - current_y_angle)
            
            if angle_diff_x <= angle_deadzone_x and angle_diff_y <= angle_deadzone_y:
                # 在死区内，检查是否应该射击
                total_angle_error = (abs(delta_x_angle)**2 + abs(delta_y_angle)**2)**0.5
                should_fire = total_angle_error <= self.config.get('laser_angle_threshold', 0.8) # 从1.0度减少到0.8度
                self.control_laser(should_fire)
                
                logger.debug(f"在角度死区内: ΔX={angle_diff_x:.2f}°, ΔY={angle_diff_y:.2f}°, 激光={'射击' if should_fire else '待机'}")
                self.last_control_time = current_time
                return
            
            # **优化：快速舵机移动**
            logger.debug(f"🎯 舵机移动指令: 当前({current_x_angle:.1f}°,{current_y_angle:.1f}°) -> 目标({target_x_angle:.1f}°,{target_y_angle:.1f}°)")
            
            success = self.move_servo_to_angle_fast(target_x_angle, target_y_angle)
            
            if success:
                # **优化：快速响应模式下减少状态管理开销**
                if self.fast_response_mode:
                    # 快速模式：立即更新位置，不等待舵机到位
                    self.current_x_position = int(target_x_angle / self.angle_per_unit)
                    self.current_y_position = int(target_y_angle / self.angle_per_unit)
                    self.servo_moving = False  # 快速模式下不设置移动状态
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
                
                # **优化：减少日志输出频率**
                if self.frame_count % 10 == 0:  # 每10帧输出一次详细信息
                    vision_method = vision_result.get('method', 'unknown')
                    target_info = self.coordinate_mapper.get_target_info()
                    target_center = target_info['target_center']
                    offset_x = center_x - target_center[0]
                    offset_y = center_y - target_center[1]
                    
                    logger.info(f"舵机移动: 视觉={vision_method}, 检测=({center_x:.0f},{center_y:.0f}), 偏差=({offset_x:+.0f},{offset_y:+.0f}), 增量=(Δ{delta_x_angle:.2f}°,Δ{delta_y_angle:.2f}°), 角度差=({angle_diff_x:.2f}°,{angle_diff_y:.2f}°)")
            else:
                logger.error("❌ 舵机移动指令发送失败")
            
            self.last_control_time = current_time
            
        except Exception as e:
            logger.error(f"YOLO+OpenCV中心点映射控制更新失败: {e}")
            self.mapping_failure_count += 1
    
    def move_servo_to_angle_fast(self, target_x_angle, target_y_angle):
        """**修复：快速舵机移动方法 - 正确的参数传递**"""
        if not (self.servo_controller and self.servo_connected):
            logger.debug("虚拟舵机模式，立即返回成功")
            return True
        
        try:
            # **优化：计算自适应速度（更激进的速度选择）**
            if self.fast_response_mode:
                current_x_angle = self.current_x_position * self.angle_per_unit
                current_y_angle = self.current_y_position * self.angle_per_unit
            else:
                current_x_angle = self.servo_controller.read_servo_position(self.x_servo_id) or (self.current_x_position * self.angle_per_unit)
                current_y_angle = self.servo_controller.read_servo_position(self.y_servo_id) or (self.current_y_position * self.angle_per_unit)
            
            delta_x_angle = abs(target_x_angle - current_x_angle)
            delta_y_angle = abs(target_y_angle - current_y_angle)
            max_delta_angle = max(delta_x_angle, delta_y_angle)
            
            # **优化：更激进的速度选择策略**
            if self.adaptive_speed:
                if max_delta_angle >= 3.0:  # 从5.0度降低到3.0度
                    selected_speed = min(2000, int(self.high_speed * 1.5))  # **修复：转换为整数**
                    speed_desc = "超高速"
                elif max_delta_angle >= 1.0:  # 从1.0度保持不变
                    selected_speed = int(self.medium_speed)  # **修复：转换为整数**
                    speed_desc = "中速"
                else:
                    selected_speed = int(self.low_speed)  # **修复：转换为整数**
                    speed_desc = "低速"
            else:
                selected_speed = min(2000, int(self.servo_speed * 1.2))  # **修复：转换为整数**
                speed_desc = "快速固定"
            
            # **修复：发送移动指令 - 确保所有参数类型正确**
            try:
                # 确保角度是浮点数，速度是整数
                x_angle = float(target_x_angle)
                y_angle = float(target_y_angle)
                speed = int(selected_speed)
                
                # 调用舵机控制方法时传递正确的参数
                success_x = self.servo_controller.write_servo_position(self.x_servo_id, x_angle, speed)
                success_y = self.servo_controller.write_servo_position(self.y_servo_id, y_angle, speed)
                
            except Exception as param_error:
                logger.error(f"❌ 参数转换错误: {param_error}")
                logger.error(f"   参数类型: x_servo_id={type(self.x_servo_id)}, target_x_angle={type(target_x_angle)}, speed={type(selected_speed)}")
                return False
            
            if success_x and success_y:
                # **优化：减少调试输出**
                if max_delta_angle > 1.0:  # 只有大幅移动时才输出
                    logger.debug(f"✓ 快速移动: X={target_x_angle:.1f}°, Y={target_y_angle:.1f}°, 速度={selected_speed}({speed_desc}), 角度变化={max_delta_angle:.2f}°")
                return True
            else:
                logger.warning(f"❌ 快速移动失败: X成功={success_x}, Y成功={success_y}")
                logger.warning(f"   参数详情: X轴ID={self.x_servo_id}, X角度={target_x_angle:.1f}°, Y轴ID={self.y_servo_id}, Y角度={target_y_angle:.1f}°, 速度={selected_speed}")
                return False
                
        except Exception as e:
            logger.error(f"快速舵机移动异常: {e}")
            logger.error(f"异常详情: target_x_angle={target_x_angle} ({type(target_x_angle)}), target_y_angle={target_y_angle} ({type(target_y_angle)})")
            import traceback
            traceback.print_exc()
            return False
    
    def move_servo_to_angle_with_wait(self, target_x_angle, target_y_angle):
        """**修复：移动舵机到指定角度（带等待逻辑）- 正确的参数传递**"""
        if not (self.servo_controller and self.servo_connected):
            logger.debug("虚拟舵机模式，立即返回成功")
            return True
        
        try:
            # **计算移动距离和自适应速度**
            current_x_angle = self.servo_controller.read_servo_position(self.x_servo_id) or (self.current_x_position * self.angle_per_unit)
            current_y_angle = self.servo_controller.read_servo_position(self.y_servo_id) or (self.current_y_position * self.angle_per_unit)
            
            delta_x_angle = abs(target_x_angle - current_x_angle)
            delta_y_angle = abs(target_y_angle - current_y_angle)
            max_delta_angle = max(delta_x_angle, delta_y_angle)
            
            # **自适应速度选择**
            if self.adaptive_speed:
                if max_delta_angle >= self.high_speed_threshold:
                    selected_speed = int(self.high_speed)  # **修复：转换为整数**
                    speed_desc = "高速"
                elif max_delta_angle >= self.low_speed_threshold:
                    selected_speed = int(self.medium_speed)  # **修复：转换为整数**
                    speed_desc = "中速"
                else:
                    selected_speed = int(self.low_speed)  # **修复：转换为整数**
                    speed_desc = "低速"
            else:
                selected_speed = int(self.servo_speed)  # **修复：转换为整数**
                speed_desc = "固定"
            
            # **修复：发送移动指令 - 确保参数类型正确**
            try:
                # 确保参数类型正确
                x_angle = float(target_x_angle)
                y_angle = float(target_y_angle)
                speed = int(selected_speed)
                
                success_x = self.servo_controller.write_servo_position(self.x_servo_id, x_angle, speed)
                success_y = self.servo_controller.write_servo_position(self.y_servo_id, y_angle, speed)
                
            except Exception as param_error:
                logger.error(f"❌ 参数转换错误: {param_error}")
                return False
            
            if success_x and success_y:
                logger.debug(f"✓ 舵机移动指令已发送: X={target_x_angle:.1f}°, Y={target_y_angle:.1f}°, 速度={selected_speed}({speed_desc}), 最大角度变化={max_delta_angle:.2f}°")
                return True
            else:
                logger.warning(f"❌ 舵机移动指令发送失败: X成功={success_x}, Y成功={success_y}")
                return False
                
        except Exception as e:
            logger.error(f"舵机移动异常: {e}")
            return False

    def increase_servo_speed(self):
        """**修复：增加舵机速度 - 确保数值类型正确**"""
        old_speed = self.servo_speed
        self.servo_speed = min(self.max_servo_speed, self.servo_speed + self.speed_step)
        
        # **修复：确保速度参数是整数**
        self.servo_speed = int(self.servo_speed)
        self.min_servo_speed = int(self.min_servo_speed)
        self.max_servo_speed = int(self.max_servo_speed)
        self.speed_step = int(self.speed_step)
        
        logger.info(f"⬆️ 舵机速度已增加: {old_speed} -> {self.servo_speed}")
    
    def decrease_servo_speed(self):
        """**修复：减少舵机速度 - 确保数值类型正确**"""
        old_speed = self.servo_speed
        self.servo_speed = max(self.min_servo_speed, self.servo_speed - self.speed_step)
        
        # **修复：确保速度参数是整数**
        self.servo_speed = int(self.servo_speed)
        
        logger.info(f"⬇️ 舵机速度已减少: {old_speed} -> {self.servo_speed}")
    
    def toggle_adaptive_speed(self):
        """**修复：切换自适应速度模式 - 确保数值类型正确**"""
        self.adaptive_speed = not self.adaptive_speed
        
        # **修复：确保所有速度参数是整数**
        self.high_speed = int(self.high_speed)
        self.medium_speed = int(self.medium_speed)
        self.low_speed = int(self.low_speed)
        
        status = "启用" if self.adaptive_speed else "禁用"
        logger.info(f"🔄 自适应速度已{status}")
        if self.adaptive_speed:
            logger.info(f"自适应速度配置: 高速(≥{self.high_speed_threshold}°)={self.high_speed}, 中速({self.low_speed_threshold}°-{self.high_speed_threshold}°)={self.medium_speed}, 低速(<{self.low_speed_threshold}°)={self.low_speed}")
        else:
            logger.info(f"固定速度: {self.servo_speed}")

    def toggle_x_polarity(self):
        """**新增：切换X轴极性**"""
        self.x_polarity *= -1
        if hasattr(self.coordinate_mapper, 'set_polarity'):
            self.coordinate_mapper.set_polarity(self.x_polarity, self.y_polarity)
        
        polarity_desc = "正常" if self.x_polarity == 1 else "反向"
        logger.info(f"🔄 X轴极性已切换为: {polarity_desc} ({self.x_polarity})")
        logger.info(f"💡 如果X轴移动方向错误，可以按X键调整")
    
    def toggle_y_polarity(self):
        """**新增：切换Y轴极性**"""
        self.y_polarity *= -1
        if hasattr(self.coordinate_mapper, 'set_polarity'):
            self.coordinate_mapper.set_polarity(self.x_polarity, self.y_polarity)
        
        polarity_desc = "正常" if self.y_polarity == 1 else "反向"
        logger.info(f"🔄 Y轴极性已切换为: {polarity_desc} ({self.y_polarity})")
        logger.info(f"💡 如果Y轴移动方向错误，可以按Y键调整")
    
    def reset_polarity(self):
        """**新增：重置极性为默认值**"""
        self.x_polarity = self.config.get('x_polarity', -1)
        self.y_polarity = self.config.get('y_polarity', -1)
        
        if hasattr(self.coordinate_mapper, 'set_polarity'):
            self.coordinate_mapper.set_polarity(self.x_polarity, self.y_polarity)
        
        x_desc = "正常" if self.x_polarity == 1 else "反向"
        y_desc = "正常" if self.y_polarity == 1 else "反向"
        logger.info(f"🔄 极性已重置为默认值: X轴={x_desc} ({self.x_polarity}), Y轴={y_desc} ({self.y_polarity})")
    
    def get_current_polarity_status(self):
        """**新增：获取当前极性状态**"""
        if hasattr(self.coordinate_mapper, 'get_polarity_info'):
            return self.coordinate_mapper.get_polarity_info()
        else:
            return {
                'x_polarity': self.x_polarity,
                'y_polarity': self.y_polarity,
                'x_description': '正常' if self.x_polarity == 1 else '反向',
                'y_description': '正常' if self.y_polarity == 1 else '反向'
            }

    def show_status(self):
        """显示系统状态"""
        logger.info("="*60)
        logger.info("系统状态:")
        logger.info(f"  控制方法: 增量角度映射 ({'快速响应' if self.fast_response_mode else '精确控制'})")
        logger.info(f"  控制频率: {1.0/self.control_interval:.0f}Hz")
        logger.info(f"  跟踪状态: {'开启' if self.is_tracking else '关闭'}")
        logger.info(f"  目标状态: {'发现' if self.target_found else '丢失'}")
        logger.info(f"  舵机状态: {'移动中' if self.servo_moving else '静止'}")
        logger.info(f"  处理帧数: {self.frame_count}")
        logger.info(f"  检测次数: {self.detection_count}")
        logger.info(f"  控制次数: {self.control_count}")
        logger.info(f"  激光射击次数: {self.laser_shot_count}")
        
        # **优化参数显示**
        logger.info("性能优化参数:")
        logger.info(f"  控制间隔: {self.control_interval*1000:.0f}ms")
        logger.info(f"  位置容差: {self.position_tolerance}°")
        logger.info(f"  移动超时: {self.move_timeout}s")
        logger.info(f"  角度死区: X={self.config.get('angle_deadzone_x', 0.3)}°, Y={self.config.get('angle_deadzone_y', 0.3)}°")
        
        # **新增：显示极性状态**
        polarity_info = self.get_current_polarity_status()
        logger.info("极性配置:")
        logger.info(f"  X轴极性: {polarity_info['x_description']} ({polarity_info['x_polarity']})")
        logger.info(f"  Y轴极性: {polarity_info['y_description']} ({polarity_info['y_polarity']})")
        logger.info("  调整方法: 按X键切换X轴极性，按Y键切换Y轴极性，按R键重置")
        
        # 映射状态
        logger.info("增量角度映射状态:")
        logger.info(f"  成功次数: {self.mapping_success_count}")
        logger.info(f"  失败次数: {self.mapping_failure_count}")
        if self.mapping_success_count + self.mapping_failure_count > 0:
            success_rate = self.mapping_success_count / (self.mapping_success_count + self.mapping_failure_count) * 100
            logger.info(f"  成功率: {success_rate:.1f}%")
        
        # 舵机状态
        x_angle = self.current_x_position * self.angle_per_unit
        y_angle = self.current_y_position * self.angle_per_unit
        center_x_angle = self.center_x_position * self.angle_per_unit
        center_y_angle = self.center_y_position * self.angle_per_unit
        
        logger.info(f"  当前位置: X={x_angle:.1f}°, Y={y_angle:.1f}°")
        logger.info(f"  中心位置: X={center_x_angle:.1f}°, Y={center_y_angle:.1f}°")
        
        # 目标信息
        target_info = self.coordinate_mapper.get_target_info()
        image_center = target_info['image_center']
        target_center = target_info['target_center']
        offset = target_info['offset']
        logger.info(f"  图像中心: ({image_center[0]:.0f}, {image_center[1]:.0f})")
        logger.info(f"  目标中心: ({target_center[0]:.0f}, {target_center[1]:.0f})")
        logger.info(f"  目标偏移: ({offset[0]:+.0f}, {offset[1]:+.0f})")
        
        # **新增：显示速度配置**
        logger.info("速度配置:")
        if self.adaptive_speed:
            logger.info(f"  模式: 自适应速度")
            logger.info(f"  基准速度: {self.servo_speed}")
            logger.info(f"  高速移动(≥{self.high_speed_threshold}°): {self.high_speed}")
            logger.info(f"  中速移动({self.low_speed_threshold}°-{self.high_speed_threshold}°): {self.medium_speed}")
            logger.info(f"  低速移动(<{self.low_speed_threshold}°): {self.low_speed}")
        else:
            logger.info(f"  模式: 固定速度")
            logger.info(f"  当前速度: {self.servo_speed}")
        logger.info(f"  速度范围: {self.min_servo_speed} - {self.max_servo_speed}")
        logger.info(f"  调整步长: {self.speed_step}")
        logger.info("  调整方法: 按+键增加速度，按-键减少速度，按A键切换自适应模式")
        
        logger.info("="*60)

    def check_servo_arrived(self):
        """**优化：快速检查舵机是否到达目标位置**"""
        if self.fast_response_mode:
            return True  # 快速模式下假定立即到达
        
        if self.target_x_angle is None or self.target_y_angle is None:
            return True
        
        try:
            # **优化：减少串口读取频率**
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
        """**优化：高频率跟踪系统主循环**"""
        logger.info("启动方块跟踪系统...")
        
        try:
            # 连接设备
            if not self.connect_devices():
                logger.error("设备连接失败，退出程序")
                return
            
            logger.info("设备连接成功，开始跟踪...")
            logger.info(f"响应模式: {'快速响应' if self.fast_response_mode else '精确控制'}")
            logger.info(f"控制频率: {1.0/self.control_interval:.0f}Hz")
            logger.info(f"目标帧率: {self.target_fps}fps")
            
            # **修复：初始化UI窗口**
            if self.ui.is_enabled():
                logger.info("🖥️ 图像显示已启用 - 将显示YOLO+OpenCV处理结果")
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
            logger.info("  + - 增加舵机速度")
            logger.info("  - - 减少舵机速度")
            logger.info("  a - 切换自适应速度模式")
            if self.ui.is_enabled():
                logger.info("  d - 切换显示模式")
                logger.info("  f - 显示FPS统计")
            logger.info("  q/ESC - 退出程序")
            
            # **优化：预热摄像头**
            logger.info("预热摄像头...")
            for _ in range(5):  # 从10帧减少到5帧
                self.cap.read()
            
            # **新增：初始化UI窗口**
            if self.ui.is_enabled():
                self.ui.initialize_display()
                logger.info(f"✓ 图像显示窗口已创建")
            
            # **新增：FPS统计变量**
            fps_counter = 0
            fps_start_time = time.time()
            current_fps = 0
            
            # **优化：高频率主循环**
            frame_start_time = time.time()
            
            while True:
                loop_start_time = time.time()
                
                # **优化：简化舵机状态检查**
                if self.servo_moving and not self.fast_response_mode:
                    # 只在精确模式下检查舵机状态
                    if self.check_servo_arrived():
                        logger.debug("✓ 舵机已到达目标位置")
                        self.servo_moving = False
                        # **优化：减少图像稳定等待时间**
                        self.wait_for_stable_image(0.05)  # 从0.3秒减少到0.05秒
                        
                    elif time.time() - self.move_start_time > self.move_timeout:
                        logger.warning("⚠️ 舵机移动超时，强制继续")
                        self.servo_moving = False
                    else:
                        # **修复：使用正确的UI方法获取按键**
                        if self.ui.is_enabled():
                            key = self.ui.get_key_input(1)
                            if self.handle_keyboard_input(key):
                                break
                        
                        time.sleep(0.01)  # 从50ms减少到10ms
                        continue
                
                # **步骤2：获取最新图像帧**
                ret, frame = self.get_latest_frame()
                if not ret:
                    logger.error("无法读取摄像头帧")
                    break
                
                # **步骤3：处理最新图像**
                processed_frame = self.process_frame(frame)
                
                # **新增：FPS计算**
                fps_counter += 1
                if time.time() - fps_start_time >= 1.0:
                    current_fps = fps_counter / (time.time() - fps_start_time)
                    fps_counter = 0
                    fps_start_time = time.time()
                
                # **步骤4：显示图像和处理用户输入**
                if self.ui.is_enabled():
                    # **新增：添加FPS显示**
                    if self.config.get('display_fps_counter', True):
                        processed_frame = self.ui.add_fps_overlay(processed_frame)
                    
                    # **修复：使用正确的显示方法并处理键盘输入**
                    key = self.ui.display_frame(processed_frame)
                    if self.handle_keyboard_input(key):
                        break
                else:
                    # 无显示模式 - 自动开始跟踪
                    if not self.is_tracking:
                        self.is_tracking = True
                        logger.info("自动开始跟踪（无显示模式）")
                
                # **优化：减少状态显示频率**
                if time.time() - frame_start_time >= 60:  # 从30秒增加到60秒
                    self.show_status()
                    frame_start_time = time.time()
                
                # **优化：精确的帧率控制**
                loop_time = time.time() - loop_start_time
                sleep_time = self.frame_time - loop_time
                if sleep_time > 0:
                    time.sleep(min(sleep_time, 0.001))  # 最大睡眠1ms
                
        except KeyboardInterrupt:
            logger.info("接收到键盘中断")
        except Exception as e:
            logger.error(f"运行时异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            logger.info("正在清理资源...")
            self.cleanup_and_exit()

    def handle_keyboard_input(self, key):
        """**新增：处理键盘输入，返回True表示需要退出**"""
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
        elif key == ord('+') or key == ord('='):  # +键 - 增加速度
            self.increase_servo_speed()
        elif key == ord('-') or key == ord('_'):  # -键 - 减少速度
            self.decrease_servo_speed()
        elif key == ord('a') or key == ord('A'):  # A键 - 切换自适应速度
            self.toggle_adaptive_speed()
        elif key == ord('d') or key == ord('D'):  # **新增：D键 - 切换显示模式**
            if self.ui.is_enabled():
                self.ui.toggle_display_mode()
        elif key == ord('f') or key == ord('F'):  # **新增：F键 - 显示FPS统计**
            self.show_fps_stats()
        elif key == ord('q') or key == 27:  # q键或ESC - 退出
            logger.info("用户请求退出")
            return True
        
        return False

    def show_fps_stats(self):
        """**新增：显示FPS统计信息**"""
        logger.info("="*50)
        logger.info("FPS 和性能统计:")
        logger.info(f"  目标帧率: {self.target_fps} fps")
        logger.info(f"  控制频率: {1.0/self.control_interval:.0f} Hz")
        logger.info(f"  处理帧数: {self.frame_count}")
        logger.info(f"  检测成功: {self.detection_count}")
        logger.info(f"  控制次数: {self.control_count}")
        
        # 计算效率
        if self.frame_count > 0:
            detection_rate = self.detection_count / self.frame_count * 100
            control_rate = self.control_count / self.frame_count * 100
            logger.info(f"  检测效率: {detection_rate:.1f}%")
            logger.info(f"  控制效率: {control_rate:.1f}%")
        
        # 映射统计
        total_mappings = self.mapping_success_count + self.mapping_failure_count
        if total_mappings > 0:
            mapping_rate = self.mapping_success_count / total_mappings * 100
            logger.info(f"  映射成功率: {mapping_rate:.1f}%")
        
        logger.info("="*50)

def load_config() -> dict:
    """加载配置"""
    return {
        # 标定文件
        'calibration_file': '/root/square/detect/square_servo_calibration.json',
        
        # 角度映射控制参数
        'angle_deadzone_x': 0.3,      # 从0.5度减少到0.3度
        'angle_deadzone_y': 0.3,      # 从0.5度减少到0.3度
        'laser_angle_threshold': 0.8, # 从1.0度减少到0.8度
        
        # **新增：极性配置参数**
        'x_polarity': -1,  # X轴极性：1=正常，-1=反向（默认反向）
        'y_polarity': -1,  # Y轴极性：1=正常，-1=反向（默认反向）
        
        # YOLO配置
        'yolo': {
            'model_path': '/root/square/detect/1.0.bin',
            'conf_thresh': 0.25,
            'iou_thresh': 0.45
        },
        
        # OpenCV配置
        'opencv': {
            'min_contour_area': 100,
            'max_contour_area': 10000,
            'blur_kernel_size': 5,
            'morph_kernel_size': 3
        },
        
        # 视觉处理流程配置
        'enable_opencv_refinement': True,
        'fallback_to_yolo': True,
        
        # 摄像头配置
        'camera_id': 0,
        'camera_width': 640,
        'camera_height': 480,
        'camera': {
            'width': 640,
            'height': 480,
            'fps': 30
        },
        
        # 舵机配置
        'servo_port': '/dev/ttyACM0',
        'servo_baudrate': 1000000,
        'x_servo_id': 1,
        'y_servo_id': 2,
        
        # 激光配置
        'laser_enabled': True,
        'laser_pin': 16,
        
        # 显示配置
        'enable_display': False,
        
        # **修复：确保所有速度参数都是整数**
        'servo_speed': 150,            # 默认舵机速度（整数）
        'min_servo_speed': 100,        # 最小速度（整数）
        'max_servo_speed': 200,        # 最大速度（整数）
        'speed_step': 50,              # 速度调整步长（整数）
        
        # **修复：确保所有自适应速度参数都是整数**
        'adaptive_speed': True,        
        'high_speed_threshold': 3.0,   # 阈值保持浮点数
        'low_speed_threshold': 1.0,    # 阈值保持浮点数
        'high_speed': 200,             # 速度必须是整数
        'medium_speed': 150,           # 速度必须是整数
        'low_speed': 100,              # 速度必须是整数
        
        # **新增：快速响应模式参数**
        'fast_response_mode': True,    # 启用快速响应模式
        'move_timeout': 1.5,           # 移动超时（浮点数）
        'position_tolerance': 1.0,     # 位置容差（浮点数）
        'position_check_interval': 0.02, # 检查间隔（浮点数）
    }

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='方块跟踪控制系统 - YOLO+OpenCV增量角度映射')
    
    parser.add_argument('--calibration-file', type=str, 
                       help='标定文件路径')
    parser.add_argument('--angle-deadzone-x', type=float, help='X轴角度死区（度）')
    parser.add_argument('--angle-deadzone-y', type=float, help='Y轴角度死区（度）')
    parser.add_argument('--laser-angle-threshold', type=float, help='激光射击角度阈值（度）')
    
    # **新增：极性配置参数**
    parser.add_argument('--x-polarity', type=int, choices=[1, -1], 
                       help='X轴极性：1=正常，-1=反向')
    parser.add_argument('--y-polarity', type=int, choices=[1, -1], 
                       help='Y轴极性：1=正常，-1=反向')
    
    parser.add_argument('--camera-id', type=int, help='摄像头ID')
    parser.add_argument('--servo-port', type=str, help='舵机串口')
    parser.add_argument('--servo-baudrate', type=int, help='舵机波特率')
    parser.add_argument('--x-servo-id', type=int, help='X轴舵机ID')
    parser.add_argument('--y-servo-id', type=int, help='Y轴舵机ID')
    
    # **修改：图像显示相关参数**
    parser.add_argument('--enable-display', action='store_true', 
                       help='启用图像显示 - 实时查看YOLO+OpenCV处理结果')
    parser.add_argument('--display-window-name', type=str, default='Square Tracker - YOLO+OpenCV Real-time',
                       help='显示窗口名称')
    parser.add_argument('--no-fps-counter', action='store_true',
                       help='禁用FPS计数显示')
    parser.add_argument('--minimal-display', action='store_true',
                       help='最小化显示模式（仅显示基本信息）')
    
    parser.add_argument('--verbose', '-v', action='store_true', help='详细输出模式')
    
    # **新增：速度控制参数**
    parser.add_argument('--servo-speed', type=int, 
                       help='舵机速度 (100-2000)')
    parser.add_argument('--adaptive-speed', action='store_true', 
                       help='启用自适应速度控制')
    parser.add_argument('--no-adaptive-speed', action='store_true', 
                       help='禁用自适应速度控制')
    parser.add_argument('--high-speed-threshold', type=float, 
                       help='大角度移动阈值(度)')
    parser.add_argument('--low-speed-threshold', type=float, 
                       help='小角度移动阈值(度)')
    
    args = parser.parse_args()
    
    # 加载配置
    config = load_config()
    
    # 更新配置
    if args.calibration_file is not None:
        config['calibration_file'] = args.calibration_file
    if args.angle_deadzone_x is not None:
        config['angle_deadzone_x'] = args.angle_deadzone_x
    if args.angle_deadzone_y is not None:
        config['angle_deadzone_y'] = args.angle_deadzone_y
    if args.laser_angle_threshold is not None:
        config['laser_angle_threshold'] = args.laser_angle_threshold
    
    # **新增：更新极性配置**
    if args.x_polarity is not None:
        config['x_polarity'] = args.x_polarity
    if args.y_polarity is not None:
        config['y_polarity'] = args.y_polarity
    
    if args.camera_id is not None:
        config['camera_id'] = args.camera_id
    if args.servo_port is not None:
        config['servo_port'] = args.servo_port
    if args.servo_baudrate is not None:
        config['servo_baudrate'] = args.servo_baudrate
    if args.x_servo_id is not None:
        config['x_servo_id'] = args.x_servo_id
    if args.y_servo_id is not None:
        config['y_servo_id'] = args.y_servo_id
    
    # **新增：图像显示配置**
    if args.enable_display:
        config['enable_display'] = True
        logger.info("🖥️ 图像显示已启用")
    
    if args.display_window_name:
        config['display_window_name'] = args.display_window_name
    
    if args.no_fps_counter:
        config['display_fps_counter'] = False
        logger.info("🖥️ FPS计数显示已禁用")
    
    if args.minimal_display:
        config['display_detection_info'] = False
        config['display_servo_status'] = False
        config['display_mapping_stats'] = False
        logger.info("🖥️ 最小化显示模式已启用")
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.info("启用详细输出模式")
        
    # **新增：更新速度配置**
    if args.servo_speed is not None:
        config['servo_speed'] = max(100, min(2000, args.servo_speed))
    if args.adaptive_speed:
        config['adaptive_speed'] = True
    if args.no_adaptive_speed:
        config['adaptive_speed'] = False
    if args.high_speed_threshold is not None:
        config['high_speed_threshold'] = args.high_speed_threshold
    if args.low_speed_threshold is not None:
        config['low_speed_threshold'] = args.low_speed_threshold
    
    # 显示配置信息
    logger.info("方块跟踪控制系统配置:")
    logger.info(f"  控制方法: YOLO+OpenCV增量角度映射")
    logger.info(f"  标定文件: {config['calibration_file']}")
    logger.info(f"  角度死区: X={config['angle_deadzone_x']}°, Y={config['angle_deadzone_y']}°")
    logger.info(f"  激光角度阈值: {config['laser_angle_threshold']}°")
    logger.info(f"  舵机波特率: {config['servo_baudrate']}")
    
    # **新增：显示极性配置**
    x_polarity_desc = "正常" if config['x_polarity'] == 1 else "反向"
    y_polarity_desc = "正常" if config['y_polarity'] == 1 else "反向"
    logger.info(f"  极性配置: X轴={x_polarity_desc} ({config['x_polarity']}), Y轴={y_polarity_desc} ({config['y_polarity']})")
    logger.info(f"  运行时调整: 按X键切换X轴极性，按Y键切换Y轴极性")
    
    # **修改：更详细的显示配置信息**
    if config['enable_display']:
        logger.info(f"  🖥️ 图像显示: 启用")
        logger.info(f"     窗口名称: {config.get('display_window_name', 'Square Tracker')}")
        logger.info(f"     FPS计数: {'启用' if config.get('display_fps_counter', True) else '禁用'}")
        logger.info(f"     检测信息: {'显示' if config.get('display_detection_info', True) else '隐藏'}")
        logger.info(f"     舵机状态: {'显示' if config.get('display_servo_status', True) else '隐藏'}")
        logger.info(f"     映射统计: {'显示' if config.get('display_mapping_stats', True) else '隐藏'}")
        logger.info("     运行时控制: D键切换显示模式，F键显示性能统计")
    else:
        logger.info(f"  🖥️ 图像显示: 禁用（无头模式）")
    
    logger.info("  检测流程: YOLO11检测 -> OpenCV精细化 -> 四边形中心点 -> 角度增量映射")
    
    # 创建并运行跟踪器
    try:
        tracker = ModularSquareTracker(config)
        tracker.run()
    except KeyboardInterrupt:
        logger.info("程序被用户中断")
    except Exception as e:
        logger.error(f"程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 确保GPIO资源被清理
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except:
                pass

if __name__ == "__main__":
    main()
