import os
import sys
import json
import time
import logging
import argparse
from typing import Dict, Tuple, Optional, Any, List

# 添加模块路径
sys.path.append('/root/square/detect')
sys.path.append('/root/square/all_duoji')

# 添加GPIO控制
try:
    import Hobot.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

# 配置日志
logger = logging.getLogger("SquareTrackerFunction")

def load_default_config() -> Dict[str, Any]:
    """加载默认配置"""
    return {
        # 标定文件
        'calibration_file': '/root/square/detect/square_servo_calibration.json',
        
        # 角度映射控制参数
        'angle_deadzone_x': 0.2,  # 减小死区，提高精度
        'angle_deadzone_y': 0.2,  # 减小死区，提高精度
        'laser_angle_threshold': 0.5,  # 减小激光阈值，提高射击精度
        
        # 极性配置参数
        'x_polarity': -1,  # X轴极性：1=正常，-1=反向（默认反向）
        'y_polarity': -1,   # Y轴极性：1=正常，-1=反向（默认正常）
        
        # **新增：D控制（微分控制）配置参数**
        'd_control_enabled': True,              # D控制总开关（默认关闭）
        'd_control_acceleration_threshold': 100,  # 加速度阈值（像素/秒²）
        'd_control_prediction_factor': 0.5,      # 预测因子（0.1-0.5）
        'd_control_min_delta_angle': 0.2,        # 最小预测角度（度）
        'd_control_max_delta_angle': 15.0,        # 最大预测角度（度）
        'd_control_velocity_smooth_factor': 0.7, # 速度平滑因子
        'd_control_debug': False,                # D控制调试输出
        
        # YOLO配置 - 纯检测方案优化，调高阈值
        'yolo': {
            'model_path': '/root/square/detect/2.0.bin',
            'conf_thresh': 0.25,  # 调高置信度阈值，减少误检
            'iou_thresh': 0.50,   # 调高IoU阈值，提高检测质量
            'min_aspect_ratio': 0.5,  # 保持长宽比筛选
            'max_aspect_ratio': 2.0   # 保持长宽比筛选
        },
        
        # OpenCV配置 - 已禁用
        'opencv': {
            'min_contour_area': 200,
            'max_contour_area': 8000,
            'blur_kernel_size': 5,
            'morph_kernel_size': 3
        },
        
        # 视觉处理流程配置 - 纯检测方案
        'enable_opencv_refinement': False,  # 禁用OpenCV精细化
        'fallback_to_yolo': True,           # 始终使用YOLO结果
        'bbox_expansion': 0,                # 不需要扩展检测框
        
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
        'display_window_name': 'Square Tracker - Pure YOLO Detection',  # 更新窗口名称
        'display_fps_counter': True,
        'display_detection_info': True,
        'display_servo_status': True,
        'display_mapping_stats': True,
        
        # 简化速度控制参数 - 支持多种速度模式
        'servo_speed': 30000,          # 默认速度（保持兼容性）
        'servo_speed_normal': 30000,   # 正常模式速度（2号、3号指令）
        'servo_speed_high': 32000,     # 高速模式速度（4号、5号指令）
        'min_servo_speed': 30000,
        'max_servo_speed': 32000,
        'speed_step': 0,  # 不再需要
        
        # 禁用自适应速度参数
        'adaptive_speed': False,
        'high_speed_threshold': 0,
        'low_speed_threshold': 0,
        'high_speed': 32000,              # 高速（对应4号5号指令）
        'medium_speed': 30000,            # 中速
        'low_speed': 30000,               # 低速
        
        # 快速响应模式参数
        'fast_response_mode': True,
        'move_timeout': 0.8,
        'position_tolerance': 0.5,
        'position_check_interval': 0.01,
        
        # 搜索模式配置 - 使用正常速度
        'search_speed': 30000,
        'search_step_angle': 2.0,
        'search_pause_time': 0.2,
        'search_max_range': 90.0,
    }

def load_config_from_file(config_file: str) -> Optional[Dict[str, Any]]:
    """从文件加载配置"""
    try:
        if os.path.exists(config_file):
            with open(config_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        else:
            logger.warning(f"配置文件不存在: {config_file}")
            return None
    except Exception as e:
        logger.error(f"加载配置文件失败: {e}")
        return None

def save_config_to_file(config: Dict[str, Any], config_file: str) -> bool:
    """保存配置到文件"""
    try:
        os.makedirs(os.path.dirname(config_file), exist_ok=True)
        with open(config_file, 'w', encoding='utf-8') as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        logger.info(f"配置已保存到: {config_file}")
        return True
    except Exception as e:
        logger.error(f"保存配置文件失败: {e}")
        return False

def merge_configs(base_config: Dict[str, Any], override_config: Dict[str, Any]) -> Dict[str, Any]:
    """合并配置，override_config 覆盖 base_config"""
    merged = base_config.copy()
    
    for key, value in override_config.items():
        if isinstance(value, dict) and key in merged and isinstance(merged[key], dict):
            merged[key] = merge_configs(merged[key], value)
        else:
            merged[key] = value
    
    return merged

def validate_config(config: Dict[str, Any]) -> Tuple[bool, List[str]]:
    """验证配置的有效性"""
    errors = []
    
    # 检查必需的配置项
    required_keys = [
        'calibration_file', 'camera_id', 'servo_port', 'servo_baudrate',
        'x_servo_id', 'y_servo_id', 'laser_pin'
    ]
    
    for key in required_keys:
        if key not in config:
            errors.append(f"缺少必需配置项: {key}")
    
    if 'camera_id' in config:
        cam_id = config['camera_id']
        if not isinstance(cam_id, int) or cam_id < 0:
            errors.append(f"摄像头ID无效: {cam_id}")
    
    # 检查文件路径
    if 'calibration_file' in config:
        cal_file = config['calibration_file']
        if not os.path.exists(cal_file):
            errors.append(f"标定文件不存在: {cal_file}")
    
    # 检查极性值
    for axis in ['x_polarity', 'y_polarity']:
        if axis in config:
            polarity = config[axis]
            if polarity not in [1, -1]:
                errors.append(f"{axis} 必须是 1 或 -1: {polarity}")
    
    return len(errors) == 0, errors

def setup_gpio_laser(laser_pin: int) -> bool:
    """设置激光控制GPIO"""
    if not GPIO_AVAILABLE:
        logger.warning("GPIO模块不可用，激光控制功能将被禁用")
        return False
    
    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(laser_pin, GPIO.OUT, initial=GPIO.LOW)
        logger.info(f"激光控制GPIO初始化成功 - 引脚{laser_pin}")
        return True
    except Exception as e:
        logger.error(f"激光控制GPIO初始化失败: {e}")
        return False

def control_gpio_laser(laser_pin: int, should_fire: bool, current_state: Any = None) -> Tuple[bool, Any]:
    """控制GPIO激光射击"""
    if not GPIO_AVAILABLE:
        return False, current_state
    
    try:
        new_state = GPIO.HIGH if should_fire else GPIO.LOW
        
        if new_state != current_state:
            GPIO.output(laser_pin, new_state)
            current_state = new_state
            
            if new_state == GPIO.HIGH:
                logger.debug("🔴 激光射击")
            else:
                logger.debug("🔵 激光关闭")
        
        return True, current_state
    except Exception as e:
        logger.error(f"激光控制失败: {e}")
        return False, current_state

def cleanup_gpio():
    """清理GPIO资源"""
    if GPIO_AVAILABLE:
        try:
            GPIO.cleanup()
            logger.info("GPIO资源已清理")
        except Exception as e:
            logger.error(f"GPIO清理失败: {e}")

def parse_command_line_args() -> argparse.Namespace:
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='方块跟踪控制系统 - YOLO+OpenCV增量角度映射')
    
    # 基本配置
    parser.add_argument('--config-file', type=str, 
                       help='配置文件路径')
    parser.add_argument('--calibration-file', type=str, 
                       help='标定文件路径')
    parser.add_argument('--angle-deadzone-x', type=float, help='X轴角度死区（度）')
    parser.add_argument('--angle-deadzone-y', type=float, help='Y轴角度死区（度）')
    parser.add_argument('--laser-angle-threshold', type=float, help='激光射击角度阈值（度）')
    
    # 极性配置参数
    parser.add_argument('--x-polarity', type=int, choices=[1, -1], 
                       help='X轴极性：1=正常，-1=反向')
    parser.add_argument('--y-polarity', type=int, choices=[1, -1], 
                       help='Y轴极性：1=正常，-1=反向')
    
    # 设备配置
    parser.add_argument('--camera-id', type=int, help='摄像头ID')
    parser.add_argument('--servo-port', type=str, help='舵机串口')
    parser.add_argument('--servo-baudrate', type=int, help='舵机波特率')
    parser.add_argument('--x-servo-id', type=int, help='X轴舵机ID')
    parser.add_argument('--y-servo-id', type=int, help='Y轴舵机ID')
    
    # 显示配置
    parser.add_argument('--enable-display', action='store_true', 
                       help='启用图像显示 - 实时查看YOLO+OpenCV处理结果')
    parser.add_argument('--display-window-name', type=str, default='Square Tracker - YOLO+OpenCV Real-time',
                       help='显示窗口名称')
    parser.add_argument('--no-fps-counter', action='store_true',
                       help='禁用FPS计数显示')
    parser.add_argument('--minimal-display', action='store_true',
                       help='最小化显示模式（仅显示基本信息）')
    
    # 速度控制参数
    parser.add_argument('--servo-speed', type=int, 
                       help='舵机速度 (100-2000)')
    parser.add_argument('--adaptive-speed', action='store_true', 
                       help='启用自适应速度控制')
    parser.add_argument('--no-adaptive-speed', action='store_true', 
                       help='禁用自适应速度控制')
    parser.add_argument('--high-speed-threshold', type=float, 
                       help='大角度移动阈值(度)')
    parser.add_argument('--medium-speed-threshold', type=float, 
                       help='中角度移动阈值(度)')
    parser.add_argument('--low-speed-threshold', type=float, 
                       help='小角度移动阈值(度)')
    
    # 搜索模式参数
    parser.add_argument('--search-speed', type=int, 
                       help='搜索旋转速度')
    parser.add_argument('--search-step-angle', type=float, 
                       help='每次搜索步进角度（度）')
    parser.add_argument('--search-pause-time', type=float, 
                       help='每步搜索后的暂停时间（秒）')
    parser.add_argument('--search-max-range', type=float, 
                       help='最大搜索范围（度）')
    
    # 其他参数
    parser.add_argument('--verbose', '-v', action='store_true', help='详细输出模式')
    parser.add_argument('--save-config', type=str, help='保存当前配置到文件')
    
    return parser.parse_args()

def apply_command_line_args(config: Dict[str, Any], args: argparse.Namespace) -> Dict[str, Any]:
    """将命令行参数应用到配置中"""
    # 基本配置
    if args.calibration_file is not None:
        config['calibration_file'] = args.calibration_file
    if args.angle_deadzone_x is not None:
        config['angle_deadzone_x'] = args.angle_deadzone_x
    if args.angle_deadzone_y is not None:
        config['angle_deadzone_y'] = args.angle_deadzone_y
    if args.laser_angle_threshold is not None:
        config['laser_angle_threshold'] = args.laser_angle_threshold
    
    # 极性配置
    if args.x_polarity is not None:
        config['x_polarity'] = args.x_polarity
    if args.y_polarity is not None:
        config['y_polarity'] = args.y_polarity
    
    # 设备配置
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
    
    # 显示配置
    if args.enable_display:
        config['enable_display'] = True
    if args.display_window_name:
        config['display_window_name'] = args.display_window_name
    if args.no_fps_counter:
        config['display_fps_counter'] = False
    if args.minimal_display:
        config['display_detection_info'] = False
        config['display_servo_status'] = False
        config['display_mapping_stats'] = False
    
    # 速度配置 - 移除速度限制
    if args.servo_speed is not None:
        config['servo_speed'] = args.servo_speed  # 移除范围限制
    if args.adaptive_speed:
        config['adaptive_speed'] = True
    if args.no_adaptive_speed:
        config['adaptive_speed'] = False
    if args.high_speed_threshold is not None:
        config['high_speed_threshold'] = args.high_speed_threshold
    if args.medium_speed_threshold is not None:
        config['medium_speed_threshold'] = args.medium_speed_threshold
    if args.low_speed_threshold is not None:
        config['low_speed_threshold'] = args.low_speed_threshold
    
    # 搜索模式配置 - 移除搜索速度限制
    if args.search_speed is not None:
        config['search_speed'] = args.search_speed  # 移除范围限制
    if args.search_step_angle is not None:
        config['search_step_angle'] = max(0.1, min(10.0, args.search_step_angle))
    if args.search_pause_time is not None:
        config['search_pause_time'] = max(0.1, min(5.0, args.search_pause_time))
    if args.search_max_range is not None:
        config['search_max_range'] = max(10.0, min(180.0, args.search_max_range))
    
    # 日志级别
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    return config

def log_configuration_info(config: Dict[str, Any]):
    """记录配置信息"""
    logger.info("方块跟踪控制系统配置:")
    logger.info(f"  控制方法: 纯YOLO检测增量角度映射")  # 更新描述
    logger.info(f"  标定文件: {config['calibration_file']}")
    logger.info(f"  角度死区: X={config['angle_deadzone_x']}°, Y={config['angle_deadzone_y']}°")
    logger.info(f"  激光角度阈值: {config['laser_angle_threshold']}°")
    logger.info(f"  舵机波特率: {config['servo_baudrate']}")
    
    # 极性配置
    x_polarity_desc = "正常" if config['x_polarity'] == 1 else "反向"
    y_polarity_desc = "正常" if config['y_polarity'] == 1 else "反向"
    logger.info(f"  极性配置: X轴={x_polarity_desc} ({config['x_polarity']}), Y轴={y_polarity_desc} ({config['y_polarity']})")
    
    # **新增：速度模式配置**
    logger.info(f"  速度配置: 正常模式={config.get('servo_speed_normal', 30000)}, 高速模式={config.get('servo_speed_high', 32000)}")
    
    # 显示配置
    if config['enable_display']:
        logger.info(f"  🖥️ 图像显示: 启用")
        logger.info(f"     窗口名称: {config.get('display_window_name', 'Square Tracker')}")
        logger.info(f"     FPS计数: {'启用' if config.get('display_fps_counter', True) else '禁用'}")
    else:
        logger.info(f"  🖥️ 图像显示: 禁用（无头模式）")
    
    logger.info("  检测流程: YOLO11检测 -> 直接使用检测框中心点 -> 角度增量映射")  # 更新描述
    logger.info("  指令速度: 2号3号指令=30000, 4号5号指令=32000")  # 新增指令速度说明

def format_status_info(tracker_core) -> str:
    """格式化状态信息为字符串"""
    lines = []
    lines.append("="*60)
    lines.append("系统状态:")
    lines.append(f"  控制方法: 增量角度映射 ({'快速响应' if tracker_core.fast_response_mode else '精确控制'})")
    lines.append(f"  控制频率: {1.0/tracker_core.control_interval:.0f}Hz")
    lines.append(f"  跟踪状态: {'开启' if tracker_core.is_tracking else '关闭'}")
    lines.append(f"  目标状态: {'发现' if tracker_core.target_found else '丢失'}")
    lines.append(f"  舵机状态: {'移动中' if tracker_core.servo_moving else '静止'}")
    
    # 处理统计
    lines.append(f"  处理帧数: {tracker_core.frame_count}")
    lines.append(f"  检测次数: {tracker_core.detection_count}")
    lines.append(f"  控制次数: {tracker_core.control_count}")
    lines.append(f"  激光射击次数: {tracker_core.laser_shot_count}")
    
    # 映射状态
    lines.append("增量角度映射状态:")
    lines.append(f"  成功次数: {tracker_core.mapping_success_count}")
    lines.append(f"  失败次数: {tracker_core.mapping_failure_count}")
    if tracker_core.mapping_success_count + tracker_core.mapping_failure_count > 0:
        success_rate = tracker_core.mapping_success_count / (tracker_core.mapping_success_count + tracker_core.mapping_failure_count) * 100
        lines.append(f"  成功率: {success_rate:.1f}%")
    
    lines.append("="*60)
    return "\n".join(lines)

def calculate_fps_stats(frame_count: int, start_time: float) -> Tuple[float, float]:
    """计算FPS统计"""
    current_time = time.time()
    elapsed_time = current_time - start_time
    
    if elapsed_time > 0:
        average_fps = frame_count / elapsed_time
        current_fps = 1.0 / (elapsed_time / max(1, frame_count))
    else:
        average_fps = 0.0
        current_fps = 0.0
    
    return average_fps, current_fps

def create_signal_handler(tracker_core):
    """创建信号处理器"""
    def signal_handler(signum, frame):
        logger.info(f"接收到信号 {signum}，正在优雅退出...")
        tracker_core.cleanup_and_exit()
    
    return signal_handler

def validate_servo_parameters(servo_speed: int, servo_id: int) -> Tuple[bool, str]:
    """验证舵机参数 - 移除速度限制"""
    # 验证速度 - 只检查类型，不限制范围
    if not isinstance(servo_speed, int):
        return False, f"舵机速度必须是整数: {type(servo_speed)}"
    
    # 验证ID
    if not isinstance(servo_id, int):
        return False, f"舵机ID必须是整数: {type(servo_id)}"
    
    if not (1 <= servo_id <= 255):
        return False, f"舵机ID超出范围 (1-255): {servo_id}"
    
    return True, "参数有效"

def ensure_integer_speed_params(config: Dict[str, Any]) -> Dict[str, Any]:
    """确保所有速度参数都是整数"""
    speed_params = [
        'servo_speed', 'min_servo_speed', 'max_servo_speed', 'speed_step',
        'high_speed', 'medium_speed', 'low_speed', 'search_speed'
    ]
    
    for param in speed_params:
        if param in config and not isinstance(config[param], int):
            try:
                config[param] = int(config[param])
                logger.debug(f"转换 {param} 为整数: {config[param]}")
            except (ValueError, TypeError):
                logger.warning(f"无法转换 {param} 为整数，使用默认值")
    
    return config
