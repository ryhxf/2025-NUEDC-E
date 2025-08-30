#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO
import time
import select
import termios
import tty
# 导入舵机控制模块
sys.path.append('/root/square/all_duoji')
from control import ServoController

def signal_handler(signal, frame):
    GPIO.cleanup()
    if 'servo_controller' in globals() and servo_controller:
        servo_controller.disconnect()
    # 恢复终端设置
    if 'old_settings' in globals():
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    sys.exit(0)

# 定义使用的GPIO通道为16
output_pin = 16 # BOARD 编码 16

# 舵机配置
SERVO_ID_1 = 1  # 1号舵机ID (A/D控制)
SERVO_ID_2 = 2  # 2号舵机ID (W/S控制)
SERVO_PORT = "/dev/ttyACM0"  # 舵机串口
SERVO_BAUDRATE = 1000000  # 舵机波特率
ANGLE_STEP = 1.0  # 每次按键的角度步进
SERVO_SPEED = 1000  # 舵机转动速度

# 全局变量
servo_controller = None
servo1_initialized = False
servo2_initialized = False
old_settings = None

def init_servo():
    """初始化舵机控制器"""
    global servo_controller, servo1_initialized, servo2_initialized
    
    servo_controller = ServoController(SERVO_PORT, SERVO_BAUDRATE)
    
    if servo_controller.connect():
        print(f"✓ 舵机连接成功 (端口: {SERVO_PORT})")
        
        # 检测1号舵机
        if servo_controller.ping_servo(SERVO_ID_1):
            print(f"✓ 1号舵机 {SERVO_ID_1} 响应正常 (A/D控制)")
            servo1_initialized = True
        else:
            print(f"✗ 1号舵机 {SERVO_ID_1} 无响应")
            servo1_initialized = False
        
        # 检测2号舵机
        if servo_controller.ping_servo(SERVO_ID_2):
            print(f"✓ 2号舵机 {SERVO_ID_2} 响应正常 (W/S控制)")
            servo2_initialized = True
        else:
            print(f"✗ 2号舵机 {SERVO_ID_2} 无响应")
            servo2_initialized = False
        
        if servo1_initialized or servo2_initialized:
            return True
        else:
            print("✗ 所有舵机都无响应")
            servo_controller.disconnect()
            return False
    else:
        print(f"✗ 舵机连接失败，请检查端口 {SERVO_PORT}")
        return False

def setup_keyboard():
    """设置键盘非阻塞输入"""
    global old_settings
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

def get_keypress():
    """获取按键输入（非阻塞）"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def move_servo_relative(direction):
    """相对移动舵机（增量控制）"""
    global servo_controller, servo1_initialized, servo2_initialized
    
    # 确定要控制的舵机
    if direction in ['a', 'd']:  # A/D控制1号舵机
        servo_id = SERVO_ID_1
        servo_name = "1号舵机"
        if not servo1_initialized:
            print(f"{servo_name}未初始化，无法控制")
            return
    elif direction in ['w', 's']:  # W/S控制2号舵机
        servo_id = SERVO_ID_2
        servo_name = "2号舵机"
        if not servo2_initialized:
            print(f"{servo_name}未初始化，无法控制")
            return
    else:
        return
    
    # 先读取当前角度
    current_angle = servo_controller.read_servo_position(servo_id)
    if current_angle is None:
        print(f"无法读取{servo_name}当前角度")
        return
    
    # 计算目标角度
    if direction in ['w', 'd']:  # 增加角度
        target_angle = current_angle + ANGLE_STEP
    elif direction in ['s', 'a']:  # 减少角度
        target_angle = current_angle - ANGLE_STEP
    else:
        return
    
    # 限制角度范围 0-360
    target_angle = max(0, min(360, target_angle))
    
    if abs(target_angle - current_angle) > 0.1:  # 避免重复设置相同角度
        print(f"{servo_name}移动: {current_angle:.1f}° -> {target_angle:.1f}° ({direction.upper()})")
        success = servo_controller.write_servo_position(servo_id, target_angle, SERVO_SPEED)
        
        if success:
            print(f"✓ {servo_name}移动成功")
        else:
            print(f"✗ {servo_name}移动失败")
    else:
        print(f"{servo_name}角度已达到限制范围 (0-360°)")

def get_current_angle(servo_id):
    """获取指定舵机当前角度"""
    global servo_controller
    
    if servo_controller is None:
        return None
    
    return servo_controller.read_servo_position(servo_id)

def show_controls():
    """显示控制说明"""
    print("\n" + "="*50)
    print("    GPIO激光控制 + 双舵机角度微调系统")
    print("="*50)
    print("GPIO16: 激光器常亮控制")
    print("舵机控制:")
    print("  A/D键 - 控制1号舵机 (A减少/D增加)")
    print("  W/S键 - 控制2号舵机 (W增加/S减少)")
    print("其他功能:")
    print("  R - 读取当前所有舵机角度")
    print("  H - 所有舵机回到中心位置 (90°)")
    print("  1 - 仅1号舵机回到中心")
    print("  2 - 仅2号舵机回到中心")
    print("  + - 增大步进角度")
    print("  - - 减小步进角度")
    print("  CTRL+C - 退出程序")
    print("="*50)

def main():
    global servo_controller, servo1_initialized, servo2_initialized, ANGLE_STEP
    
    print("正在初始化GPIO激光控制和双舵机系统...")
    
    # 设置管脚编码模式为硬件编号 BOARD
    GPIO.setmode(GPIO.BOARD)
    # 设置为输出模式，并且初始化为高电平
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    print("✓ GPIO16 设置为高电平常亮!")
    
    # 初始化舵机
    init_servo()
    if not (servo1_initialized or servo2_initialized):
        print("⚠ 舵机初始化失败，但GPIO激光控制仍可用")
    
    # 设置键盘输入
    setup_keyboard()
    
    # 显示控制说明
    show_controls()
    
    try:
        # 获取初始角度
        servo1_angle = get_current_angle(SERVO_ID_1) if servo1_initialized else None
        servo2_angle = get_current_angle(SERVO_ID_2) if servo2_initialized else None
        
        print(f"\n系统就绪! 激光器已启动")
        if servo1_angle is not None:
            print(f"1号舵机角度: {servo1_angle:.1f}° (A/D控制)")
        else:
            print("1号舵机: 未连接")
            
        if servo2_angle is not None:
            print(f"2号舵机角度: {servo2_angle:.1f}° (W/S控制)")
        else:
            print("2号舵机: 未连接")
            
        print(f"当前步进角度: {ANGLE_STEP:.1f}°")
        print("使用A/D控制1号舵机，W/S控制2号舵机，按CTRL+C退出...")
        
        while True:
            # 检查键盘输入
            key = get_keypress()
            
            if key:
                key_lower = key.lower()
                
                if key_lower in ['w', 'a', 's', 'd']:
                    move_servo_relative(key_lower)
                
                elif key_lower == 'r':  # 读取所有角度
                    print("当前舵机角度:")
                    if servo1_initialized:
                        angle1 = get_current_angle(SERVO_ID_1)
                        print(f"  1号舵机: {angle1:.1f}°" if angle1 is not None else "  1号舵机: 读取失败")
                    else:
                        print("  1号舵机: 未初始化")
                        
                    if servo2_initialized:
                        angle2 = get_current_angle(SERVO_ID_2)
                        print(f"  2号舵机: {angle2:.1f}°" if angle2 is not None else "  2号舵机: 读取失败")
                    else:
                        print("  2号舵机: 未初始化")
                
                elif key_lower == 'h':  # 所有舵机回到中心
                    print("所有舵机回到中心位置...")
                    if servo1_initialized:
                        success1 = servo_controller.write_servo_position(SERVO_ID_1, 90.0, SERVO_SPEED)
                        print("✓ 1号舵机已回到中心: 90.0°" if success1 else "✗ 1号舵机移动失败")
                    
                    if servo2_initialized:
                        success2 = servo_controller.write_servo_position(SERVO_ID_2, 90.0, SERVO_SPEED)
                        print("✓ 2号舵机已回到中心: 90.0°" if success2 else "✗ 2号舵机移动失败")
                
                elif key == '1':  # 仅1号舵机回中心
                    if servo1_initialized:
                        print("1号舵机回到中心位置...")
                        success = servo_controller.write_servo_position(SERVO_ID_1, 90.0, SERVO_SPEED)
                        print("✓ 1号舵机已回到中心: 90.0°" if success else "✗ 1号舵机移动失败")
                    else:
                        print("1号舵机未初始化")
                
                elif key == '2':  # 仅2号舵机回中心
                    if servo2_initialized:
                        print("2号舵机回到中心位置...")
                        success = servo_controller.write_servo_position(SERVO_ID_2, 90.0, SERVO_SPEED)
                        print("✓ 2号舵机已回到中心: 90.0°" if success else "✗ 2号舵机移动失败")
                    else:
                        print("2号舵机未初始化")
                
                elif key == '+' or key == '=':  # 增大步进
                    ANGLE_STEP = min(10.0, ANGLE_STEP + 0.5)
                    print(f"步进角度增大到: {ANGLE_STEP:.1f}°")
                
                elif key == '-':  # 减小步进
                    ANGLE_STEP = max(0.1, ANGLE_STEP - 0.5)
                    print(f"步进角度减小到: {ANGLE_STEP:.1f}°")
                
                elif key == '\x03':  # Ctrl+C
                    break
                    
                elif key in ['\r', '\n']:  # 回车键 - 显示状态
                    angle1 = get_current_angle(SERVO_ID_1) if servo1_initialized else None
                    angle2 = get_current_angle(SERVO_ID_2) if servo2_initialized else None
                    angle1_status = f"{angle1:.1f}°" if angle1 is not None else "未连接"
                    angle2_status = f"{angle2:.1f}°" if angle2 is not None else "未连接"
                    print(f"状态: GPIO16=高电平(激光开), 1号舵机={angle1_status}, 2号舵机={angle2_status}, 步进={ANGLE_STEP:.1f}°")
                
                else:
                    print(f"未知按键: {repr(key)} (使用H查看帮助)")
            
            # 保持GPIO16为高电平状态
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n收到退出信号...")
    finally:
        print("正在清理资源...")
        GPIO.cleanup()
        if servo_controller:
            servo_controller.disconnect()
        # 恢复终端设置
        if old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("✓ 系统已安全退出")

if __name__=='__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()