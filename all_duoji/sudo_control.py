import serial
import time  # 确保time模块正确导入
import struct
import threading
import sys
import select
import termios
import tty
from control import ServoController

class VelocityServoController(ServoController):
    """基于官方库的速度+方向控制器 - 支持连续旋转"""
    
    def __init__(self, port='COM3', baudrate=1000000):
        super().__init__(port, baudrate)
        self.velocity_control_active = {}  # {servo_id: bool}
        self.current_velocities = {}       # {servo_id: velocity}
        # 添加命令缓存，避免重复发送相同命令
        self.last_commands = {}            # {servo_id: last_velocity}
        
    def set_continuous_rotation_mode(self, servo_id):
        """
        设置舵机为连续旋转模式 (速度控制模式)
        :param servo_id: 舵机ID
        :return: 成功返回True
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            # 设置为速度控制模式
            success = self.set_servo_mode(servo_id, mode=1)
            if success:
                self.velocity_control_active[servo_id] = True
                self.current_velocities[servo_id] = 0
                print(f"舵机 {servo_id} 已设置为连续旋转模式")
                
                # 设置初始速度为0 (停止)
                self.set_servo_velocity(servo_id, 0)
                return True
            else:
                print(f"设置舵机 {servo_id} 连续旋转模式失败")
                return False
                
        except Exception as e:
            print(f"设置连续旋转模式失败: {e}")
            return False
    
    def set_position_mode(self, servo_id):
        """
        恢复舵机为位置控制模式
        :param servo_id: 舵机ID
        :return: 成功返回True
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            # 先停止旋转
            if servo_id in self.velocity_control_active:
                self.set_servo_velocity(servo_id, 0)
                time.sleep(0.1)
            
            # 设置为位置控制模式
            success = self.set_servo_mode(servo_id, mode=0)
            if success:
                self.velocity_control_active[servo_id] = False
                self.current_velocities[servo_id] = 0
                print(f"舵机 {servo_id} 已恢复为位置控制模式")
                return True
            else:
                print(f"恢复舵机 {servo_id} 位置控制模式失败")
                return False
                
        except Exception as e:
            print(f"恢复位置控制模式失败: {e}")
            return False
    
    def set_servo_velocity(self, servo_id, velocity):
        """
        设置舵机转动速度和方向 (连续旋转模式) - 优化版本
        :param servo_id: 舵机ID
        :param velocity: 速度值 (-4095 到 4095)
                        正数=顺时针, 负数=逆时针, 0=停止
        :return: 成功返回True
        """
        if not self.ser or not self.ser.is_open:
            return False
        
        if servo_id not in self.velocity_control_active or not self.velocity_control_active[servo_id]:
            return False
        
        # 检查是否需要发送命令（避免重复发送相同速度）
        if servo_id in self.last_commands and self.last_commands[servo_id] == velocity:
            return True
        
        try:
            # 限制速度范围
            velocity = max(-4095, min(4095, velocity))
            
            # STS3215速度控制格式
            if velocity >= 0:
                speed_value = velocity
                direction = 0  # 顺时针
            else:
                speed_value = abs(velocity)
                direction = 1  # 逆时针
            
            # 构建速度控制指令
            packet = [
                0xFF, 0xFF,           # 帧头
                servo_id,             # 舵机ID
                0x05,                 # 数据长度
                0x03,                 # 写指令
                0x2E                  # 速度寄存器地址 (连续旋转模式)
            ]
            
            # 添加速度值 (小端序)
            if direction == 1:  # 逆时针
                speed_value |= 0x8000  # 设置最高位为1
            
            packet.extend(struct.pack('<H', speed_value))
            
            # 计算校验和
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # 发送指令 - 优化串口操作
            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))
            
            # 更新记录
            self.current_velocities[servo_id] = velocity
            self.last_commands[servo_id] = velocity
            
            return True
            
        except Exception as e:
            print(f"设置速度失败: {e}")
            return False
    
    def stop_servo(self, servo_id):
        """
        停止舵机旋转
        :param servo_id: 舵机ID
        """
        return self.set_servo_velocity(servo_id, 0)
    
    def stop_all_servos(self):
        """停止所有处于速度控制模式的舵机"""
        for servo_id in self.velocity_control_active:
            if self.velocity_control_active[servo_id]:
                self.stop_servo(servo_id)
    
    def get_servo_velocity(self, servo_id):
        """
        获取舵机当前设置的速度
        :param servo_id: 舵机ID
        :return: 当前速度值
        """
        return self.current_velocities.get(servo_id, 0)
    
    def is_velocity_mode(self, servo_id):
        """
        检查舵机是否处于速度控制模式
        :param servo_id: 舵机ID
        :return: True=速度模式, False=位置模式
        """
        return self.velocity_control_active.get(servo_id, False)
    
    def enable_velocity_mode_for_servo(self, servo_id):
        """
        为指定舵机启用速度控制模式
        :param servo_id: 舵机ID
        :return: 成功返回True
        """
        return self.set_continuous_rotation_mode(servo_id)
    
    def disable_velocity_mode_for_servo(self, servo_id):
        """
        为指定舵机禁用速度控制模式
        :param servo_id: 舵机ID
        :return: 成功返回True
        """
        return self.set_position_mode(servo_id)
    
    def set_velocity_smooth(self, servo_id, target_velocity, acceleration_time=0.5):
        """
        平滑设置速度，避免突变
        :param servo_id: 舵机ID
        :param target_velocity: 目标速度
        :param acceleration_time: 加速时间(秒)
        :return: 成功返回True
        """
        if servo_id not in self.current_velocities:
            self.current_velocities[servo_id] = 0
        
        current_vel = self.current_velocities[servo_id]
        velocity_diff = target_velocity - current_vel
        
        # 如果差异很小，直接设置
        if abs(velocity_diff) < 50:
            return self.set_servo_velocity(servo_id, target_velocity)
        
        # 分步加速
        steps = max(5, int(acceleration_time * 20))  # 20Hz更新频率
        step_size = velocity_diff / steps
        
        for i in range(steps):
            intermediate_vel = current_vel + step_size * (i + 1)
            self.set_servo_velocity(servo_id, int(intermediate_vel))
            time.sleep(acceleration_time / steps)
        
        return True

class VelocityTrackingController:
    """速度跟踪控制器 - 用于目标跟踪"""
    
    def __init__(self, servo_controller: VelocityServoController):
        self.servo_controller = servo_controller
        self.tracking_servos = {}  # {servo_id: {'enabled': bool, 'max_speed': int}}
        self.velocity_mapping = {
            'linear_factor': 10.0,  # 线性映射因子
            'deadzone': 5.0,        # 死区
            'max_velocity': 1000    # 最大速度
        }
        # 添加速度平滑处理
        self.velocity_filter = {}  # {servo_id: filtered_velocity}
        self.filter_factor = 0.7   # 滤波系数
        
    def enable_tracking_for_servo(self, servo_id, max_speed=1000):
        """
        为舵机启用速度跟踪模式
        :param servo_id: 舵机ID
        :param max_speed: 最大跟踪速度
        """
        if self.servo_controller.enable_velocity_mode_for_servo(servo_id):
            self.tracking_servos[servo_id] = {
                'enabled': True,
                'max_speed': max_speed
            }
            return True
        return False
    
    def disable_tracking_for_servo(self, servo_id):
        """
        为舵机禁用速度跟踪模式
        :param servo_id: 舵机ID
        """
        # 先停止
        self.servo_controller.stop_servo(servo_id)
        time.sleep(0.1)
        
        # 恢复位置模式
        if self.servo_controller.disable_velocity_mode_for_servo(servo_id):
            if servo_id in self.tracking_servos:
                del self.tracking_servos[servo_id]
            return True
        return False
    
    def set_velocity_mapping(self, linear_factor=10.0, deadzone=5.0, max_velocity=1000):
        """
        设置速度映射参数
        :param linear_factor: 线性映射因子
        :param deadzone: 死区大小
        :param max_velocity: 最大速度
        """
        self.velocity_mapping.update({
            'linear_factor': linear_factor,
            'deadzone': deadzone,
            'max_velocity': max_velocity
        })
    
    def error_to_velocity(self, error):
        """
        将位置误差转换为速度命令 - 优化版本
        :param error: 位置误差 (像素或角度)
        :return: 速度命令 (-max_velocity 到 +max_velocity)
        """
        # 死区处理
        if abs(error) < self.velocity_mapping['deadzone']:
            return 0
        
        # 非线性映射 - 提供更好的控制响应
        abs_error = abs(error)
        sign = 1 if error > 0 else -1
        
        # 分段映射：小误差用低增益，大误差用高增益
        if abs_error < 50:
            velocity = error * self.velocity_mapping['linear_factor'] * 0.5
        elif abs_error < 100:
            velocity = error * self.velocity_mapping['linear_factor'] * 0.8
        else:
            velocity = error * self.velocity_mapping['linear_factor']
        
        # 限制范围
        max_vel = self.velocity_mapping['max_velocity']
        velocity = max(-max_vel, min(max_vel, velocity))
        
        return int(velocity)
    
    def update_tracking(self, servo_id, position_error):
        """
        根据位置误差更新舵机速度 - 优化版本
        :param servo_id: 舵机ID
        :param position_error: 位置误差
        :return: 成功返回True
        """
        if servo_id not in self.tracking_servos or not self.tracking_servos[servo_id]['enabled']:
            return False
        
        # 转换为速度命令
        target_velocity = self.error_to_velocity(position_error)
        
        # 速度平滑滤波
        if servo_id in self.velocity_filter:
            filtered_velocity = (self.velocity_filter[servo_id] * self.filter_factor + 
                               target_velocity * (1 - self.filter_factor))
            self.velocity_filter[servo_id] = filtered_velocity
            target_velocity = int(filtered_velocity)
        else:
            self.velocity_filter[servo_id] = target_velocity
        
        # 限制最大速度
        max_speed = self.tracking_servos[servo_id]['max_speed']
        target_velocity = max(-max_speed, min(max_speed, target_velocity))
        
        # 设置速度
        return self.servo_controller.set_servo_velocity(servo_id, target_velocity)
    
    def stop_all_tracking(self):
        """停止所有跟踪舵机"""
        for servo_id in self.tracking_servos:
            if self.tracking_servos[servo_id]['enabled']:
                self.servo_controller.stop_servo(servo_id)
    
    def get_tracking_status(self):
        """获取跟踪状态"""
        status = {}
        for servo_id, config in self.tracking_servos.items():
            status[servo_id] = {
                'enabled': config['enabled'],
                'max_speed': config['max_speed'],
                'current_velocity': self.servo_controller.get_servo_velocity(servo_id),
                'is_velocity_mode': self.servo_controller.is_velocity_mode(servo_id)
            }
        return status

class TerminalVelocityController:
    """终端速度控制器 - 实时键盘控制"""
    
    def __init__(self):
        self.controller = VelocityServoController()
        self.connected = False
        self.active_servo = None
        self.running = False
        self.current_speed = 0
        self.speed_step = 100  # 速度调节步长
        self.max_speed = 4095
        
        # 保存终端设置
        self.old_settings = None
        
    def setup_terminal(self):
        """设置终端为原始模式，支持实时按键检测"""
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
    
    def restore_terminal(self):
        """恢复终端设置"""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self):
        """非阻塞获取按键"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
    
    def connect_servo(self):
        """连接舵机"""
        print("=== 连接舵机 ===")
        port = input("请输入串口号 (默认 /dev/ttyACM0): ").strip() or "/dev/ttyACM0"
        
        self.controller.port = port
        self.connected = self.controller.connect()
        
        if self.connected:
            print("连接成功!")
            return True
        else:
            print("连接失败!")
            return False
    
    def setup_servo(self):
        """设置要控制的舵机"""
        if not self.connected:
            print("请先连接舵机")
            return False
        
        try:
            servo_id = int(input("请输入要控制的舵机ID: "))
            
            # 测试舵机连接
            if not self.controller.ping_servo(servo_id):
                print(f"舵机 {servo_id} 无响应")
                return False
            
            # 设置为连续旋转模式
            if self.controller.set_continuous_rotation_mode(servo_id):
                self.active_servo = servo_id
                print(f"舵机 {servo_id} 已准备就绪")
                return True
            else:
                print(f"设置舵机 {servo_id} 失败")
                return False
                
        except ValueError:
            print("输入格式错误")
            return False
    
    def handle_key(self, key):
        """处理按键输入"""
        if key is None:
            return True
        
        key = key.lower()
        
        # 方向控制
        if key == 'w':  # 增加顺时针速度
            new_speed = max(0, self.current_speed) + self.speed_step
            self.update_speed(new_speed)
        elif key == 's':  # 增加逆时针速度
            new_speed = min(0, self.current_speed) - self.speed_step
            self.update_speed(new_speed)
        elif key == 'a':  # 减小当前速度
            if self.current_speed > 0:
                new_speed = max(0, self.current_speed - self.speed_step)
            elif self.current_speed < 0:
                new_speed = min(0, self.current_speed + self.speed_step)
            else:
                new_speed = 0
            self.update_speed(new_speed)
        elif key == 'd':  # 增加当前速度
            if self.current_speed >= 0:
                new_speed = self.current_speed + self.speed_step
            else:
                new_speed = self.current_speed - self.speed_step
            self.update_speed(new_speed)
        elif key == ' ':  # 立即停止
            self.update_speed(0)
        
        # 速度调节
        elif key in ['+', '=']:  # 增大步长
            self.speed_step = min(500, self.speed_step + 50)
            print(f"\n步长增加到: {self.speed_step}")
        elif key in ['-', '_']:  # 减小步长
            self.speed_step = max(50, self.speed_step - 50)
            print(f"\n步长减小到: {self.speed_step}")
        
        # 快速速度设置
        elif key in '123456789':
            percentage = int(key) * 10
            target_speed = int(self.max_speed * percentage / 100)
            if self.current_speed < 0:
                target_speed = -target_speed
            self.update_speed(target_speed)
        elif key == '0':  # 快速停止
            self.update_speed(0)
        
        # 其他功能和退出控制
        elif key in ['h']:  # 帮助
            print()  # 换行
            self.show_help()
        elif key in ['q']:  # 退出实时控制
            print("\n正在退出实时控制模式...")
            print("停止舵机旋转...")
            self.update_speed(0)
            time.sleep(0.5)
            print("退出完成！")
            return False
        elif key in ['e']:  # 紧急停止并退出
            print("\n*** 紧急停止！***")
            self.update_speed(0)
            if self.active_servo:
                self.controller.set_position_mode(self.active_servo)
            print("已恢复位置模式，程序退出！")
            return False
        elif key in ['r']:  # 重置舵机
            print("\n重置舵机...")
            self.update_speed(0)
            time.sleep(0.5)
            self.controller.set_position_mode(self.active_servo)
            time.sleep(0.5)
            self.controller.set_continuous_rotation_mode(self.active_servo)
            print("舵机已重置")
        
        return True
    
    def show_help(self):
        """显示控制帮助"""
        print("\n" + "="*60)
        print("    实时速度控制帮助")
        print("="*60)
        print("方向控制:")
        print("  w/W     - 增加顺时针速度")
        print("  s/S     - 增加逆时针速度")
        print("  a/A     - 减小当前速度")
        print("  d/D     - 增加当前速度")
        print("  空格键   - 立即停止")
        print("")
        print("速度调节:")
        print("  +/=     - 增大速度步长")
        print("  -/_     - 减小速度步长")
        print("  1-9     - 快速设置速度 (10%-90%)")
        print("  0       - 快速停止")
        print("")
        print("退出控制:")
        print("  q/Q     - 安全退出实时控制")
        print("  e/E     - 紧急停止并退出")
        print("  Ctrl+C  - 强制中断程序")
        print("")
        print("其他功能:")
        print("  h/H     - 显示此帮助")
        print("  r/R     - 重置舵机")
        print("="*60)
        print(f"当前舵机: {self.active_servo}")
        print(f"当前速度: {self.current_speed}")
        print(f"速度步长: {self.speed_step}")
        print("="*60)
        print("💡 提示: 按 'q' 安全退出，按 'e' 紧急停止")
    
    def update_speed(self, new_speed):
        """更新舵机速度"""
        if self.active_servo is None:
            return
        
        # 限制速度范围
        new_speed = max(-self.max_speed, min(self.max_speed, new_speed))
        
        if self.controller.set_servo_velocity(self.active_servo, new_speed):
            self.current_speed = new_speed
            
            # 显示状态
            direction = ""
            if new_speed > 0:
                direction = "顺时针"
            elif new_speed < 0:
                direction = "逆时针"
            else:
                direction = "停止"
            
            print(f"\r速度: {abs(new_speed):4d} ({direction:4s}) | 步长: {self.speed_step:3d} | 按h查看帮助", end="", flush=True)
    
    def run_realtime_control(self):
        """运行实时控制"""
        if not self.connected or self.active_servo is None:
            print("请先连接并设置舵机")
            return
        
        print("开始实时速度控制...")
        print("💡 按 'h' 查看控制帮助，按 'q' 安全退出，按 'e' 紧急停止")
        
        try:
            self.setup_terminal()
            self.running = True
            self.current_speed = 0
            
            # 显示初始状态
            self.show_help()
            
            while self.running:
                key = self.get_key()
                if not self.handle_key(key):
                    break
                
                time.sleep(0.01)  # 小延时，避免CPU占用过高
                
        except KeyboardInterrupt:
            print("\n*** 收到 Ctrl+C 中断信号 ***")
            print("正在安全停止舵机...")
            if self.active_servo:
                self.controller.stop_servo(self.active_servo)
                time.sleep(0.5)
                self.controller.set_position_mode(self.active_servo)
            print("舵机已停止并恢复位置模式")
        finally:
            # 停止舵机并恢复终端
            if self.active_servo:
                print("正在执行安全退出流程...")
                self.controller.stop_servo(self.active_servo)
                time.sleep(0.2)
            
            self.restore_terminal()
            self.running = False
            print("实时控制已安全结束")
    
    def show_main_menu(self):
        """显示主菜单"""
        print("\n" + "="*50)
        print("    舵机速度+方向控制系统")
        print("="*50)
        print("1. 连接舵机")
        print("2. 设置控制舵机")
        print("3. 开始实时控制")
        print("4. 批量速度控制")
        print("5. 舵机状态查询")
        print("6. 恢复位置模式")
        print("0. 退出程序")  # 明确标明这是退出选项
        print("="*50)
        
        if self.connected:
            print(f"连接状态: 已连接 ({self.controller.port})")
        else:
            print("连接状态: 未连接")
        
        if self.active_servo:
            print(f"活动舵机: {self.active_servo}")
            print(f"当前速度: {self.current_speed}")
        
        print("="*50)
        print("💡 提示: 输入 '0' 或 'Ctrl+C' 可退出程序")
    
    def safe_exit(self):
        """安全退出程序"""
        print("\n正在安全退出程序...")
        
        # 停止所有舵机
        if self.connected:
            print("正在停止所有舵机...")
            try:
                self.controller.stop_all_servos()
                time.sleep(0.5)
                
                # 如果有活动舵机，恢复其位置模式
                if self.active_servo:
                    print(f"正在恢复舵机 {self.active_servo} 为位置模式...")
                    self.controller.set_position_mode(self.active_servo)
                    
                # 断开连接
                self.controller.disconnect()
                print("所有舵机已安全停止")
            except Exception as e:
                print(f"停止舵机时出现错误: {e}")
        
        print("程序已安全退出！谢谢使用！")
    
    def run(self):
        """运行主程序"""
        print("欢迎使用舵机速度+方向控制系统!")
        print("基于官方库的连续旋转控制")
        print("💡 随时可按 Ctrl+C 安全退出程序")
        
        try:
            while True:
                self.show_main_menu()
                choice = input("请选择操作 (0-6): ").strip()
                
                if choice == '1':
                    self.connect_servo()
                elif choice == '2':
                    self.setup_servo()
                elif choice == '3':
                    self.run_realtime_control()
                elif choice == '4':
                    self.batch_velocity_control()
                elif choice == '5':
                    self.query_servo_status()
                elif choice == '6':
                    self.restore_position_mode()
                elif choice == '0':
                    # 正常退出
                    self.safe_exit()
                    break
                elif choice.lower() in ['exit', 'quit', 'q']:
                    # 支持额外的退出命令
                    self.safe_exit()
                    break
                else:
                    print("无效选择，请重新输入")
                    print("💡 输入 '0' 退出程序")
                
                if choice not in ['3', '0']:  # 实时控制和退出不需要暂停
                    input("\n按回车键继续...")
                    
        except KeyboardInterrupt:
            print("\n\n*** 收到 Ctrl+C 信号 ***")
            self.safe_exit()
        except Exception as e:
            print(f"\n程序异常: {e}")
            self.safe_exit()

def test_velocity_control():
    """测试速度控制功能"""
    print("测试速度控制功能...")
    
    controller = VelocityServoController("/dev/ttyACM0", 1000000)
    
    if controller.connect():
        servo_id = int(input("请输入要测试的舵机ID: "))
        
        if controller.ping_servo(servo_id):
            print("设置为连续旋转模式...")
            if controller.set_continuous_rotation_mode(servo_id):
                
                # 测试序列
                test_speeds = [0, 500, 1000, 2000, 0, -500, -1000, -2000, 0]
                
                for speed in test_speeds:
                    print(f"测试速度: {speed}")
                    controller.set_servo_velocity(servo_id, speed)
                    time.sleep(3)
                
                print("测试完成，恢复位置模式...")
                controller.set_position_mode(servo_id)
            else:
                print("设置连续旋转模式失败")
        else:
            print("舵机无响应")
        
        controller.disconnect()
    else:
        print("连接失败")

# 模块级别的便捷函数
def create_velocity_controller(port='/dev/ttyACM0', baudrate=1000000):
    """
    创建速度控制器实例
    :param port: 串口
    :param baudrate: 波特率
    :return: VelocityServoController实例
    """
    return VelocityServoController(port, baudrate)

def create_tracking_controller(velocity_controller):
    """
    创建跟踪控制器实例
    :param velocity_controller: 速度控制器实例
    :return: VelocityTrackingController实例
    """
    return VelocityTrackingController(velocity_controller)

if __name__ == "__main__":
    # 保持原有的命令行界面
    print("请选择模式:")
    print("1. 终端实时控制")
    print("2. 速度控制功能测试")
    print("💡 提示: 随时可按 Ctrl+C 退出程序")
    
    try:
        choice = input("请输入选择 (1/2): ").strip()
        
        if choice == '1':
            controller = TerminalVelocityController()
            controller.run()
        elif choice == '2':
            test_velocity_control()
        else:
            print("无效选择，程序退出")
    except KeyboardInterrupt:
        print("\n程序被用户中断，退出")
    except Exception as e:
        print(f"程序异常: {e}")
