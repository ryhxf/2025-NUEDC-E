import serial
import time
import struct

class ServoController:
    """舵机控制类 - 支持 Feetech STS3215"""
    
    def __init__(self, port='COM3', baudrate=1000000):
        """
        初始化舵机控制器
        :param port: 串口号 (Linux: /dev/ttyACM0, Windows: COM3)
        :param baudrate: 波特率 (STS3215 默认: 1000000)
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        # 多圈位置跟踪器 - 解决边界问题的核心
        self.multi_turn_positions = {}  # {servo_id: absolute_position}
        self.last_known_positions = {}  # {servo_id: last_single_turn_position}
        
    def connect(self):
        """连接舵机"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"成功连接到端口 {self.port} (波特率: {self.baudrate})")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("连接已断开")
    
    def angle_to_position(self, angle):
        """
        将角度转换为舵机位置值 - 支持连续角度，不做限制
        :param angle: 角度 (任意值，支持连续角度)
        :return: 位置值 (支持超过4095的连续值)
        """
        # 移除角度限制，支持连续角度
        return int((angle / 360.0) * 4095)
    
    def position_to_angle(self, position):
        """
        将舵机位置值转换为角度 - 支持连续位置
        :param position: 位置值 (任意值，支持连续位置)
        :return: 角度 (支持连续角度)
        """
        # 移除位置限制，支持连续位置
        return (position / 4095.0) * 360
    
    def calculate_checksum(self, packet):
        """计算STS3215校验和"""
        return (~sum(packet[2:]) & 0xFF)
    
    def write_servo_position(self, servo_id, angle, speed=1000):
        """
        控制STS3215舵机转到指定角度 - 支持连续角度
        :param servo_id: 舵机ID
        :param angle: 目标角度 (支持连续角度，不限制在0-360)
        :param speed: 转动速度 (0-4095, 默认1000)
        :return: 成功返回True，失败返回False
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            # 直接转换角度，不做任何限制
            position = self.angle_to_position(angle)
            
            # STS3215 写位置指令格式
            # 指令: FF FF ID LEN CMD ADDR PARAM1 PARAM2 PARAM3 PARAM4 CHK
            packet = [
                0xFF, 0xFF,           # 帧头
                servo_id,             # 舵机ID
                0x07,                 # 数据长度
                0x03,                 # 写指令
                0x2A                  # 目标位置寄存器地址
            ]
            
            # 添加位置和速度参数 (小端序)
            # 注意：这里可能需要处理超过65535的位置值
            if position > 65535:
                # 对于超大位置值，可能需要分段处理或使用特殊协议
                print(f"警告：位置值 {position} 超过16位范围，尝试取模处理")
                position = position % 4096  # 临时方案：取模到单圈范围
            
            packet.extend(struct.pack('<H', position))  # 目标位置 (2字节)
            packet.extend(struct.pack('<H', speed))     # 转动速度 (2字节)
            
            # 计算校验和
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # 清空输入缓冲区并发送指令
            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))
            
            print(f"舵机 {servo_id} 转动到 {angle:.1f}° (位置值: {position}, 速度: {speed})")
            
            # 可选：等待并检查响应
            time.sleep(0.01)
            if self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)
                if len(response) >= 6:
                    # 检查是否有错误响应
                    if response[4] == 0x00:  # 无错误
                        return True
                    else:
                        print(f"舵机响应错误码: {response[4]}")
            
            return True
            
        except Exception as e:
            print(f"写入失败: {e}")
            return False
    
    def read_servo_position(self, servo_id):
        """
        读取STS3215舵机当前角度
        :param servo_id: 舵机ID
        :return: 当前角度，失败返回None
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return None
        
        try:
            # STS3215 读位置指令格式
            packet = [
                0xFF, 0xFF,           # 帧头
                servo_id,             # 舵机ID
                0x04,                 # 数据长度
                0x02,                 # 读指令
                0x38,                 # 当前位置寄存器地址
                0x02                  # 读取字节数
            ]
            
            # 计算校验和
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # 清空输入缓冲区并发送指令
            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))
            
            # 等待响应
            time.sleep(0.02)
            
            if self.ser.in_waiting >= 8:
                response = self.ser.read(self.ser.in_waiting)
                
                # 调试输出
                print(f"原始响应: {[hex(x) for x in response]}")
                
                if len(response) >= 8:
                    # 验证响应格式
                    if response[0] == 0xFF and response[1] == 0xFF and response[2] == servo_id:
                        # 提取位置值 (小端序)
                        position = struct.unpack('<H', response[5:7])[0]
                        angle = self.position_to_angle(position)
                        print(f"舵机 {servo_id} 当前位置值: {position}, 角度: {angle:.1f}°")
                        return angle
                    else:
                        print("响应格式错误")
                else:
                    print("响应数据不完整")
            else:
                print("未收到响应或数据不足")
            
            return None
            
        except Exception as e:
            print(f"读取失败: {e}")
            return None
    
    def set_servo_speed(self, servo_id, speed):
        """
        设置STS3215舵机转动速度
        :param servo_id: 舵机ID
        :param speed: 速度 (0-4095)
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            packet = [
                0xFF, 0xFF,           # 帧头
                servo_id,             # 舵机ID
                0x05,                 # 数据长度
                0x03,                 # 写指令
                0x20                  # 速度寄存器地址
            ]
            
            # 添加速度参数 (小端序)
            packet.extend(struct.pack('<H', speed))
            
            # 计算校验和
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # 发送指令
            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))
            
            print(f"舵机 {servo_id} 速度设置为 {speed}")
            return True
            
        except Exception as e:
            print(f"设置速度失败: {e}")
            return False
    
    def ping_servo(self, servo_id):
        """
        ping舵机，检查连接状态
        :param servo_id: 舵机ID
        :return: 成功返回True
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            packet = [
                0xFF, 0xFF,           # 帧头
                servo_id,             # 舵机ID
                0x02,                 # 数据长度
                0x01                  # PING指令
            ]
            
            # 计算校验和
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # 发送指令
            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))
            
            # 等待响应
            time.sleep(0.01)
            
            if self.ser.in_waiting >= 6:
                response = self.ser.read(self.ser.in_waiting)
                if len(response) >= 6 and response[0] == 0xFF and response[1] == 0xFF:
                    print(f"舵机 {servo_id} PING 成功")
                    return True
            
            print(f"舵机 {servo_id} PING 失败")
            return False
            
        except Exception as e:
            print(f"PING失败: {e}")
            return False
    
    def write_servo_position_with_direction(self, servo_id, angle, speed=1000, direction_control=False):
        """
        控制STS3215舵机转到指定角度（增强版 - 支持方向控制）
        :param servo_id: 舵机ID
        :param angle: 目标角度 (0-360)
        :param speed: 转动速度 (0-4095, 默认1000)
        :param direction_control: 是否启用方向控制
        :return: 成功返回True，失败返回False
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            position = self.angle_to_position(angle)
            
            if direction_control:
                # 先设置转动模式为位置模式，确保可以控制方向
                self.set_servo_mode(servo_id, mode=0)  # 0=位置模式，1=速度模式
            
            # STS3215 写位置指令格式
            packet = [
                0xFF, 0xFF,           # 帧头
                servo_id,             # 舵机ID
                0x07,                 # 数据长度
                0x03,                 # 写指令
                0x2A                  # 目标位置寄存器地址
            ]
            
            # 添加位置和速度参数 (小端序)
            packet.extend(struct.pack('<H', position))  # 目标位置 (2字节)
            packet.extend(struct.pack('<H', speed))     # 转动速度 (2字节)
            
            # 计算校验和
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # 清空输入缓冲区并发送指令
            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))
            
            print(f"舵机 {servo_id} 转动到 {angle:.1f}° (位置值: {position}, 速度: {speed})")
            return True
            
        except Exception as e:
            print(f"写入失败: {e}")
            return False
    
    def set_servo_mode(self, servo_id, mode=0):
        """
        设置舵机工作模式
        :param servo_id: 舵机ID
        :param mode: 0=位置模式, 1=速度模式, 2=PWM模式
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            packet = [
                0xFF, 0xFF,           # 帧头
                servo_id,             # 舵机ID
                0x04,                 # 数据长度
                0x03,                 # 写指令
                0x21,                 # 模式寄存器地址
                mode                  # 模式值
            ]
            
            # 计算校验和
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # 发送指令
            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))
            
            print(f"舵机 {servo_id} 模式设置为 {mode} ({'位置' if mode==0 else '速度' if mode==1 else 'PWM'})")
            return True
            
        except Exception as e:
            print(f"设置模式失败: {e}")
            return False
    
    def set_servo_direction(self, servo_id, direction=0):
        """
        设置舵机转动方向
        :param servo_id: 舵机ID  
        :param direction: 0=正常方向, 1=反向
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            # STS3215可能支持方向设置，尝试写入方向寄存器
            packet = [
                0xFF, 0xFF,           # 帧头
                servo_id,             # 舵机ID
                0x04,                 # 数据长度
                0x03,                 # 写指令
                0x28,                 # 方向寄存器地址（需要查阅具体手册）
                direction             # 方向值
            ]
            
            # 计算校验和
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # 发送指令
            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))
            
            print(f"舵机 {servo_id} 方向设置为 {'正常' if direction==0 else '反向'}")
            return True
            
        except Exception as e:
            print(f"设置方向失败: {e}")
            return False
    
    def write_servo_position_smart(self, servo_id, target_angle, speed=1000, force_direction=None):
        """
        智能舵机位置控制 - 避免反向转动
        :param servo_id: 舵机ID
        :param target_angle: 目标角度 (0-360)
        :param speed: 转动速度
        :param force_direction: 强制方向 ('cw'=顺时针, 'ccw'=逆时针, None=自动选择)
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            # 获取当前位置
            current_angle = self.read_servo_position(servo_id)
            if current_angle is None:
                print("无法读取当前位置，使用普通模式")
                return self.write_servo_position(servo_id, target_angle, speed)
            
            # 计算角度差
            angle_diff = target_angle - current_angle
            
            # 处理跨越边界的情况
            if angle_diff > 180:
                angle_diff -= 360
            elif angle_diff < -180:
                angle_diff += 360
            
            print(f"当前角度: {current_angle:.1f}°, 目标角度: {target_angle:.1f}°, 角度差: {angle_diff:.1f}°")
            
            # 根据角度差和强制方向决定路径
            if force_direction == 'cw':
                # 强制顺时针
                if angle_diff < 0:
                    # 需要顺时针转动，但角度差为负，需要走长路径
                    intermediate_angle = target_angle + 180
                    if intermediate_angle >= 360:
                        intermediate_angle -= 360
                    
                    print(f"强制顺时针: 先到中间点 {intermediate_angle:.1f}°")
                    self.write_servo_position(servo_id, intermediate_angle, speed)
                    time.sleep(abs(angle_diff + 180) / 360 * 2)  # 估算时间
            
            elif force_direction == 'ccw':
                # 强制逆时针
                if angle_diff > 0:
                    # 需要逆时针转动，但角度差为正，需要走长路径
                    intermediate_angle = target_angle - 180
                    if intermediate_angle < 0:
                        intermediate_angle += 360
                    
                    print(f"强制逆时针: 先到中间点 {intermediate_angle:.1f}°")
                    self.write_servo_position(servo_id, intermediate_angle, speed)
                    time.sleep(abs(angle_diff - 180) / 360 * 2)  # 估算时间
            
            # 移动到最终目标
            return self.write_servo_position(servo_id, target_angle, speed)
            
        except Exception as e:
            print(f"智能控制失败: {e}")
            return False
    
    def write_servo_position_continuous(self, servo_id, target_angle, current_angle=None, speed=1000, step_size=5):
        """
        连续移动舵机到目标角度，避免绕大弯
        :param servo_id: 舵机ID
        :param target_angle: 目标角度 (0-360)
        :param current_angle: 当前角度，如果为None则自动读取
        :param speed: 转动速度
        :param step_size: 步进大小
        :return: 成功返回True
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            # 获取当前角度
            if current_angle is None:
                current_angle = self.read_servo_position(servo_id)
                if current_angle is None:
                    print("无法读取当前位置，使用普通模式")
                    return self.write_servo_position(servo_id, target_angle, speed)
            
            # 规范化角度到0-360范围
            current_angle = current_angle % 360
            target_angle = target_angle % 360
            
            print(f"连续移动: {current_angle:.1f}° -> {target_angle:.1f}°")
            
            # 计算角度差
            angle_diff = target_angle - current_angle
            
            # 处理跨边界情况
            if abs(angle_diff) > 180:
                if angle_diff > 0:
                    # 从当前角度增大到360，然后从0到目标
                    print("跨边界路径: 增大方向")
                    
                    # 第一段：current -> 360
                    current_pos = current_angle
                    while current_pos < 360 and self.ser and self.ser.is_open:
                        next_pos = min(360, current_pos + step_size)
                        self.write_servo_position(servo_id, next_pos, speed)
                        time.sleep(0.05)
                        current_pos = next_pos
                    
                    # 小延时确保到达边界
                    time.sleep(0.1)
                    
                    # 第二段：0 -> target
                    current_pos = 0
                    self.write_servo_position(servo_id, 0, speed)
                    time.sleep(0.1)
                    
                    while current_pos < target_angle and self.ser and self.ser.is_open:
                        next_pos = min(target_angle, current_pos + step_size)
                        self.write_servo_position(servo_id, next_pos, speed)
                        time.sleep(0.05)
                        current_pos = next_pos
                else:
                    # 从当前角度减小到0，然后从360到目标
                    print("跨边界路径: 减小方向")
                    
                    # 第一段：current -> 0
                    current_pos = current_angle
                    while current_pos > 0 and self.ser and self.ser.is_open:
                        next_pos = max(0, current_pos - step_size)
                        self.write_servo_position(servo_id, next_pos, speed)
                        time.sleep(0.05)
                        current_pos = next_pos
                    
                    # 小延时确保到达边界
                    time.sleep(0.1)
                    
                    # 第二段：360 -> target
                    current_pos = 360
                    self.write_servo_position(servo_id, 360, speed)
                    time.sleep(0.1)
                    
                    while current_pos > target_angle and self.ser and self.ser.is_open:
                        next_pos = max(target_angle, current_pos - step_size)
                        self.write_servo_position(servo_id, next_pos, speed)
                        time.sleep(0.05)
                        current_pos = next_pos
            else:
                # 直接路径，分步移动
                print("直接路径")
                current_pos = current_angle
                direction = 1 if angle_diff > 0 else -1
                
                while abs(current_pos - target_angle) > step_size and self.ser and self.ser.is_open:
                    current_pos += step_size * direction
                    self.write_servo_position(servo_id, current_pos, speed)
                    time.sleep(0.05)
                
                # 最后移动到精确位置
                self.write_servo_position(servo_id, target_angle, speed)
            
            print(f"连续移动完成: {target_angle:.1f}°")
            return True
            
        except Exception as e:
            print(f"连续移动失败: {e}")
            return False
    
    def get_multi_turn_position(self, servo_id):
        """
        获取多圈绝对位置 - 官方库风格的边界问题解决方案
        :param servo_id: 舵机ID
        :return: 多圈绝对角度 (可以超过360°)
        """
        current_angle = self.read_servo_position(servo_id)
        if current_angle is None:
            return None
        
        # 初始化位置跟踪器
        if servo_id not in self.multi_turn_positions:
            self.multi_turn_positions[servo_id] = current_angle
            self.last_known_positions[servo_id] = current_angle
            return current_angle
        
        last_pos = self.last_known_positions[servo_id]
        current_pos = current_angle
        
        # 检测跨边界
        position_diff = current_pos - last_pos
        
        if position_diff > 180:
            # 从高角度跳到低角度 (逆时针跨边界)
            self.multi_turn_positions[servo_id] -= 360
        elif position_diff < -180:
            # 从低角度跳到高角度 (顺时针跨边界)
            self.multi_turn_positions[servo_id] += 360
        
        # 更新绝对位置
        self.multi_turn_positions[servo_id] += (current_pos - last_pos)
        self.last_known_positions[servo_id] = current_pos
        
        return self.multi_turn_positions[servo_id]
    
    def write_servo_position_official_style(self, servo_id, target_angle, speed=1000, force_direction=None):
        """
        官方库风格的舵机位置控制 - 彻底解决边界问题
        :param servo_id: 舵机ID
        :param target_angle: 目标角度 (0-360)
        :param speed: 转动速度
        :param force_direction: 强制方向 ('shortest', 'cw', 'ccw')
        :return: 成功返回True
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            # 获取当前多圈位置
            current_multi_turn = self.get_multi_turn_position(servo_id)
            if current_multi_turn is None:
                print("无法读取当前位置")
                return False
            
            # 计算目标的多圈位置
            current_single_turn = current_multi_turn % 360
            target_single_turn = target_angle % 360
            
            # 根据方向策略计算最终目标位置
            if force_direction == 'shortest' or force_direction is None:
                # 最短路径算法 (官方库默认行为)
                diff = target_single_turn - current_single_turn
                
                if diff > 180:
                    # 目标在右边但距离>180°，走左边更短
                    target_multi_turn = current_multi_turn - (360 - diff)
                elif diff < -180:
                    # 目标在左边但距离>180°，走右边更短
                    target_multi_turn = current_multi_turn + (360 + diff)
                else:
                    # 直接路径最短
                    target_multi_turn = current_multi_turn + diff
                    
            elif force_direction == 'cw':
                # 强制顺时针
                if target_single_turn >= current_single_turn:
                    target_multi_turn = current_multi_turn + (target_single_turn - current_single_turn)
                else:
                    target_multi_turn = current_multi_turn + (360 - current_single_turn + target_single_turn)
                    
            elif force_direction == 'ccw':
                # 强制逆时针
                if target_single_turn <= current_single_turn:
                    target_multi_turn = current_multi_turn - (current_single_turn - target_single_turn)
                else:
                    target_multi_turn = current_multi_turn - (current_single_turn + 360 - target_single_turn)
            
            print(f"位置规划: {current_multi_turn:.1f}° -> {target_multi_turn:.1f}° (单圈: {target_single_turn:.1f}°)")
            
            # 分段移动策略
            return self._execute_multi_turn_movement(servo_id, target_multi_turn, speed)
            
        except Exception as e:
            print(f"官方风格控制失败: {e}")
            return False
    
    def _execute_multi_turn_movement(self, servo_id, target_multi_turn, speed=1000):
        """
        执行多圈移动 - 官方库的核心算法
        :param servo_id: 舵机ID
        :param target_multi_turn: 目标多圈位置
        :param speed: 速度
        :return: 成功返回True
        """
        current_multi_turn = self.get_multi_turn_position(servo_id)
        if current_multi_turn is None:
            return False
        
        total_movement = target_multi_turn - current_multi_turn
        
        # 如果移动距离小于等于360°，直接移动
        if abs(total_movement) <= 360:
            final_single_turn = target_multi_turn % 360
            success = self.write_servo_position(servo_id, final_single_turn, speed)
            if success:
                # 更新多圈位置跟踪器
                self.multi_turn_positions[servo_id] = target_multi_turn
                self.last_known_positions[servo_id] = final_single_turn
            return success
        
        # 大角度移动，分段执行
        print(f"大角度移动 {total_movement:.1f}°，分段执行")
        
        # 计算分段点
        segments = int(abs(total_movement) / 350) + 1  # 每段最多350°
        step_size = total_movement / segments
        
        current_pos = current_multi_turn
        
        for i in range(segments):
            if i == segments - 1:
                # 最后一段，移动到精确目标
                next_pos = target_multi_turn
            else:
                next_pos = current_pos + step_size
            
            next_single_turn = next_pos % 360
            
            print(f"分段 {i+1}/{segments}: {current_pos:.1f}° -> {next_pos:.1f}° (单圈: {next_single_turn:.1f}°)")
            
            success = self.write_servo_position(servo_id, next_single_turn, speed)
            if not success:
                print(f"分段移动失败: 段 {i+1}")
                return False
            
            # 等待舵机到达位置
            estimated_time = abs(step_size) / 360 * 2  # 估算时间
            time.sleep(max(0.1, min(2.0, estimated_time)))
            
            # 更新位置
            current_pos = next_pos
            self.multi_turn_positions[servo_id] = current_pos
            self.last_known_positions[servo_id] = next_single_turn
        
        print("分段移动完成")
        return True
    
    def calibrate_servo_position(self, servo_id):
        """
        校准舵机位置 - 重置多圈计数器
        :param servo_id: 舵机ID
        """
        current_angle = self.read_servo_position(servo_id)
        if current_angle is not None:
            self.multi_turn_positions[servo_id] = current_angle
            self.last_known_positions[servo_id] = current_angle
            print(f"舵机 {servo_id} 位置已校准为 {current_angle:.1f}°")
        else:
            print(f"舵机 {servo_id} 校准失败")
    
    def set_servo_home_position(self, servo_id, home_angle=180):
        """
        设置舵机零点位置
        :param servo_id: 舵机ID  
        :param home_angle: 零点角度 (默认180°中心位置)
        """
        success = self.write_servo_position(servo_id, home_angle, 1000)
        if success:
            # 重置多圈计数器
            self.multi_turn_positions[servo_id] = home_angle
            self.last_known_positions[servo_id] = home_angle
            print(f"舵机 {servo_id} 零点设置为 {home_angle:.1f}°")
        return success
    
    def get_servo_status(self, servo_id):
        """
        获取舵机完整状态信息
        :param servo_id: 舵机ID
        :return: 状态字典
        """
        status = {
            'servo_id': servo_id,
            'single_turn_position': self.read_servo_position(servo_id),
            'multi_turn_position': self.get_multi_turn_position(servo_id),
            'connected': False
        }
        
        # 检查连接状态
        if self.ping_servo(servo_id):
            status['connected'] = True
        
        return status
    
    def move_servo_relative(self, servo_id, relative_angle, speed=1000):
        """
        相对位置移动 - 基于当前位置的偏移
        :param servo_id: 舵机ID
        :param relative_angle: 相对角度 (正数顺时针，负数逆时针)
        :param speed: 速度
        :return: 成功返回True
        """
        current_multi_turn = self.get_multi_turn_position(servo_id)
        if current_multi_turn is None:
            return False
        
        target_multi_turn = current_multi_turn + relative_angle
        target_single_turn = target_multi_turn % 360
        
        print(f"相对移动: {relative_angle:+.1f}° -> 目标 {target_single_turn:.1f}°")
        
        return self._execute_multi_turn_movement(servo_id, target_multi_turn, speed)
    
    def write_servo_velocity(self, servo_id, velocity):
        """
        控制STS3215舵机速度模式
        :param servo_id: 舵机ID
        :param velocity: 目标速度 (-4095到4095, 正数顺时针，负数逆时针，0停止)
        :return: 成功返回True，失败返回False
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return False
        
        try:
            # 首先确保舵机处于速度模式
            if not self.set_servo_mode(servo_id, mode=1):  # 1=速度模式
                return False
            
            # 限制速度范围
            velocity = max(-4095, min(4095, velocity))
            
            # STS3215 写速度指令格式
            packet = [
                0xFF, 0xFF,           # 帧头
                servo_id,             # 舵机ID
                0x05,                 # 数据长度
                0x03,                 # 写指令
                0x2E                  # 目标速度寄存器地址
            ]
            
            # 添加速度参数 (小端序，处理负数)
            if velocity < 0:
                # 负速度：设置反向标志
                abs_velocity = abs(velocity)
                velocity_value = abs_velocity | 0x8000  # 设置最高位为反向标志
            else:
                velocity_value = velocity
            
            packet.extend(struct.pack('<H', velocity_value))
            
            # 计算校验和
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # 发送指令
            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))
            
            print(f"舵机 {servo_id} 速度设置为 {velocity}")
            return True
            
        except Exception as e:
            print(f"设置舵机速度失败: {e}")
            return False
    
    def stop_servo(self, servo_id):
        """
        停止舵机运动
        :param servo_id: 舵机ID
        :return: 成功返回True
        """
        return self.write_servo_velocity(servo_id, 0)
    
    def read_servo_velocity(self, servo_id):
        """
        读取STS3215舵机当前速度
        :param servo_id: 舵机ID
        :return: 当前速度，失败返回None
        """
        if not self.ser or not self.ser.is_open:
            print("请先连接舵机")
            return None
        
        try:
            # STS3215 读速度指令格式
            packet = [
                0xFF, 0xFF,           # 帧头
                servo_id,             # 舵机ID
                0x04,                 # 数据长度
                0x02,                 # 读指令
                0x3A,                 # 当前速度寄存器地址
                0x02                  # 读取字节数
            ]
            
            # 计算校验和
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # 发送指令
            self.ser.reset_input_buffer()
            self.ser.write(bytes(packet))
            
            # 等待响应
            time.sleep(0.02)
            
            if self.ser.in_waiting >= 8:
                response = self.ser.read(self.ser.in_waiting)
                
                if len(response) >= 8:
                    if response[0] == 0xFF and response[1] == 0xFF and response[2] == servo_id:
                        # 提取速度值 (小端序)
                        speed_raw = struct.unpack('<H', response[5:7])[0]
                        
                        # 处理正负速度
                        if speed_raw & 0x8000:  # 检查反向标志
                            velocity = -(speed_raw & 0x7FFF)
                        else:
                            velocity = speed_raw
                        
                        print(f"舵机 {servo_id} 当前速度: {velocity}")
                        return velocity
            
            return None
            
        except Exception as e:
            print(f"读取舵机速度失败: {e}")
            return None
    
class ServoMenu:
    """舵机控制菜单类"""
    
    def __init__(self):
        self.controller = ServoController()
        self.connected = False
    
    def show_menu(self):
        """显示主菜单"""
        print("\n" + "="*50)
        print("    Feetech STS3215 舵机控制系统")
        print("="*50)
        print("1. 连接舵机")
        print("2. 断开连接")
        print("3. PING 舵机")
        print("4. 控制舵机角度")
        print("5. 读取舵机角度")
        print("6. 设置舵机速度")
        print("7. 批量控制舵机")
        print("8. 舵机角度扫描")
        print("9. 显示连接状态")
        print("0. 退出程序")
        print("="*50)
    
    def connect_servo(self):
        """连接舵机"""
        port = input("请输入串口号 (Linux: /dev/ttyACM0, Windows: COM3): ").strip()
        if not port:
            port = "/dev/ttyACM0"  # 默认Linux端口
        
        baudrate = input("请输入波特率 (STS3215默认: 1000000): ").strip()
        if not baudrate:
            baudrate = 1000000
        else:
            try:
                baudrate = int(baudrate)
            except:
                print("波特率格式错误，使用默认值 1000000")
                baudrate = 1000000
        
        self.controller.port = port
        self.controller.baudrate = baudrate
        self.connected = self.controller.connect()
    
    def ping_servo(self):
        """PING舵机"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID (0-253): "))
            if servo_id < 0 or servo_id > 253:
                print("舵机ID超出范围")
                return
            
            self.controller.ping_servo(servo_id)
            
        except ValueError:
            print("输入格式错误")
    
    def control_servo_angle(self):
        """控制舵机角度"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID (0-253): "))
            if servo_id < 0 or servo_id > 253:
                print("舵机ID超出范围")
                return
            
            angle = float(input("请输入目标角度 (0-360): "))
            if angle < 0 or angle > 360:
                print("角度超出范围")
                return
            
            speed = input("请输入转动速度 (0-4095, 默认1000): ").strip()
            if speed:
                speed = int(speed)
            else:
                speed = 1000
            
            success = self.controller.write_servo_position(servo_id, angle, speed)
            if success:
                time.sleep(1)  # 等待舵机转动
                current_angle = self.controller.read_servo_position(servo_id)
                if current_angle is not None:
                    print(f"验证当前角度: {current_angle:.1f}°")
            
        except ValueError:
            print("输入格式错误")
    
    def read_servo_angle(self):
        """读取舵机角度"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID (0-253): "))
            if servo_id < 0 or servo_id > 253:
                print("舵机ID超出范围")
                return
            
            angle = self.controller.read_servo_position(servo_id)
            if angle is not None:
                print(f"舵机 {servo_id} 当前角度: {angle:.1f}°")
            else:
                print("读取失败")
            
        except ValueError:
            print("输入格式错误")
    
    def set_servo_speed(self):
        """设置舵机速度"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID (0-253): "))
            speed = int(input("请输入速度 (0-4095): "))
            
            if servo_id < 0 or servo_id > 253:
                print("舵机ID超出范围")
                return
            
            if speed < 0 or speed > 4095:
                print("速度超出范围")
                return
            
            self.controller.set_servo_speed(servo_id, speed)
            
        except ValueError:
            print("输入格式错误")
    
    def batch_control(self):
        """批量控制舵机"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_ids = input("请输入舵机ID列表 (用逗号分隔): ").strip()
            ids = [int(x.strip()) for x in servo_ids.split(',')]
            
            angles = input("请输入对应角度列表 (用逗号分隔): ").strip()
            angle_list = [float(x.strip()) for x in angles.split(',')]
            
            if len(ids) != len(angle_list):
                print("舵机ID数量与角度数量不匹配")
                return
            
            speed = input("请输入转动速度 (0-4095, 默认1000): ").strip()
            if speed:
                speed = int(speed)
            else:
                speed = 1000
            
            print("开始批量控制...")
            for servo_id, angle in zip(ids, angle_list):
                if 0 <= servo_id <= 253 and 0 <= angle <= 360:
                    self.controller.write_servo_position(servo_id, angle, speed)
                    time.sleep(0.1)
                else:
                    print(f"跳过无效参数: ID={servo_id}, 角度={angle}")
            
            print("批量控制完成")
            
        except ValueError:
            print("输入格式错误")
    
    def angle_sweep(self):
        """舵机角度扫描"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID (0-253): "))
            start_angle = float(input("请输入起始角度 (0-360): "))
            end_angle = float(input("请输入结束角度 (0-360): "))
            step = float(input("请输入步进角度 (默认10): ") or "10")
            delay = float(input("请输入步进延时/秒 (默认1): ") or "1")
            speed = int(input("请输入转动速度 (0-4095, 默认1000): ") or "1000")
            
            if not (0 <= servo_id <= 253 and 0 <= start_angle <= 360 and 0 <= end_angle <= 360):
                print("参数超出范围")
                return
            
            print(f"开始角度扫描: {start_angle}° -> {end_angle}°")
            
            current = start_angle
            direction = 1 if end_angle > start_angle else -1
            
            while (direction > 0 and current <= end_angle) or (direction < 0 and current >= end_angle):
                self.controller.write_servo_position(servo_id, current, speed)
                print(f"目标角度: {current:.1f}°")
                time.sleep(delay)
                current += step * direction
            
            print("角度扫描完成")
            
        except ValueError:
            print("输入格式错误")
    
    def show_status(self):
        """显示连接状态"""
        print(f"连接状态: {'已连接' if self.connected else '未连接'}")
        if self.connected:
            print(f"端口: {self.controller.port}")
            print(f"波特率: {self.controller.baudrate}")
            print("舵机型号: Feetech STS3215")
    
    def run(self):
        """运行主程序"""
        print("欢迎使用 Feetech STS3215 舵机控制系统!")
        
        while True:
            self.show_menu()
            choice = input("请选择操作 (0-9): ").strip()
            
            if choice == '1':
                self.connect_servo()
            elif choice == '2':
                self.controller.disconnect()
                self.connected = False
            elif choice == '3':
                self.ping_servo()
            elif choice == '4':
                self.control_servo_angle()
            elif choice == '5':
                self.read_servo_angle()
            elif choice == '6':
                self.set_servo_speed()
            elif choice == '7':
                self.batch_control()
            elif choice == '8':
                self.angle_sweep()
            elif choice == '9':
                self.show_status()
            elif choice == '0':
                self.controller.disconnect()
                print("程序退出，谢谢使用!")
                break
            else:
                print("无效选择，请重新输入")
            
            input("\n按回车键继续...")

class AdvancedServoMenu:
    """高级舵机控制菜单 - 包含官方库风格边界处理"""
    
    def __init__(self):
        self.controller = ServoController()
        self.connected = False
    
    def show_advanced_menu(self):
        """显示高级功能菜单"""
        print("\n" + "="*60)
        print("    高级舵机控制系统 (官方库风格边界处理)")
        print("="*60)
        print("1. 连接舵机")
        print("2. 校准舵机位置")
        print("3. 设置零点位置")
        print("4. 智能角度控制 (最短路径)")
        print("5. 强制方向控制 (顺时针/逆时针)")
        print("6. 相对位置移动")
        print("7. 多圈位置查询")
        print("8. 舵机状态监控")
        print("9. 大角度移动测试")
        print("10. 边界跨越测试")
        print("0. 退出")
        print("="*60)
    
    def connect_servo(self):
        """连接舵机"""
        port = input("请输入串口号 (Linux: /dev/ttyACM0, Windows: COM3): ").strip()
        if not port:
            port = "/dev/ttyACM0"  # 默认Linux端口
        
        baudrate = input("请输入波特率 (STS3215默认: 1000000): ").strip()
        if not baudrate:
            baudrate = 1000000
        else:
            try:
                baudrate = int(baudrate)
            except:
                print("波特率格式错误，使用默认值 1000000")
                baudrate = 1000000
        
        self.controller.port = port
        self.controller.baudrate = baudrate
        self.connected = self.controller.connect()
    
    def calibrate_position(self):
        """校准位置功能"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID: "))
            self.controller.calibrate_servo_position(servo_id)
        except ValueError:
            print("输入格式错误")
    
    def set_home_position(self):
        """设置零点位置"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID: "))
            home_angle = float(input("请输入零点角度 (默认180): ") or "180")
            self.controller.set_servo_home_position(servo_id, home_angle)
        except ValueError:
            print("输入格式错误")
    
    def smart_angle_control(self):
        """智能角度控制"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID: "))
            target_angle = float(input("请输入目标角度 (0-360): "))
            speed = int(input("请输入速度 (0-4095, 默认1000): ") or "1000")
            
            print("执行智能控制 (最短路径)...")
            success = self.controller.write_servo_position_official_style(
                servo_id, target_angle, speed, 'shortest'
            )
            
            if success:
                print("✓ 控制成功")
                # 显示最终状态
                status = self.controller.get_servo_status(servo_id)
                print(f"最终位置: {status['single_turn_position']:.1f}° (多圈: {status['multi_turn_position']:.1f}°)")
            else:
                print("✗ 控制失败")
                
        except ValueError:
            print("输入格式错误")
    
    def force_direction_control(self):
        """强制方向控制"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID: "))
            target_angle = float(input("请输入目标角度 (0-360): "))
            speed = int(input("请输入速度 (0-4095, 默认1000): ") or "1000")
            
            print("选择强制方向:")
            print("1. 强制顺时针")
            print("2. 强制逆时针")
            direction_choice = input("请选择 (1/2): ").strip()
            
            if direction_choice == '1':
                direction = 'cw'
                print("执行强制顺时针控制...")
            elif direction_choice == '2':
                direction = 'ccw'
                print("执行强制逆时针控制...")
            else:
                print("无效选择")
                return
            
            success = self.controller.write_servo_position_official_style(
                servo_id, target_angle, speed, direction
            )
            
            if success:
                print("✓ 强制方向控制成功")
            else:
                print("✗ 控制失败")
                
        except ValueError:
            print("输入格式错误")
    
    def relative_movement(self):
        """相对位置移动"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID: "))
            relative_angle = float(input("请输入相对角度 (正数顺时针，负数逆时针): "))
            speed = int(input("请输入速度 (0-4095, 默认1000): ") or "1000")
            
            success = self.controller.move_servo_relative(servo_id, relative_angle, speed)
            
            if success:
                print("✓ 相对移动成功")
            else:
                print("✗ 移动失败")
                
        except ValueError:
            print("输入格式错误")
    
    def query_multi_turn_position(self):
        """查询多圈位置"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID: "))
            
            status = self.controller.get_servo_status(servo_id)
            
            print(f"\n舵机 {servo_id} 位置信息:")
            print(f"  单圈位置: {status['single_turn_position']:.1f}°")
            print(f"  多圈位置: {status['multi_turn_position']:.1f}°")
            print(f"  连接状态: {'已连接' if status['connected'] else '未连接'}")
            
        except ValueError:
            print("输入格式错误")
    
    def monitor_servo_status(self):
        """舵机状态监控"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID: "))
            duration = int(input("请输入监控时长/秒 (默认10): ") or "10")
            
            print(f"开始监控舵机 {servo_id}，持续 {duration} 秒...")
            print("时间\t单圈位置\t多圈位置")
            print("-" * 40)
            
            start_time = time.time()
            while time.time() - start_time < duration:
                status = self.controller.get_servo_status(servo_id)
                elapsed = time.time() - start_time
                
                print(f"{elapsed:.1f}s\t{status['single_turn_position']:.1f}°\t\t{status['multi_turn_position']:.1f}°")
                time.sleep(0.5)
                
        except ValueError:
            print("输入格式错误")
        except KeyboardInterrupt:
            print("\n监控已停止")
    
    def test_large_angle_movement(self):
        """大角度移动测试"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID: "))
            
            # 先校准位置
            self.controller.calibrate_servo_position(servo_id)
            
            print("大角度移动测试序列:")
            test_angles = [0, 360, 720, 1080, 540, 180]
            
            for i, angle in enumerate(test_angles):
                print(f"\n步骤 {i+1}: 移动到 {angle}°")
                success = self.controller.write_servo_position_official_style(
                    servo_id, angle % 360, 1000, 'shortest'
                )
                
                if success:
                    status = self.controller.get_servo_status(servo_id)
                    print(f"  到达位置: {status['multi_turn_position']:.1f}°")
                else:
                    print("  移动失败")
                    break
                
                input("  按回车继续下一步...")
                
        except ValueError:
            print("输入格式错误")
    
    def test_boundary_crossing(self):
        """边界跨越测试"""
        if not self.connected:
            print("请先连接舵机")
            return
        
        try:
            servo_id = int(input("请输入舵机ID: "))
            
            # 校准位置
            self.controller.calibrate_servo_position(servo_id)
            
            print("边界跨越测试序列:")
            test_cases = [
                (350, 10, "从350°到10°"),
                (10, 350, "从10°到350°"),
                (0, 359, "从0°到359°"),
                (359, 1, "从359°到1°"),
                (270, 90, "从270°到90°"),
                (90, 270, "从90°到270°")
            ]
            
            for start, end, description in test_cases:
                print(f"\n{description}")
                
                # 先移动到起始位置
                self.controller.write_servo_position_official_style(servo_id, start, 1000)
                time.sleep(1)
                
                # 测试最短路径
                print("  执行最短路径移动...")
                self.controller.write_servo_position_official_style(servo_id, end, 500, 'shortest')
                
                status = self.controller.get_servo_status(servo_id)
                print(f"  最终位置: {status['single_turn_position']:.1f}° (多圈: {status['multi_turn_position']:.1f}°)")
                
                input("  观察移动方向，按回车继续...")
                
        except ValueError:
            print("输入格式错误")
    
    def run(self):
        """运行高级控制程序"""
        print("欢迎使用高级舵机控制系统!")
        print("本系统采用官方库风格的边界处理算法")
        
        while True:
            self.show_advanced_menu()
            choice = input("请选择操作 (0-10): ").strip()
            
            if choice == '1':
                self.connect_servo()
            elif choice == '2':
                self.calibrate_position()
            elif choice == '3':
                self.set_home_position()
            elif choice == '4':
                self.smart_angle_control()
            elif choice == '5':
                self.force_direction_control()
            elif choice == '6':
                self.relative_movement()
            elif choice == '7':
                self.query_multi_turn_position()
            elif choice == '8':
                self.monitor_servo_status()
            elif choice == '9':
                self.test_large_angle_movement()
            elif choice == '10':
                self.test_boundary_crossing()
            elif choice == '0':
                self.controller.disconnect()
                print("程序退出!")
                break
            else:
                print("无效选择")
            
            input("\n按回车键继续...")

def test_sts3215():
    """专门测试STS3215舵机"""
    print("测试 Feetech STS3215 舵机...")
    
    # 创建控制器 (使用正确的参数)
    port = input("请输入串口号 (默认 /dev/ttyACM0): ").strip() or "/dev/ttyACM0"
    controller = ServoController(port, 1000000)
    
    # 测试连接
    if controller.connect():
        print("连接测试通过")
        
        # 测试舵机ID
        test_id = int(input("请输入要测试的舵机ID (默认 1): ") or "1")
        
        # PING测试
        if controller.ping_servo(test_id):
            print(f"舵机 {test_id} 响应正常")
            
            # 测试角度转换
            print(f"180° 对应位置值: {controller.angle_to_position(180)}")
            print(f"位置值 2048 对应角度: {controller.position_to_angle(2048):.1f}°")
            
            # 测试舵机控制
            test_angles = [0, 90, 180, 270, 360]
            
            for angle in test_angles:
                print(f"\n测试角度: {angle}°")
                success = controller.write_servo_position(test_id, angle, 1000)
                if success:
                    time.sleep(2)  # 等待转动完成
                    
                    current = controller.read_servo_position(test_id)
                    if current is not None:
                        print(f"实际角度: {current:.1f}°")
                        error = abs(angle - current)
                        print(f"误差: {error:.1f}°")
                    else:
                        print("读取角度失败")
                        
                input("按回车继续下一个角度...")
        else:
            print(f"舵机 {test_id} 无响应，请检查ID和连接")
        
        controller.disconnect()
    else:
        print("连接测试失败，请检查串口和波特率")

def test_direction_control():
    """测试方向控制功能"""
    print("测试舵机方向控制...")
    
    port = input("请输入串口号 (默认 /dev/ttyACM0): ").strip() or "/dev/ttyACM0"
    controller = ServoController(port, 1000000)
    
    if controller.connect():
        servo_id = int(input("请输入舵机ID (默认 1): ") or "1")
        
        if controller.ping_servo(servo_id):
            print("测试方向控制功能...")
            
            # 创建方向控制器
            dir_controller = ServoDirectionController(controller)
            
            # 测试跨边界移动
            test_sequence = [
                (270, "移动到270°"),
                (350, "移动到350°"),
                (10, "跨边界移动到10°"),
                (340, "跨边界移动到340°"),
                (20, "再次跨边界到20°"),
                (180, "回到中心位置")
            ]
            
            for angle, description in test_sequence:
                print(f"\n{description}")
                dir_controller.move_servo_continuous(servo_id, angle, 500)
                input("观察转动方向，按回车继续...")
        
        controller.disconnect()
    else:
        print("连接失败")

def test_boundary_control():
    """测试边界控制功能"""
    print("测试舵机边界控制...")
    
    port = input("请输入串口号 (默认 /dev/ttyACM0): ").strip() or "/dev/ttyACM0"
    controller = ServoController(port, 1000000)
    
    if controller.connect():
        servo_id = int(input("请输入舵机ID (默认 1): ") or "1")
        
        if controller.ping_servo(servo_id):
            print("测试各种边界控制模式...")
            
            # 测试序列
            test_cases = [
                (180, 180, 'normal', "重置到中心"),
                (270, 350, 'smart', "智能移动: 270° -> 350°"),
                (350, 10, 'smart', "智能跨边界: 350° -> 10°"),
                (10, 340, 'continuous', "连续移动: 10° -> 340°"),
                (340, 20, 'force_cw', "强制顺时针: 340° -> 20°"),
                (20, 340, 'force_ccw', "强制逆时针: 20° -> 340°"),
                (180, 180, 'normal', "回到中心")
            ]
            
            current_angle = 180
            for start, target, mode, description in test_cases:
                print(f"\n{description}")
                
                # 先移动到起始位置
                if start != current_angle:
                    controller.write_servo_position_enhanced(servo_id, start, current_angle, 1000, 'normal')
                    time.sleep(1)
                    current_angle = start
                
                # 执行测试移动
                print(f"执行: {start}° -> {target}° ({mode})")
                success = controller.write_servo_position_enhanced(servo_id, target, current_angle, 500, mode)
                
                if success:
                    print("✓ 移动成功")
                    current_angle = target
                else:
                    print("✗ 移动失败")
                
                input("观察舵机动作，按回车继续...")
        
        controller.disconnect()
    else:
        print("连接失败")

def test_official_style_boundary():
    """测试官方库风格的边界处理"""
    print("测试官方库风格边界处理算法...")
    
    controller = ServoController("/dev/ttyACM0", 1000000)
    
    if controller.connect():
        servo_id = 1
        
        if controller.ping_servo(servo_id):
            print("测试多圈位置跟踪和边界处理...")
            
            # 校准起始位置
            controller.calibrate_servo_position(servo_id)
            
            # 测试序列
            moves = [
                (350, '最短路径'),
                (10, '跨边界 350°->10°'),
                (350, '跨边界 10°->350°'),
                (180, '回到中心'),
            ]
            
            for target, description in moves:
                print(f"\n{description}: 移动到 {target}°")
                
                # 显示移动前状态
                before = controller.get_multi_turn_position(servo_id)
                print(f"  移动前多圈位置: {before:.1f}°")
                
                # 执行移动
                success = controller.write_servo_position_official_style(servo_id, target, 1000)
                
                if success:
                    time.sleep(2)
                    after = controller.get_multi_turn_position(servo_id)
                    single = controller.read_servo_position(servo_id)
                    print(f"  移动后单圈位置: {single:.1f}°")
                    print(f"  移动后多圈位置: {after:.1f}°")
                    print(f"  实际移动角度: {after - before:.1f}°")
                else:
                    print("  移动失败")
                
                input("  按回车继续...")
        
        controller.disconnect()
    else:
        print("连接失败")

if __name__ == "__main__":
    # 选择运行模式
    print("请选择运行模式:")
    print("1. 传统交互式菜单")
    print("2. 高级控制菜单 (官方库风格)")
    print("3. 官方风格边界处理测试")
    print("4. STS3215专项测试")
    
    choice = input("请输入选择 (1/2/3/4): ").strip()
    
    if choice == '1':
        menu = ServoMenu()
        menu.run()
    elif choice == '2':
        advanced_menu = AdvancedServoMenu()
        advanced_menu.run()
    elif choice == '3':
        test_official_style_boundary()
    elif choice == '4':
        test_sts3215()
    else:
        print("无效选择")