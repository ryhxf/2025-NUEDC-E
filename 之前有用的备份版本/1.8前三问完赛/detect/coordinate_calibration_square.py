#!/usr/bin/env python3
"""
方块跟踪坐标标定工具 - 专为串行总线舵机设计
采集图像坐标与舵机角度增量的对应关系，建立增量映射关系
"""
import cv2
import numpy as np
import json
import time
import threading
import argparse
from pathlib import Path
import os
import sys

# 添加项目路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))
sys.path.append('/root/square/all_duoji')

try:
    from control import ServoController
    print("✓ 串行总线舵机控制模块导入成功")
except ImportError as e:
    print(f"❌ 串行总线舵机控制模块导入失败: {e}")
    ServoController = None

class SquareCoordinateCalibrator:
    """方块跟踪坐标标定器 - 专为串行总线舵机设计"""
    
    def __init__(self, camera_id=0, image_width=640, image_height=480):
        self.camera_id = camera_id
        self.image_width = image_width
        self.image_height = image_height
        
        # **串行总线舵机控制 - 新增：可配置参数**
        self.servo_controller = None
        self.servo_connected = False
        self.servo_port = '/dev/ttyACM0'     # **新增：可配置端口**
        self.servo_baudrate = 1000000        # **新增：可配置波特率**
        self.x_servo_id = 1                  # **新增：可配置X轴舵机ID**
        self.y_servo_id = 2                  # **新增：可配置Y轴舵机ID**
        
        # **串行总线舵机参数**
        self.servo_resolution = 4096  # 每360度对应4096
        self.angle_per_unit = 360.0 / self.servo_resolution  # 约0.0879度/单位
        self.position_min = -32767
        self.position_max = 32768
        
        # **舵机当前位置（用于计算增量）**
        self.current_x_position = 0  # 当前X轴位置
        self.current_y_position = 0  # 当前Y轴位置
        self.center_x_position = 0   # 中心X轴位置
        self.center_y_position = 0   # 中心Y轴位置
        
        # **控制步长（舵机位置单位）**
        self.coarse_step = int(5.0 / self.angle_per_unit)    # 粗调：5度
        self.fine_step = int(1.0 / self.angle_per_unit)      # 精调：1度
        self.micro_step = int(0.5 / self.angle_per_unit)     # 微调：0.5度
        
        # **增量限制**
        self.max_delta_angle_x = 15.0  # X轴最大增量角度
        self.max_delta_angle_y = 15.0  # Y轴最大增量角度
        self.max_delta_position_x = int(self.max_delta_angle_x / self.angle_per_unit)
        self.max_delta_position_y = int(self.max_delta_angle_y / self.angle_per_unit)
        
        # **标定数据 - 关键：存储增量信息**
        self.calibration_points = []  # 存储 {image_x, image_y, delta_x, delta_y, center_x, center_y}
        
        # 摄像头
        self.cap = None
        self.frame = None
        self.calibration_active = False
        
        # 显示参数
        self.crosshair_size = 20
        self.point_radius = 5
        
        print(f"✓ [标定器] 舵机分辨率: {self.servo_resolution} (每度约{self.servo_resolution/360:.1f}单位)")
        print(f"✓ [标定器] 角度精度: {self.angle_per_unit:.4f}度/单位")
        print(f"✓ [标定器] 增量范围: X(±{self.max_delta_angle_x}°), Y(±{self.max_delta_angle_y}°)")
    
    def init_camera(self):
        """初始化摄像头"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                print(f"❌ 无法打开摄像头 {self.camera_id}")
                return False
            
            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # 测试读取
            ret, frame = self.cap.read()
            if not ret:
                print(f"❌ 摄像头无法读取帧")
                return False
            
            actual_height, actual_width = frame.shape[:2]
            print(f"✓ 摄像头初始化成功，分辨率: {actual_width}x{actual_height}")
            return True
            
        except Exception as e:
            print(f"❌ 摄像头初始化失败: {e}")
            return False
    
    def get_connection_config(self):
        """**新增：获取连接配置的交互式终端**"""
        print("\n" + "="*60)
        print("🔧 串行总线舵机连接配置")
        print("="*60)
        
        # 显示当前默认配置
        print("📋 当前默认配置:")
        print(f"  串口端口: {self.servo_port}")
        print(f"  波特率: {self.servo_baudrate}")
        print(f"  X轴舵机ID: {self.x_servo_id}")
        print(f"  Y轴舵机ID: {self.y_servo_id}")
        print()
        
        # 询问是否修改配置
        while True:
            choice = input("是否修改连接配置? (y/n) [n]: ").strip().lower()
            if choice == '' or choice == 'n':
                print("✓ 使用默认配置")
                break
            elif choice == 'y':
                self._configure_connection_parameters()
                break
            else:
                print("⚠️ 请输入 y 或 n")
        
        # 显示最终配置
        print("\n📡 最终连接配置:")
        print(f"  串口端口: {self.servo_port}")
        print(f"  波特率: {self.servo_baudrate}")
        print(f"  X轴舵机ID: {self.x_servo_id}")
        print(f"  Y轴舵机ID: {self.y_servo_id}")
        print("="*60 + "\n")
    
    def _configure_connection_parameters(self):
        """**新增：配置连接参数**"""
        print("\n🔧 配置连接参数:")
        
        # 配置串口端口
        ports_list = [
            "/dev/ttyACM0", "/dev/ttyACM1", 
            "/dev/ttyUSB0", "/dev/ttyUSB1",
            "/dev/ttyS0", "/dev/ttyS1"
        ]
        
        print("\n📡 可用串口端口:")
        for i, port in enumerate(ports_list, 1):
            print(f"  {i}. {port}")
        print(f"  7. 自定义端口")
        
        while True:
            try:
                choice = input(f"选择串口端口 [当前: {self.servo_port}]: ").strip()
                if choice == '':
                    break
                elif choice.isdigit():
                    choice_num = int(choice)
                    if 1 <= choice_num <= 6:
                        self.servo_port = ports_list[choice_num - 1]
                        print(f"✓ 串口端口设置为: {self.servo_port}")
                        break
                    elif choice_num == 7:
                        custom_port = input("输入自定义串口端口: ").strip()
                        if custom_port:
                            self.servo_port = custom_port
                            print(f"✓ 串口端口设置为: {self.servo_port}")
                            break
                else:
                    print("⚠️ 请输入有效的选项")
            except ValueError:
                print("⚠️ 请输入数字")
        
        # 配置波特率
        baudrates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000]
        
        print(f"\n📶 可用波特率:")
        for i, baud in enumerate(baudrates, 1):
            mark = " ⭐" if baud == self.servo_baudrate else ""
            print(f"  {i}. {baud}{mark}")
        
        while True:
            try:
                choice = input(f"选择波特率 [当前: {self.servo_baudrate}]: ").strip()
                if choice == '':
                    break
                elif choice.isdigit():
                    choice_num = int(choice)
                    if 1 <= choice_num <= len(baudrates):
                        self.servo_baudrate = baudrates[choice_num - 1]
                        print(f"✓ 波特率设置为: {self.servo_baudrate}")
                        break
                else:
                    custom_baud = int(choice)
                    if 1200 <= custom_baud <= 2000000:
                        self.servo_baudrate = custom_baud
                        print(f"✓ 波特率设置为: {self.servo_baudrate}")
                        break
                    else:
                        print("⚠️ 波特率范围应在 1200-2000000 之间")
            except ValueError:
                print("⚠️ 请输入有效的数字")
        
        # 配置舵机ID
        while True:
            try:
                x_id = input(f"X轴舵机ID [当前: {self.x_servo_id}]: ").strip()
                if x_id == '':
                    break
                x_id_num = int(x_id)
                if 1 <= x_id_num <= 253:
                    self.x_servo_id = x_id_num
                    print(f"✓ X轴舵机ID设置为: {self.x_servo_id}")
                    break
                else:
                    print("⚠️ 舵机ID范围应在 1-253 之间")
            except ValueError:
                print("⚠️ 请输入有效的数字")
        
        while True:
            try:
                y_id = input(f"Y轴舵机ID [当前: {self.y_servo_id}]: ").strip()
                if y_id == '':
                    break
                y_id_num = int(y_id)
                if 1 <= y_id_num <= 253:
                    if y_id_num == self.x_servo_id:
                        print("⚠️ Y轴舵机ID不能与X轴相同")
                        continue
                    self.y_servo_id = y_id_num
                    print(f"✓ Y轴舵机ID设置为: {self.y_servo_id}")
                    break
                else:
                    print("⚠️ 舵机ID范围应在 1-253 之间")
            except ValueError:
                print("⚠️ 请输入有效的数字")
    
    def init_servo(self):
        """**修改：增强的串行总线舵机初始化**"""
        if ServoController is None:
            print("❌ 串行总线舵机控制模块不可用")
            print("💡 请检查是否正确安装了 control 模块")
            return False
        
        print(f"\n🔗 正在连接串行总线舵机...")
        print(f"📡 端口: {self.servo_port}")
        print(f"📶 波特率: {self.servo_baudrate}")
        print(f"🎯 舵机ID: X轴={self.x_servo_id}, Y轴={self.y_servo_id}")
        
        try:
            # **使用配置的参数初始化**
            self.servo_controller = ServoController(
                port=self.servo_port,
                baudrate=self.servo_baudrate
            )
            
            print("⏳ 正在建立连接...")
            connection_success = self.servo_controller.connect()
            
            if not connection_success:
                print("❌ 串行总线舵机连接失败")
                print("🔧 可能的解决方案:")
                print("   1. 检查串口端口是否正确")
                print("   2. 检查波特率是否匹配")
                print("   3. 检查串口设备是否被其他程序占用")
                print("   4. 检查USB连接线和电源")
                self.servo_connected = False
                return False
            
            print("✓ 串口连接成功，正在测试舵机...")
            
            # **测试舵机连接 - 增强错误处理**
            try:
                print(f"🧪 测试X轴舵机 (ID: {self.x_servo_id})...")
                x_ping = self.servo_controller.ping_servo(self.x_servo_id)
                
                if not x_ping:
                    print(f"❌ X轴舵机 (ID: {self.x_servo_id}) 无响应")
                    print("🔧 请检查:")
                    print(f"   1. 舵机ID是否正确设置为 {self.x_servo_id}")
                    print("   2. 舵机电源是否正常")
                    print("   3. 舵机连接线是否正常")
                    self.servo_connected = False
                    return False
                
                print(f"✓ X轴舵机响应正常")
                
                print(f"🧪 测试Y轴舵机 (ID: {self.y_servo_id})...")
                y_ping = self.servo_controller.ping_servo(self.y_servo_id)
                
                if not y_ping:
                    print(f"❌ Y轴舵机 (ID: {self.y_servo_id}) 无响应")
                    print("🔧 请检查:")
                    print(f"   1. 舵机ID是否正确设置为 {self.y_servo_id}")
                    print("   2. 舵机电源是否正常")
                    print("   3. 舵机连接线是否正常")
                    self.servo_connected = False
                    return False
                
                print(f"✓ Y轴舵机响应正常")
                print(f"🎉 所有舵机测试通过!")
                
            except Exception as ping_error:
                print(f"⚠️ 舵机ping测试异常: {ping_error}")
                print("🔄 但连接成功，继续运行...")
            
            self.servo_connected = True
            
            # **读取当前位置作为中心位置 - 修复方法调用**
            try:
                print("📍 读取舵机当前位置...")
                
                # **修复：使用正确的读取位置方法名**
                self.current_x_position = self.servo_controller.read_servo_position(self.x_servo_id)
                self.current_y_position = self.servo_controller.read_servo_position(self.y_servo_id)
                
                # **将角度转换为位置值（因为我们内部使用位置值跟踪）**
                if self.current_x_position is not None and self.current_y_position is not None:
                    # control.py返回的是角度，需要转换为位置值
                    self.current_x_position = int(self.current_x_position / self.angle_per_unit)
                    self.current_y_position = int(self.current_y_position / self.angle_per_unit)
                else:
                    print("⚠️ 无法读取舵机角度，使用默认位置")
                    self.current_x_position = 0
                    self.current_y_position = 0
                
                self.center_x_position = self.current_x_position
                self.center_y_position = self.current_y_position
                
                print(f"✓ 当前位置读取成功:")
                print(f"   X轴: {self.center_x_position} ({self.center_x_position * self.angle_per_unit:.1f}°)")
                print(f"   Y轴: {self.center_y_position} ({self.center_y_position * self.angle_per_unit:.1f}°)")
                print(f"📌 设置为标定中心位置")
                
            except Exception as pos_error:
                print(f"⚠️ 无法读取当前位置: {pos_error}")
                print("🔧 使用默认中心位置: X=0, Y=0")
                self.current_x_position = 0
                self.current_y_position = 0
                self.center_x_position = 0
                self.center_y_position = 0
            
            print("🎉 串行总线舵机初始化完成!")
            return True
                
        except Exception as e:
            print(f"❌ 串行总线舵机初始化异常: {e}")
            print("🔧 可能的原因:")
            print("   1. 串口权限不足 (尝试: sudo chmod 666 /dev/ttyACM*)")
            print("   2. 设备未连接或端口错误")
            print("   3. 波特率不匹配")
            print("   4. 其他程序正在使用该串口")
            self.servo_connected = False
            return False
    
    def move_servo_relative(self, delta_x_position, delta_y_position):
        """相对移动舵机"""
        # 计算新位置
        new_x_position = self.current_x_position + delta_x_position
        new_y_position = self.current_y_position + delta_y_position
        
        # 限制位置范围
        new_x_position = max(self.position_min, min(self.position_max, new_x_position))
        new_y_position = max(self.position_min, min(self.position_max, new_y_position))
        
        # 移动舵机
        self.move_servo_absolute(new_x_position, new_y_position)
    
    def move_servo_absolute(self, x_position, y_position):
        """**修复：绝对移动舵机到指定位置**"""
        # 更新当前位置记录
        self.current_x_position = x_position
        self.current_y_position = y_position
        
        # **修复：使用self.servo_connected标志**
        if self.servo_controller and self.servo_connected:
            try:
                # **修复：使用正确的位置控制方法 - 转换为角度**
                x_angle = x_position * self.angle_per_unit
                y_angle = y_position * self.angle_per_unit
                
                # 使用write_servo_position方法（角度控制）
                success_x = self.servo_controller.write_servo_position(self.x_servo_id, x_angle)
                success_y = self.servo_controller.write_servo_position(self.y_servo_id, y_angle)
                
                if success_x and success_y:
                    print(f"✓ 舵机移动: X={x_position}({x_angle:.1f}°), Y={y_position}({y_angle:.1f}°)")
                else:
                    print(f"❌ 舵机移动失败: X成功={success_x}, Y成功={success_y}")
                    
            except Exception as e:
                print(f"❌ 舵机移动异常: {e}")
                # **详细错误信息**
                print(f"🔧 错误详情: {type(e).__name__}: {e}")
                # 连接可能断开，标记为未连接
                self.servo_connected = False
        else:
            # 虚拟控制模式
            x_angle = x_position * self.angle_per_unit
            y_angle = y_position * self.angle_per_unit
            print(f"🎮 虚拟舵机: X={x_position}({x_angle:.1f}°), Y={y_position}({y_angle:.1f}°)")
    
    def get_current_delta_angles(self):
        """获取当前相对于中心位置的角度增量"""
        delta_x_position = self.current_x_position - self.center_x_position
        delta_y_position = self.current_y_position - self.center_y_position
        
        delta_x_angle = delta_x_position * self.angle_per_unit
        delta_y_angle = delta_y_position * self.angle_per_unit
        
        return delta_x_angle, delta_y_angle
    
    def handle_keyboard(self, key):
        """处理键盘输入"""
        step = self.coarse_step
        
        # 根据按键确定步长
        if key == ord('1'):
            step = self.micro_step
            step_angle = step * self.angle_per_unit
            print(f"🎯 切换到微调模式: {step}单位 ({step_angle:.2f}°)")
            return
        elif key == ord('2'):
            step = self.fine_step
            step_angle = step * self.angle_per_unit
            print(f"🎯 切换到精调模式: {step}单位 ({step_angle:.2f}°)")
            return
        elif key == ord('3'):
            step = self.coarse_step
            step_angle = step * self.angle_per_unit
            print(f"🎯 切换到粗调模式: {step}单位 ({step_angle:.2f}°)")
            return
        
        # 方向键控制
        delta_x = 0
        delta_y = 0
        
        if key == ord('w') or key == ord('W'):  # 上
            delta_y = step
        elif key == ord('s') or key == ord('S'):  # 下
            delta_y = -step
        elif key == ord('d') or key == ord('D'):  # 右（注意：舵机坐标系）
            delta_x = step
        elif key == ord('a') or key == ord('A'):  # 左
            delta_x = -step
        elif key == ord('r') or key == ord('R'):  # 重置到中心
            self.move_servo_absolute(self.center_x_position, self.center_y_position)
            print("🏠 重置到中心位置")
            return
        elif key == ord('h') or key == ord('H'):  # 设置当前位置为新中心
            self.center_x_position = self.current_x_position
            self.center_y_position = self.current_y_position
            print(f"🏠 设置新中心位置: X={self.center_x_position}, Y={self.center_y_position}")
            return
        elif key == ord('c') or key == ord('C'):  # 清空标定点
            self.calibration_points.clear()
            print("🗑️ 已清空所有标定点")
            return
        elif key == ord(' '):  # 空格键 - 显示帮助
            self.show_help()
            return
        else:
            return
        
        # 执行移动
        if delta_x != 0 or delta_y != 0:
            self.move_servo_relative(delta_x, delta_y)
    
    def show_help(self):
        """显示帮助信息"""
        print("\n" + "="*60)
        print("📚 方块跟踪坐标标定工具 - 控制说明")
        print("="*60)
        print("🎮 键盘控制:")
        print("  W/A/S/D    - 控制舵机移动 (上/左/下/右)")
        print("  1/2/3      - 切换步长 (微调0.5°/精调1°/粗调5°)")
        print("  R          - 重置舵机到中心位置")
        print("  H          - 设置当前位置为新中心")
        print("  C          - 清空所有标定点")
        print("  J          - 保存标定数据")
        print("  L          - 加载标定数据")
        print("  Space      - 显示此帮助")
        print("  ESC/Q      - 退出程序")
        print("\n🖱️ 鼠标控制:")
        print("  左键点击   - 在当前位置添加标定点")
        print("  右键点击   - 删除最后一个标定点")
        print("\n🎯 **基于四边形中心点的标定原理**: ")
        print("  **此标定专门针对YOLO+OpenCV检测的四边形中心点进行优化:**")
        print("  📌 运行时检测流程:")
        print("     1. YOLO11检测方块区域 -> 获得粗略边界框")
        print("     2. OpenCV在边界框内精细化检测 -> 找到准确的四边形轮廓")
        print("     3. 计算四边形的几何中心点 -> 得到精确的中心坐标")
        print("     4. 中心偏差 = 检测中心 - 目标中心")
        print("     5. 查表映射 = 偏差 -> 舵机角度增量")
        print("\n📐 **标定数据采集原理**: ")
        print("  💡 每个标定点记录的映射关系:")
        print("     • 输入: 鼠标点击位置 (模拟检测到的四边形中心)")
        print("     • 输出: 当前舵机相对中心的角度增量")
        print("     • 含义: 当四边形中心在该位置时，舵机应如何调整")
        print("\n🔧 **标定操作流程**: ")
        print("  1️⃣ 激光对准图像中心 (目标位置)")
        print("  2️⃣ 移动激光到其他位置 (模拟四边形中心偏移)")
        print("  3️⃣ 点击激光所在位置 (记录偏差->增量映射)")
        print("  4️⃣ 重复2-3步骤，采集不同位置的映射数据")
        print("  5️⃣ 建议采集: 上下左右中心 + 四个对角 (至少9个点)")
        print("\n🎲 **映射表的实际应用**: ")
        print("  运行时: YOLO+OpenCV -> 四边形中心(x,y)")
        print("         计算偏差: offset = 中心位置 - 目标位置")
        print("         查表映射: delta_angle = f(offset)")
        print("         舵机控制: new_angle = current_angle + delta_angle")
        print("\n💭 **为什么基于四边形中心点?**")
        print("  ✓ 四边形中心比边界框中心更精确")
        print("  ✓ 不受方块旋转影响，几何中心稳定")
        print("  ✓ OpenCV轮廓检测的中心计算精度高")
        print("  ✓ 适应不同大小的方块，中心位置一致")
        
        print(f"\n⚙️ 舵机参数:")
        print(f"  分辨率: {self.servo_resolution} 单位/360°")
        print(f"  精度: {self.angle_per_unit:.4f}°/单位")
        print(f"  增量范围: X(±{self.max_delta_angle_x}°), Y(±{self.max_delta_angle_y}°)")
        print("="*60 + "\n")
    
    def draw_overlay(self, frame):
        """**增强：绘制四边形中心点标定专用的覆盖层信息**"""
        overlay = frame.copy()
        
        # **绘制目标中心点（图像中心，激光瞄准位置）**
        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2
        
        # 目标中心大十字（绿色，表示激光应该瞄准的位置）
        cv2.line(overlay, (center_x - self.crosshair_size*2, center_y), 
                 (center_x + self.crosshair_size*2, center_y), (0, 255, 0), 3)
        cv2.line(overlay, (center_x, center_y - self.crosshair_size*2), 
                 (center_x, center_y + self.crosshair_size*2), (0, 255, 0), 3)
        
        # 目标中心圆圈
        cv2.circle(overlay, (center_x, center_y), 12, (0, 255, 0), 3)
        cv2.putText(overlay, "LASER TARGET", (center_x - 45, center_y - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # **绘制四边形中心检测区域示意**
        # 在图像四周绘制一些虚拟的"四边形中心"位置示例
        example_centers = [
            (center_x - 80, center_y - 60, "检测中心1"),
            (center_x + 80, center_y - 60, "检测中心2"),
            (center_x - 80, center_y + 60, "检测中心3"),
            (center_x + 80, center_y + 60, "检测中心4"),
        ]
        
        for ex_x, ex_y, label in example_centers:
            if 0 < ex_x < frame.shape[1] and 0 < ex_y < frame.shape[0]:
                # 绘制示例四边形中心（小圆圈）
                cv2.circle(overlay, (ex_x, ex_y), 6, (200, 200, 200), 1)
                cv2.putText(overlay, "○", (ex_x - 3, ex_y + 3), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)
        
        # **绘制网格辅助线（帮助理解坐标系统）**
        grid_step = 60  # 每60像素画一条网格线
        for x in range(grid_step, frame.shape[1], grid_step):
            cv2.line(overlay, (x, 0), (x, frame.shape[0]), (80, 80, 80), 1)
        for y in range(grid_step, frame.shape[0], grid_step):
            cv2.line(overlay, (0, y), (frame.shape[1], y), (80, 80, 80), 1)
        
        # **绘制已标定的点及其含义**
        for i, point in enumerate(self.calibration_points):
            point_x, point_y = point['image_x'], point['image_y']
            
            # 计算该点相对于目标中心的偏差
            offset_x = point_x - center_x
            offset_y = point_y - center_y
            
            # 绘制标定点（红色圆圈，表示"模拟的四边形中心位置"）
            cv2.circle(overlay, (point_x, point_y), self.point_radius + 2, (0, 0, 255), 2)
            cv2.circle(overlay, (point_x, point_y), self.point_radius, (0, 0, 255), -1)
            
            # 标定点编号
            cv2.putText(overlay, str(i+1), 
                       (point_x + 12, point_y - 12),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # **绘制从目标中心到标定点的偏差向量（重要！）**
            cv2.arrowedLine(overlay, (center_x, center_y), (point_x, point_y), 
                           (255, 0, 255), 2, tipLength=0.3)
            
            # **显示偏差值和对应的角度增量**
            offset_text = f"偏差({offset_x:+d},{offset_y:+d})"
            delta_text = f"增量({point['delta_x']:+.1f}°,{point['delta_y']:+.1f}°)"
            
            cv2.putText(overlay, offset_text, 
                       (point_x + 15, point_y + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
            cv2.putText(overlay, delta_text, 
                       (point_x + 15, point_y + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
        
        # **显示当前激光位置和舵机状态**
        servo_text = f"激光位置: X={self.current_x_position * self.angle_per_unit:.1f}°, Y={self.current_y_position * self.angle_per_unit:.1f}°"
        cv2.putText(overlay, servo_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # **显示当前相对中心的角度增量**
        delta_x_angle, delta_y_angle = self.get_current_delta_angles()
        delta_text = f"当前增量: X={delta_x_angle:+.2f}°, Y={delta_y_angle:+.2f}°"
        cv2.putText(overlay, delta_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # **显示标定进度**
        progress_text = f"标定点: {len(self.calibration_points)}/9 (建议采集9个点)"
        color = (0, 255, 0) if len(self.calibration_points) >= 9 else (0, 255, 255)
        cv2.putText(overlay, progress_text, (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # **四边形中心点标定专用提示**
        help_text1 = "四边形中心标定: 激光=目标中心(绿), 点击=模拟检测中心(红), 箭头=偏差映射"
        cv2.putText(overlay, help_text1, (10, frame.shape[0] - 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
        
        help_text2 = "WASD:移动激光 | H:设中心 | R:重置 | 123:步长 | J:保存 | 点击:添加映射点"
        cv2.putText(overlay, help_text2, (10, frame.shape[0] - 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
        
        return overlay
    
    def mouse_callback(self, event, x, y, flags, param):
        """**修改：鼠标点击回调 - 明确四边形中心点的映射逻辑**"""
        if event == cv2.EVENT_LBUTTONDOWN:
            # **左键点击 - 添加四边形中心点映射数据**
            delta_x_angle, delta_y_angle = self.get_current_delta_angles()
            
            # 计算点击位置相对于图像中心的偏差
            image_center_x = self.image_width / 2
            image_center_y = self.image_height / 2
            offset_x = x - image_center_x
            offset_y = y - image_center_y
            
            point_data = {
                'image_x': x,
                'image_y': y,
                'delta_x': delta_x_angle,  # **关键：存储角度增量**
                'delta_y': delta_y_angle,  # **关键：存储角度增量**
                'center_x': self.center_x_position,  # 记录中心位置
                'center_y': self.center_y_position,
                'current_x': self.current_x_position,  # 记录当前位置
                'current_y': self.current_y_position,
                'offset_x': offset_x,     # **新增：记录偏差值**
                'offset_y': offset_y,     # **新增：记录偏差值**
                'timestamp': time.time()
            }
            
            self.calibration_points.append(point_data)
            
            print(f"📍 添加四边形中心映射点 #{len(self.calibration_points)}: "
                  f"模拟检测中心({x}, {y}) -> 偏差({offset_x:+.0f},{offset_y:+.0f})像素 -> 角度增量(Δ{delta_x_angle:.2f}°, Δ{delta_y_angle:.2f}°)")
            print(f"   含义: 当YOLO+OpenCV检测到四边形中心在({x},{y})时，舵机应调整({delta_x_angle:+.2f}°,{delta_y_angle:+.2f}°)")
        
        elif event == cv2.EVENT_RBUTTONDOWN:
            # 右键点击 - 删除最近的标定点
            if self.calibration_points:
                removed = self.calibration_points.pop()
                offset_x = removed.get('offset_x', removed['image_x'] - self.image_width/2)
                offset_y = removed.get('offset_y', removed['image_y'] - self.image_height/2)
                print(f"🗑️ 删除四边形中心映射点: "
                      f"检测中心({removed['image_x']}, {removed['image_y']}) -> "
                      f"偏差({offset_x:+.0f},{offset_y:+.0f}) -> 增量(Δ{removed['delta_x']:.2f}°, Δ{removed['delta_y']:.2f}°)")
            else:
                print("⚠️ 没有四边形中心映射点可删除")
    
    def save_calibration(self, filename="square_servo_calibration.json"):
        """保存标定数据"""
        if len(self.calibration_points) < 4:
            print(f"⚠️ 标定点不足，至少需要4个点，当前只有{len(self.calibration_points)}个")
            return False
        
        try:
            calibration_data = {
                'image_size': {
                    'width': self.image_width,
                    'height': self.image_height
                },
                'servo_config': {
                    'x_servo_id': self.x_servo_id,
                    'y_servo_id': self.y_servo_id,
                    'resolution': self.servo_resolution,
                    'angle_per_unit': self.angle_per_unit,
                    'position_min': self.position_min,
                    'position_max': self.position_max
                },
                'delta_limits': {
                    'max_delta_x': self.max_delta_angle_x,
                    'max_delta_y': self.max_delta_angle_y
                },
                'center_position': {
                    'x': self.center_x_position,
                    'y': self.center_y_position
                },
                'calibration_points': self.calibration_points,
                'timestamp': time.time(),
                'point_count': len(self.calibration_points)
            }
            
            with open(filename, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            print(f"✅ 标定数据已保存到: {filename}")
            print(f"✅ 共保存 {len(self.calibration_points)} 个标定点")
            print(f"✅ 中心位置: X={self.center_x_position}, Y={self.center_y_position}")
            return True
            
        except Exception as e:
            print(f"❌ 保存标定数据失败: {e}")
            return False
    
    def load_calibration(self, filename="square_servo_calibration.json"):
        """加载标定数据"""
        try:
            if not os.path.exists(filename):
                print(f"⚠️ 标定文件不存在: {filename}")
                return False
            
            with open(filename, 'r') as f:
                data = json.load(f)
            
            self.calibration_points = data.get('calibration_points', [])
            
            # 恢复中心位置
            center_pos = data.get('center_position', {})
            if center_pos:
                self.center_x_position = center_pos.get('x', 0)
                self.center_y_position = center_pos.get('y', 0)
            
            print(f"✅ 标定数据已加载: {filename}")
            print(f"✅ 加载了 {len(self.calibration_points)} 个标定点")
            print(f"✅ 中心位置: X={self.center_x_position}, Y={self.center_y_position}")
            return True
            
        except Exception as e:
            print(f"❌ 加载标定数据失败: {e}")
            return False
    
    def run(self):
        """**修改：增强的运行流程**"""
        print("🚀 启动方块跟踪坐标标定工具")
        print("="*60)
        
        # **新增：连接配置步骤**
        self.get_connection_config()
        
        print("📷 初始化摄像头...")
        if not self.init_camera():
            print("❌ 摄像头初始化失败，程序退出")
            return
        
        print("🎮 初始化串行总线舵机...")
        servo_connected = self.init_servo()
        
        if not servo_connected:
            print("\n❌ 舵机连接失败，程序将退出")
            print("💡 请检查连接配置并重新运行程序")
            if self.cap:
                self.cap.release()
            return
        
        # 尝试加载已有标定数据
        print("\n📁 检查已有标定数据...")
        self.load_calibration()
        
        # 显示帮助
        self.show_help()
        
        # 设置窗口和鼠标回调
        window_name = "Square Servo Calibration Tool"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        print("✅ 标定工具已启动")
        print("🎯 请使用WASD控制舵机移动，点击图像添加标定点")
        print("🔧 舵机控制: W=上升 S=下降 A=右转 D=左转")
        
        try:
            while True:
                # 读取摄像头帧
                ret, frame = self.cap.read()
                if not ret:
                    print("❌ 无法读取摄像头帧")
                    break
                
                # 绘制覆盖层
                display_frame = self.draw_overlay(frame)
                
                # 显示图像
                cv2.imshow(window_name, display_frame)
                
                # 处理键盘输入
                key = cv2.waitKey(1) & 0xFF
                
                if key == 27 or key == ord('q') or key == ord('Q'):  # ESC或Q键退出
                    break
                elif key == ord('j') or key == ord('J'):  # J键保存
                    self.save_calibration()
                elif key == ord('l') or key == ord('L'):  # L键加载
                    self.load_calibration()
                elif key != 255:  # 其他按键
                    self.handle_keyboard(key)
                
        except KeyboardInterrupt:
            print("\n⚠️ 收到中断信号")
        
        finally:
            # 清理资源
            if self.cap:
                self.cap.release()
            
            # **修复：安全断开舵机连接**
            if self.servo_controller and self.servo_connected:
                try:
                    print("🔌 正在断开舵机连接...")
                    self.servo_controller.disconnect()
                    print("✓ 舵机连接已安全断开")
                except Exception as e:
                    print(f"⚠️ 舵机断开异常: {e}")
            
            cv2.destroyAllWindows()
            print("🏁 标定工具已退出")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='方块跟踪串行总线舵机坐标标定工具')
    parser.add_argument('--camera-id', type=int, default=0, help='摄像头ID')
    parser.add_argument('--width', type=int, default=640, help='图像宽度')
    parser.add_argument('--height', type=int, default=480, help='图像高度')
    
    args = parser.parse_args()
    
    calibrator = SquareCoordinateCalibrator(
        camera_id=args.camera_id,
        image_width=args.width,
        image_height=args.height
    )
    
    calibrator.run()

if __name__ == "__main__":
    main()
