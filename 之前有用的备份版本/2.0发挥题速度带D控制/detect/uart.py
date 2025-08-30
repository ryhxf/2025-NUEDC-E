#!/usr/bin/env python3

import serial
import threading
import time
import logging
import queue
import signal
import sys

# **修复：从正确的模块导入**
from all_control import SquareTrackerCore
from all_control_function import load_default_config as load_config

logger = logging.getLogger("UARTController")

class UARTController:
    """串口控制器 - 处理串口通信和指令解析，支持预加载快速响应"""
    
    def __init__(self, port='/dev/ttyS1', baudrate=115200, timeout=1):
        """
        初始化串口控制器
        :param port: 串口设备路径
        :param baudrate: 波特率
        :param timeout: 超时时间
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        # 串口对象
        self.serial_conn = None
        self.is_connected = False
        
        # 线程控制
        self.running = False
        self.receive_thread = None
        
        # 指令队列
        self.command_queue = queue.Queue(maxsize=10)
        
        # 指令格式定义
        self.FRAME_HEAD = '@'
        self.FRAME_TAIL = '\r\n'
        
        # 支持的指令
        self.COMMANDS = {
            '2.': 'normal_mode',      # 正常执行模式
            '3.': 'search_mode',      # **新增：搜索模式**
            '4.': 'laser_high_mode',  # 激光常亮模式
            '5.': 'laser_high_mode',  # 激光常亮模式
            '6.': 'reserved_6',       # 保留功能6
            '7.': 'reserved_7'        # 保留功能7
        }
        
        # 当前模式
        self.current_mode = None
        self.laser_force_high = False
        
        # 统计信息
        self.received_commands = 0
        self.valid_commands = 0
        self.invalid_commands = 0
        
        # **修改：预加载跟踪器核心模块**
        self.tracker_core = None
        self.tracker_thread = None
        self.tracker_running = False
        self.tracker_ready = False  # **新增：跟踪器就绪状态**
        
        logger.info(f"串口控制器初始化: {port}, {baudrate}bps")
        
        # **新增：立即预加载跟踪器**
        self._preload_tracker()
    
    def _preload_tracker(self):
        """**新增：预加载跟踪器，提前初始化所有模型和设备**"""
        try:
            logger.info("🚀 开始预加载跟踪器核心...")
            start_time = time.time()
            
            # 创建跟踪器配置
            config = load_config()
            config['enable_display'] = False  # 串口模式下禁用显示
            
            logger.info("📦 正在初始化跟踪器核心（包含模型加载）...")
            self.tracker_core = SquareTrackerCore(config)
            
            # 设置外部控制模式
            self.tracker_core.set_external_control(True)
            self.tracker_core.set_laser_force_high(False)  # 默认正常模式
            
            logger.info("🔌 正在连接设备（摄像头+舵机）...")
            if not self.tracker_core.connect_devices():
                logger.error("❌ 预加载时设备连接失败")
                return False
            
            # **关键：启动跟踪线程，但暂停跟踪功能**
            logger.info("🔄 启动跟踪器工作线程（热备状态）...")
            self.tracker_core.is_tracking = False  # 暂停跟踪
            self.tracker_running = True
            self.tracker_thread = threading.Thread(target=self._tracker_worker, daemon=True)
            self.tracker_thread.start()
            
            # 等待跟踪器稳定
            time.sleep(1.0)
            
            self.tracker_ready = True
            load_time = time.time() - start_time
            
            logger.info(f"✅ 跟踪器预加载完成！耗时: {load_time:.2f}秒")
            logger.info("⚡ 现在串口指令可以实现秒级响应")
            
            return True
            
        except Exception as e:
            logger.error(f"❌ 预加载跟踪器失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def connect(self) -> bool:
        """连接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            if self.serial_conn.is_open:
                self.is_connected = True
                logger.info(f"✓ 串口连接成功: {self.port}")
                logger.info(f"  配置: {self.baudrate}bps, 8N1, timeout={self.timeout}s")
                return True
            else:
                logger.error(f"❌ 串口连接失败: {self.port}")
                return False
                
        except Exception as e:
            logger.error(f"❌ 串口连接异常: {e}")
            return False
    
    def disconnect(self):
        """**修改：断开串口连接 - 包含跟踪器清理**"""
        # 先停止跟踪器
        self._stop_tracker()
        
        # 然后停止串口监听
        self.stop_listening()
        
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
                logger.info("串口连接已断开")
            except Exception as e:
                logger.error(f"断开串口连接失败: {e}")
        
        self.is_connected = False
    
    def start_listening(self):
        """开始监听串口数据"""
        if not self.is_connected:
            logger.error("串口未连接，无法开始监听")
            return False
        
        if self.running:
            logger.warning("串口监听已在运行中")
            return True
        
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_worker, daemon=True)
        self.receive_thread.start()
        
        logger.info("串口监听线程已启动")
        return True
    
    def stop_listening(self):
        """停止监听串口数据"""
        self.running = False
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)
            logger.info("串口监听线程已停止")
    
    def _receive_worker(self):
        """串口数据接收工作线程"""
        receive_buffer = ""
        
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    # 读取数据
                    if self.serial_conn.in_waiting > 0:
                        data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                        receive_buffer += data
                        
                        # 查找完整的指令帧
                        while self.FRAME_TAIL in receive_buffer:
                            frame_end = receive_buffer.find(self.FRAME_TAIL)
                            frame = receive_buffer[:frame_end]
                            receive_buffer = receive_buffer[frame_end + len(self.FRAME_TAIL):]
                            
                            # 处理接收到的帧
                            self._process_frame(frame)
                    
                    time.sleep(0.01)  # 避免过度占用CPU
                else:
                    logger.warning("串口连接丢失，停止监听")
                    break
                    
            except Exception as e:
                logger.error(f"串口接收数据异常: {e}")
                time.sleep(0.1)
    
    def _process_frame(self, frame: str):
        """处理接收到的数据帧"""
        try:
            self.received_commands += 1
            
            # 检查帧头
            if not frame.startswith(self.FRAME_HEAD):
                logger.warning(f"无效帧头: {frame}")
                self.invalid_commands += 1
                return
            
            # 移除帧头
            command_content = frame[1:]  # 移除@符号
            
            logger.debug(f"接收到指令: {frame}")
            
            # 解析指令
            command_parsed = False
            for cmd_prefix, cmd_type in self.COMMANDS.items():
                if command_content.startswith(cmd_prefix):
                    self._handle_command(cmd_type, command_content)
                    command_parsed = True
                    break
            
            if not command_parsed:
                logger.warning(f"未知指令: {command_content}")
                self.invalid_commands += 1
            else:
                self.valid_commands += 1
            
        except Exception as e:
            logger.error(f"处理指令帧异常: {e}")
            self.invalid_commands += 1
    
    def _handle_command(self, command_type: str, full_command: str):
        """**修改：处理解析后的指令 - 4号5号指令启用D控制**"""
        try:
            logger.info(f"🎯 执行指令: {command_type} - {full_command}")
            
            if not self.tracker_ready:
                logger.warning("⚠️ 跟踪器未就绪，忽略指令")
                self.send_response("ERROR:tracker_not_ready")
                return
            
            # **快速响应：直接模式切换，无需重新初始化**
            response_start = time.time()
            
            if command_type == 'normal_mode':
                self.current_mode = 'normal'
                self.laser_force_high = False
                logger.info("✓ 切换到正常执行模式（速度30000，D控制禁用）")
                
                # **快速切换：只改变模式，不重启跟踪器**
                self.tracker_core.set_laser_force_high(False)
                self.tracker_core.set_servo_speed_mode('normal')  # 设置正常速度模式（禁用D控制）
                self.tracker_core.start_tracking()  # 开始跟踪
                
                self._add_command_to_queue('start_normal_mode')
                response_time = (time.time() - response_start) * 1000
                logger.info(f"⚡ 正常模式响应耗时: {response_time:.1f}ms")
                self.send_response("OK:normal_mode_started")
                
            elif command_type == 'search_mode':
                # **搜索模式：使用正常速度，禁用D控制**
                self.current_mode = 'search'
                self.laser_force_high = False
                logger.info("✓ 切换到搜索跟踪模式（速度30000，D控制禁用）")
                
                # 解析搜索指令参数
                search_params = self._parse_search_command(full_command)
                if search_params is None:
                    logger.error("❌ 搜索指令参数解析失败")
                    self.send_response("ERROR:invalid_search_params")
                    return
                
                # 设置搜索模式速度（禁用D控制）
                self.tracker_core.set_servo_speed_mode('normal')  # 搜索使用正常速度（禁用D控制）
                
                # 启动搜索模式
                success = self._start_search_mode(search_params)
                
                self._add_command_to_queue('start_search_mode')
                response_time = (time.time() - response_start) * 1000
                logger.info(f"⚡ 搜索模式响应耗时: {response_time:.1f}ms")
                
                if success:
                    self.send_response("OK:search_mode_started")
                else:
                    self.send_response("ERROR:search_mode_start_failed")
                
            elif command_type == 'laser_high_mode':
                self.current_mode = 'laser_high'
                self.laser_force_high = True
                logger.info("✓ 切换到激光常亮模式（速度32000，D控制启用）")
                
                # **关键修改：4号5号指令使用32000速度并启用D控制**
                self.tracker_core.set_laser_force_high(True)  # 激光强制高电平
                self.tracker_core.set_servo_speed_mode('high_speed')  # 设置高速模式32000（启用D控制）
                self.tracker_core.start_tracking()  # 开始跟踪
                
                self._add_command_to_queue('start_laser_high_mode')
                response_time = (time.time() - response_start) * 1000
                logger.info(f"⚡ 激光常亮模式响应耗时: {response_time:.1f}ms，使用32000速度+D控制预测")
                self.send_response("OK:laser_high_mode_started")
                
            elif command_type.startswith('reserved_'):
                logger.info(f"⚠️ 保留功能: {command_type}")
                self.send_response(f"INFO:reserved_function_{command_type}")
                
            else:
                logger.warning(f"未实现的指令类型: {command_type}")
                self.send_response("ERROR:unknown_command")
            
        except Exception as e:
            logger.error(f"处理指令异常: {e}")
            self.send_response("ERROR:command_processing_failed")
    
    def _parse_search_command(self, full_command: str):
        """**修正：解析搜索指令参数 - 360度格式转4096制**
        
        指令格式：
        @3.direction.angle\r\n - 按方向转指定角度后跟踪（angle是360度制角度值0-360）
        @3.direction.0\r\n - 按方向持续旋转搜索直到找到目标
        
        Args:
            full_command: 完整指令内容（不含@符号）
            
        Returns:
            dict: 搜索参数字典或None（解析失败）
        """
        try:
            # 移除指令前缀 "3."
            params_str = full_command[2:]  # 移除 "3."
            
            # 按 "." 分割参数
            parts = params_str.split('.')
            
            if len(parts) != 2:
                logger.error(f"搜索指令格式错误，需要2个参数: direction.angle，当前: {params_str}")
                return None
            
            # 解析方向参数
            try:
                direction = int(parts[0])
                if direction not in [0, 1]:
                    logger.error(f"无效的方向参数: {direction}，必须是0（左转）或1（右转）")
                    return None
            except ValueError:
                logger.error(f"方向参数必须是数字: {parts[0]}")
                return None
            
            # 解析角度参数（360度制）
            try:
                angle_360 = float(parts[1])  # 支持小数
                if not (0 <= angle_360 <= 360):
                    logger.error(f"无效的角度值: {angle_360}，必须在0-360度之间")
                    return None
            except ValueError:
                logger.error(f"角度参数必须是数字: {parts[1]}")
                return None
            
            # **关键修正：将360度制转换为4096制角度增量**
            angle_4096 = int((angle_360 / 360.0) * 4096.0)
            
            # 判断搜索模式
            if angle_360 == 0:
                # 持续搜索模式
                search_mode = 'continuous'
                angle_increment = None
                mode_desc = "持续搜索直到找到目标"
            else:
                # 固定角度模式
                search_mode = 'fixed_angle'
                angle_increment = angle_360  # 保持度数格式用于舵机控制
                mode_desc = f"转动{angle_360:.1f}°（转换为{angle_4096}/4096制）后跟踪"
            
            search_params = {
                'direction': direction,  # 0=左转, 1=右转
                'angle_360': angle_360,  # 原始360度制角度值
                'angle_4096': angle_4096,  # 转换后的4096制角度值
                'angle_degrees': angle_360,  # 实际度数（用于舵机控制）
                'angle_increment': angle_increment,  # 实际增减角度值（度）
                'search_mode': search_mode
            }
            
            direction_desc = "左转" if direction == 0 else "右转"
            logger.info(f"🔍 搜索参数: {direction_desc} {mode_desc}")
            logger.info(f"    角度转换: {angle_360}° (360制) → {angle_4096} (4096制)")
            
            return search_params
            
        except Exception as e:
            logger.error(f"解析搜索指令异常: {e}")
            return None
    
    def _start_search_mode(self, search_params: dict):
        """**修正：启动搜索模式 - 支持角度增量和持续搜索**"""
        try:
            if not self.tracker_core:
                logger.error("跟踪器核心未初始化")
                return False
            
            # 设置跟踪器为搜索模式
            self.tracker_core.set_laser_force_high(False)  # 搜索时不强制激光
            
            # 先停止当前跟踪
            self.tracker_core.stop_tracking()
            time.sleep(0.1)  # 短暂延时确保停止
            
            direction = search_params['direction']
            angle_increment = search_params.get('angle_increment')
            search_mode = search_params['search_mode']
            
            if search_mode == 'fixed_angle':
                # 固定角度模式：先转动指定角度，再开始跟踪
                logger.info(f"🔄 固定角度搜索: {'左转' if direction == 0 else '右转'} {angle_increment:.1f}°")
                
                # 执行角度旋转
                success = self._execute_angle_rotation(direction, angle_increment)
                if not success:
                    logger.error("❌ 角度旋转执行失败")
                    return False
                
                # 等待舵机移动完成
                time.sleep(0.5)
                
                # 开始正常跟踪
                logger.info("✅ 角度旋转完成，开始目标跟踪")
                self.tracker_core.start_tracking()
                
            else:
                # 持续搜索模式：启动搜索直到找到目标
                logger.info(f"🔍 持续搜索: {'左转' if direction == 0 else '右转'} 直到找到目标")
                
                # 启动持续搜索模式
                success = self._start_continuous_search(direction)
                if not success:
                    logger.error("❌ 持续搜索启动失败")
                    return False
            
            return True
                
        except Exception as e:
            logger.error(f"启动搜索模式异常: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _execute_angle_rotation(self, direction: int, angle_degrees: float) -> bool:
        """**新增：执行指定角度旋转**"""
        try:
            if not self.tracker_core.servo_controller or not self.tracker_core.servo_connected:
                logger.error("舵机控制器未连接")
                return False
            
            # 读取当前X轴角度
            current_x_angle = self.tracker_core.servo_controller.read_servo_position(self.tracker_core.x_servo_id)
            if current_x_angle is None:
                logger.error("无法读取当前X轴角度")
                return False
            
            # 计算目标角度
            if direction == 0:  # 左转
                target_x_angle = current_x_angle - angle_degrees
            else:  # 右转
                target_x_angle = current_x_angle + angle_degrees
            
            logger.info(f"🎯 执行角度旋转: {current_x_angle:.1f}° → {target_x_angle:.1f}° ({'左转' if direction == 0 else '右转'} {angle_degrees:.1f}°)")
            
            # 发送舵机移动指令（使用固定30000速度）
            success = self.tracker_core.servo_controller.write_servo_position(
                self.tracker_core.x_servo_id, 
                target_x_angle, 
                30000
            )
            
            if success:
                # 更新位置记录
                self.tracker_core.current_x_position = int(target_x_angle / self.tracker_core.angle_per_unit)
                logger.info(f"✅ 角度旋转指令已发送，速度: 30000")
                return True
            else:
                logger.error("❌ 角度旋转指令发送失败")
                return False
                
        except Exception as e:
            logger.error(f"执行角度旋转异常: {e}")
            return False
    
    def _start_continuous_search(self, direction: int) -> bool:
        """**新增：启动持续搜索模式**"""
        try:
            # 设置搜索相关参数
            self.tracker_core.search_mode_active = True
            self.tracker_core.search_direction = direction
            
            # 记录起始角度
            current_x_angle = self.tracker_core.servo_controller.read_servo_position(self.tracker_core.x_servo_id)
            if current_x_angle is None:
                logger.error("无法读取当前X轴角度作为搜索起点")
                return False
            
            self.tracker_core.search_start_angle_x = current_x_angle
            self.tracker_core.search_current_angle = current_x_angle
            self.tracker_core.search_last_step_time = time.time()
            
            # 启用视觉检测但不启用控制
            self.tracker_core.is_tracking = False  # 关键：视觉运行但不控制舵机
            
            # 启动搜索线程
            search_thread = threading.Thread(target=self._continuous_search_worker, args=(direction,), daemon=True)
            search_thread.start()
            
            logger.info(f"🔍 持续搜索已启动: {'左转' if direction == 0 else '右转'} 从{current_x_angle:.1f}°开始")
            return True
            
        except Exception as e:
            logger.error(f"启动持续搜索异常: {e}")
            return False
    
    def _continuous_search_worker(self, direction: int):
        """**新增：持续搜索工作线程**"""
        try:
            logger.info(f"🔍 持续搜索线程启动: {'左转' if direction == 0 else '右转'}")
            
            search_step_angle = 2.0  # 每次搜索步进角度
            search_pause_time = 0.3  # 每步暂停时间
            max_search_range = 90.0  # 最大搜索范围
            
            while self.tracker_core.search_mode_active:
                try:
                    # 检查是否找到目标
                    if self.tracker_core.target_found:
                        logger.info("🎯 持续搜索中发现目标，停止搜索，开始跟踪")
                        self.tracker_core.search_mode_active = False
                        self.tracker_core.start_tracking()  # 开始正常跟踪
                        break
                    
                    # 计算下一个搜索角度
                    if direction == 0:  # 左转
                        next_angle = self.tracker_core.search_current_angle - search_step_angle
                    else:  # 右转
                        next_angle = self.tracker_core.search_current_angle + search_step_angle
                    
                    # 检查搜索范围
                    angle_diff = abs(next_angle - self.tracker_core.search_start_angle_x)
                    if angle_diff > max_search_range:
                        logger.info(f"🔍 持续搜索范围已达上限({max_search_range}°)，停止搜索")
                        self.tracker_core.search_mode_active = False
                        break
                    
                    # 执行搜索步进
                    success = self.tracker_core.servo_controller.write_servo_position(
                        self.tracker_core.x_servo_id, 
                        float(next_angle), 
                        30000
                    )
                    
                    if success:
                        self.tracker_core.search_current_angle = next_angle
                        self.tracker_core.current_x_position = int(next_angle / self.tracker_core.angle_per_unit)
                        logger.debug(f"🔍 持续搜索步进: {next_angle:.1f}°")
                    else:
                        logger.error("持续搜索步进失败")
                        break
                    
                    # 等待下一步
                    time.sleep(search_pause_time)
                    
                except Exception as e:
                    logger.error(f"持续搜索步进异常: {e}")
                    break
            
            logger.info("🔍 持续搜索线程结束")
            
        except Exception as e:
            logger.error(f"持续搜索工作线程异常: {e}")
        finally:
            self.tracker_core.search_mode_active = False
    
    def _tracker_worker(self):
        """**新增：跟踪器工作线程**"""
        try:
            logger.info("🔄 跟踪器工作线程启动")
            
            while self.tracker_running and not self.tracker_core.should_stop:
                try:
                    # 执行一次跟踪循环
                    if not self.tracker_core.run_single_cycle():
                        logger.warning("跟踪循环返回False，可能需要停止")
                        break
                    
                    # 短暂休眠避免过度占用CPU
                    time.sleep(0.001)
                    
                except Exception as e:
                    logger.error(f"跟踪器工作线程异常: {e}")
                    time.sleep(0.1)
            
            logger.info("🔄 跟踪器工作线程结束")
            
        except Exception as e:
            logger.error(f"跟踪器工作线程严重异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.tracker_running = False
    
    def _stop_tracker(self):
        """**新增：停止跟踪器**"""
        try:
            if self.tracker_core:
                logger.info("🛑 停止跟踪器核心...")
                self.tracker_core.stop_core()
                
                # 等待跟踪线程结束
                if self.tracker_thread and self.tracker_thread.is_alive():
                    self.tracker_running = False
                    self.tracker_thread.join(timeout=3.0)
                    if self.tracker_thread.is_alive():
                        logger.warning("跟踪线程未能及时结束")
                    else:
                        logger.info("✓ 跟踪线程已结束")
                
                # 清理跟踪器资源
                logger.info("🧹 清理跟踪器资源...")
                try:
                    self.tracker_core.cleanup_gpio_resources()
                    self.tracker_core.disconnect_devices()
                except Exception as e:
                    logger.error(f"清理跟踪器核心时出错: {e}")
                
                self.tracker_core = None
                self.tracker_ready = False
                logger.info("✓ 跟踪器已停止")
                
        except Exception as e:
            logger.error(f"停止跟踪器异常: {e}")

    def send_response(self, response: str):
        """**新增：发送响应到串口**"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                response_with_tail = response + self.FRAME_TAIL
                self.serial_conn.write(response_with_tail.encode('utf-8'))
                logger.debug(f"发送响应: {response}")
            else:
                logger.warning(f"串口未连接，无法发送响应: {response}")
        except Exception as e:
            logger.error(f"发送响应异常: {e}")

    def _add_command_to_queue(self, command: str):
        """**新增：添加命令到队列**"""
        try:
            if not self.command_queue.full():
                self.command_queue.put(command, block=False)
                logger.debug(f"命令已加入队列: {command}")
            else:
                logger.warning(f"命令队列已满，丢弃命令: {command}")
        except Exception as e:
            logger.error(f"添加命令到队列异常: {e}")

    def get_command(self):
        """**新增：从队列获取命令**"""
        try:
            return self.command_queue.get(block=False)
        except queue.Empty:
            return None
        except Exception as e:
            logger.error(f"获取命令异常: {e}")
            return None

    def get_statistics(self):
        """**新增：获取统计信息**"""
        return {
            'received_commands': self.received_commands,
            'valid_commands': self.valid_commands,
            'invalid_commands': self.invalid_commands,
            'current_mode': self.current_mode,
            'laser_force_high': self.laser_force_high
        }

    def get_tracker_status(self):
        """**新增：获取跟踪器状态**"""
        if not self.tracker_core:
            return {
                'ready': False,
                'running': False,
                'tracking': False,
                'target_found': False,
                'frame_count': 0,
                'detection_count': 0,
                'laser_force_high': False
            }
        
        return {
            'ready': self.tracker_ready,
            'running': self.tracker_running,
            'tracking': self.tracker_core.is_tracking,
            'target_found': self.tracker_core.target_found,
            'frame_count': self.tracker_core.frame_count,
            'detection_count': self.tracker_core.detection_count,
            'laser_force_high': self.tracker_core.external_laser_force_high
        }

def test_uart_controller():
    """**修改：测试串口控制器 - 预加载模式测试**"""
    print("="*50)
    print("串口控制器 + 跟踪器集成测试（预加载快速响应模式）")
    print("="*50)
    
    # **修复：设置信号处理器 - 只在主线程中设置**
    def signal_handler(sig, frame):
        print("\n正在停止串口测试...")
        if 'uart_ctrl' in locals():
            uart_ctrl.disconnect()
        sys.exit(0)
    
    # **修复：检查是否在主线程中**
    try:
        signal.signal(signal.SIGINT, signal_handler)
        print("✓ 信号处理器设置成功")
    except ValueError as e:
        print(f"⚠️ 无法设置信号处理器（非主线程）: {e}")
        print("将使用其他方式处理中断")
    except Exception as e:
        print(f"⚠️ 信号处理器设置失败: {e}")
    
    # **修改：创建串口控制器（自动预加载）**
    print("🚀 创建串口控制器并预加载跟踪器...")
    uart_ctrl = UARTController()
    
    # **新增：等待预加载完成**
    print("⏳ 等待预加载完成...")
    wait_count = 0
    while not uart_ctrl.tracker_ready and wait_count < 30:  # 最多等待30秒
        time.sleep(1)
        wait_count += 1
        print(f"   预加载中... ({wait_count}/30)")
    
    if not uart_ctrl.tracker_ready:
        print("❌ 预加载失败或超时，退出测试")
        return
    
    print("✅ 预加载完成！")
    
    try:
        # 连接串口
        if not uart_ctrl.connect():
            print("串口连接失败，退出测试")
            return
        
        # 开始监听
        uart_ctrl.start_listening()
        
        print("\n🎯 串口测试已启动，等待指令...")
        print("📡 支持的指令格式:")
        print("  @2.xxx\\r\\n - 正常模式（秒级响应）")
        print("  @4.xxx\\r\\n - 激光常亮模式（秒级响应）")
        print("  @5.xxx\\r\\n - 激光常亮模式（秒级响应）")
        print("⚡ 现在指令响应速度已优化到毫秒级！")
        print("按 Ctrl+C 退出测试")
        
        # **增加：提示测试方法**
        print("\n🔬 测试方法:")
        print("1. 在另一个终端中执行: echo -e '@2.test\\r\\n' > /dev/ttyS1")
        print("2. 或者使用串口调试工具发送指令")
        print("3. 观察此窗口的输出信息，注意响应时间")
        
        # 主循环
        last_status_time = time.time()
        
        while True:
            # 检查是否有新指令
            command = uart_ctrl.get_command()
            if command:
                print(f"✅ 收到指令: {command}")
            
            # 每5秒显示一次状态信息
            if time.time() - last_status_time >= 5.0:
                stats = uart_ctrl.get_statistics()
                tracker_status = uart_ctrl.get_tracker_status()
                
                print(f"\n--- 状态更新 ---")
                print(f"串口: 模式={stats['current_mode']}, 指令: {stats['valid_commands']}/{stats['received_commands']}")
                print(f"跟踪器: 就绪={tracker_status['ready']}, 运行={tracker_status['running']}, 跟踪={tracker_status['tracking']}, 目标={tracker_status['target_found']}")
                print(f"处理: 帧数={tracker_status['frame_count']}, 检测={tracker_status['detection_count']}")
                if tracker_status.get('laser_force_high', False):
                    print("🔴 激光强制高电平模式")
                print("--- 状态结束 ---\n")
                
                last_status_time = time.time()
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        uart_ctrl.disconnect()
        print("串口测试结束")

if __name__ == "__main__":
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='[%(name)s] [%(asctime)s] [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S'
    )
    
    test_uart_controller()
