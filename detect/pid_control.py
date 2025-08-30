#!/usr/bin/env python3

import time
import logging
from typing import Tuple, Optional, Dict, Any
import threading

logger = logging.getLogger("PIDController")

class PIDController:
    """PID控制器 - 用于精确的目标跟踪控制"""
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0, 
                 output_limits: Tuple[float, float] = (-180.0, 180.0),
                 sample_time: float = 0.01):
        """
        初始化PID控制器
        
        Args:
            kp: 比例增益
            ki: 积分增益  
            kd: 微分增益
            output_limits: 输出限制 (min, max)
            sample_time: 采样时间间隔
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.sample_time = sample_time
        
        # PID状态变量
        self.reset()
        
        logger.info(f"PID控制器初始化: Kp={kp}, Ki={ki}, Kd={kd}")
        logger.info(f"输出限制: {output_limits}, 采样时间: {sample_time}s")
    
    def reset(self):
        """重置PID控制器状态"""
        self.last_error = 0.0
        self.last_time = None
        self.integral = 0.0
        self.last_output = 0.0
        
        logger.debug("PID控制器状态已重置")
    
    def update(self, error: float, current_time: Optional[float] = None) -> float:
        """
        更新PID控制器并计算输出
        
        Args:
            error: 当前误差值 (目标值 - 当前值)
            current_time: 当前时间戳，如果为None则使用系统时间
            
        Returns:
            PID控制器输出值
        """
        if current_time is None:
            current_time = time.time()
        
        # 初始化时间
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = error
            return 0.0
        
        # 计算时间间隔
        dt = current_time - self.last_time
        
        # 检查采样时间
        if dt < self.sample_time:
            return self.last_output
        
        # 比例项
        proportional = self.kp * error
        
        # 积分项
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # 微分项
        if dt > 0:
            derivative = self.kd * (error - self.last_error) / dt
        else:
            derivative = 0.0
        
        # 计算总输出
        output = proportional + integral + derivative
        
        # 应用输出限制
        if self.output_limits:
            output = max(self.output_limits[0], min(output, self.output_limits[1]))
            
            # 防止积分饱和
            if output != proportional + integral + derivative:
                self.integral -= error * dt  # 回滚积分项
        
        # 更新状态
        self.last_error = error
        self.last_time = current_time
        self.last_output = output
        
        return output
    
    def set_gains(self, kp: float, ki: float, kd: float):
        """设置PID增益参数"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        logger.info(f"PID增益已更新: Kp={kp}, Ki={ki}, Kd={kd}")
    
    def get_gains(self) -> Tuple[float, float, float]:
        """获取当前PID增益参数"""
        return self.kp, self.ki, self.kd
    
    def get_status(self) -> Dict[str, Any]:
        """获取PID控制器状态信息"""
        return {
            'kp': self.kp,
            'ki': self.ki, 
            'kd': self.kd,
            'last_error': self.last_error,
            'integral': self.integral,
            'last_output': self.last_output,
            'output_limits': self.output_limits,
            'sample_time': self.sample_time
        }

class DualAxisPIDController:
    """双轴PID控制器 - 用于X/Y轴独立控制"""
    
    def __init__(self, 
                 x_gains: Tuple[float, float, float] = (2.0, 0.1, 0.5),
                 y_gains: Tuple[float, float, float] = (2.0, 0.1, 0.5),
                 output_limits: Tuple[float, float] = (-10.0, 10.0),
                 sample_time: float = 0.02):
        """
        初始化双轴PID控制器
        
        Args:
            x_gains: X轴PID增益 (Kp, Ki, Kd)
            y_gains: Y轴PID增益 (Kp, Ki, Kd)
            output_limits: 输出限制 (min, max) 度数
            sample_time: 采样时间间隔
        """
        self.x_pid = PIDController(*x_gains, output_limits, sample_time)
        self.y_pid = PIDController(*y_gains, output_limits, sample_time)
        
        self.target_center = (320, 240)  # 默认目标中心点
        self.deadzone_x = 5  # X轴死区像素
        self.deadzone_y = 5  # Y轴死区像素
        
        # 控制状态
        self.enabled = False
        self.last_update_time = 0
        self.control_count = 0
        
        logger.info("双轴PID控制器初始化完成")
        logger.info(f"X轴增益: {x_gains}, Y轴增益: {y_gains}")
        logger.info(f"输出限制: {output_limits}°, 采样时间: {sample_time}s")
    
    def set_target_center(self, center_x: float, center_y: float):
        """设置目标中心点"""
        self.target_center = (center_x, center_y)
        logger.debug(f"目标中心点已设置: ({center_x}, {center_y})")
    
    def set_deadzone(self, deadzone_x: int, deadzone_y: int):
        """设置死区大小"""
        self.deadzone_x = deadzone_x
        self.deadzone_y = deadzone_y
        logger.info(f"死区已设置: X={deadzone_x}px, Y={deadzone_y}px")
    
    def enable(self):
        """启用PID控制"""
        self.enabled = True
        self.reset()
        logger.info("PID控制已启用")
    
    def disable(self):
        """禁用PID控制"""
        self.enabled = False
        logger.info("PID控制已禁用")
    
    def reset(self):
        """重置PID控制器状态"""
        self.x_pid.reset()
        self.y_pid.reset()
        self.last_update_time = 0
        self.control_count = 0
        logger.debug("双轴PID控制器状态已重置")
    
    def update(self, detected_x: float, detected_y: float, 
               current_time: Optional[float] = None) -> Tuple[float, float, bool]:
        """
        更新PID控制器并计算舵机角度增量
        
        Args:
            detected_x: 检测到的目标X坐标
            detected_y: 检测到的目标Y坐标
            current_time: 当前时间戳
            
        Returns:
            (delta_x_angle, delta_y_angle, in_deadzone): 角度增量和是否在死区内
        """
        if not self.enabled:
            return 0.0, 0.0, False
        
        if current_time is None:
            current_time = time.time()
        
        # 计算误差 (目标位置 - 当前位置)
        error_x = self.target_center[0] - detected_x
        error_y = self.target_center[1] - detected_y
        
        # 检查是否在死区内
        in_deadzone = (abs(error_x) <= self.deadzone_x and 
                      abs(error_y) <= self.deadzone_y)
        
        if in_deadzone:
            logger.debug(f"目标在死区内: 误差=({error_x:.1f}, {error_y:.1f})px")
            return 0.0, 0.0, True
        
        # 更新PID控制器
        delta_x_angle = self.x_pid.update(error_x, current_time)
        delta_y_angle = self.y_pid.update(error_y, current_time)
        
        self.control_count += 1
        self.last_update_time = current_time
        
        logger.debug(f"PID输出: 误差=({error_x:.1f}, {error_y:.1f})px, "
                    f"角度增量=({delta_x_angle:.2f}°, {delta_y_angle:.2f}°)")
        
        return delta_x_angle, delta_y_angle, False
    
    def set_gains(self, x_gains: Tuple[float, float, float], 
                  y_gains: Tuple[float, float, float]):
        """设置双轴PID增益"""
        self.x_pid.set_gains(*x_gains)
        self.y_pid.set_gains(*y_gains)
        logger.info(f"双轴PID增益已更新: X轴={x_gains}, Y轴={y_gains}")
    
    def get_status(self) -> Dict[str, Any]:
        """获取双轴PID控制器状态"""
        return {
            'enabled': self.enabled,
            'target_center': self.target_center,
            'deadzone': (self.deadzone_x, self.deadzone_y),
            'control_count': self.control_count,
            'last_update_time': self.last_update_time,
            'x_pid_status': self.x_pid.get_status(),
            'y_pid_status': self.y_pid.get_status()
        }

class PIDSquareTracker:
    """基于PID控制的方块跟踪器"""
    
    def __init__(self, tracker_core, pid_config: Optional[Dict[str, Any]] = None):
        """
        初始化PID方块跟踪器
        
        Args:
            tracker_core: 跟踪器核心实例
            pid_config: PID配置参数
        """
        self.tracker_core = tracker_core
        
        # 默认PID配置
        default_config = {
            'x_gains': (2.5, 0.1, 0.8),  # X轴PID参数
            'y_gains': (2.5, 0.1, 0.8),  # Y轴PID参数
            'output_limits': (-15.0, 15.0),  # 输出角度限制
            'sample_time': 0.02,  # 采样时间
            'deadzone_x': 8,  # X轴死区像素
            'deadzone_y': 8,  # Y轴死区像素
            'target_center': (320, 240)  # 目标中心点
        }
        
        if pid_config:
            default_config.update(pid_config)
        
        self.config = default_config
        
        # 创建双轴PID控制器
        self.pid_controller = DualAxisPIDController(
            x_gains=self.config['x_gains'],
            y_gains=self.config['y_gains'],
            output_limits=self.config['output_limits'],
            sample_time=self.config['sample_time']
        )
        
        # 设置参数
        self.pid_controller.set_target_center(*self.config['target_center'])
        self.pid_controller.set_deadzone(self.config['deadzone_x'], self.config['deadzone_y'])
        
        # 控制状态
        self.active = False
        self.lock = threading.Lock()
        
        logger.info("PID方块跟踪器初始化完成")
    
    def start_pid_tracking(self):
        """启动PID跟踪"""
        with self.lock:
            self.active = True
            self.pid_controller.enable()
            
            # 替换跟踪器的控制方法
            self.original_control_method = self.tracker_core.process_coordinate_mapping_control
            self.tracker_core.process_coordinate_mapping_control = self._pid_control_method
            
            logger.info("🎯 PID跟踪已启动")
    
    def stop_pid_tracking(self):
        """停止PID跟踪"""
        with self.lock:
            self.active = False
            self.pid_controller.disable()
            
            # 恢复原始控制方法
            if hasattr(self, 'original_control_method'):
                self.tracker_core.process_coordinate_mapping_control = self.original_control_method
            
            logger.info("⏹️ PID跟踪已停止")
    
    def _pid_control_method(self, center_x: float, center_y: float, vision_result: dict):
        """PID控制方法 - 替换原始的坐标映射控制"""
        if not self.active:
            return
        
        current_time = time.time()
        
        # 控制频率限制
        if current_time - self.tracker_core.last_control_time < self.tracker_core.control_interval:
            return
        
        try:
            # 使用PID控制器计算角度增量
            delta_x_angle, delta_y_angle, in_deadzone = self.pid_controller.update(
                center_x, center_y, current_time
            )
            
            if in_deadzone:
                # 在死区内，检查是否应该射击
                error_x = abs(center_x - self.config['target_center'][0])
                error_y = abs(center_y - self.config['target_center'][1])
                total_error = (error_x**2 + error_y**2)**0.5
                
                should_fire = total_error <= 5.0  # 5像素内射击
                self.tracker_core.control_laser(should_fire)
                
                logger.debug(f"PID死区内: 误差={total_error:.1f}px, 激光={'射击' if should_fire else '待机'}")
                self.tracker_core.last_control_time = current_time
                return
            
            # 获取当前舵机角度
            if self.tracker_core.fast_response_mode:
                current_x_angle = self.tracker_core.current_x_position * self.tracker_core.angle_per_unit
                current_y_angle = self.tracker_core.current_y_position * self.tracker_core.angle_per_unit
            else:
                current_x_angle = self.tracker_core.servo_controller.read_servo_position(self.tracker_core.x_servo_id)
                current_y_angle = self.tracker_core.servo_controller.read_servo_position(self.tracker_core.y_servo_id)
                
                if current_x_angle is None or current_y_angle is None:
                    current_x_angle = self.tracker_core.current_x_position * self.tracker_core.angle_per_unit
                    current_y_angle = self.tracker_core.current_y_position * self.tracker_core.angle_per_unit
            
            # 计算目标角度
            target_x_angle = current_x_angle + delta_x_angle
            target_y_angle = current_y_angle + delta_y_angle
            
            # 执行舵机移动
            logger.debug(f"🎯 PID舵机控制: 当前({current_x_angle:.1f}°,{current_y_angle:.1f}°) "
                        f"-> 目标({target_x_angle:.1f}°,{target_y_angle:.1f}°)")
            
            success = self.tracker_core.move_servo_to_angle_fast(target_x_angle, target_y_angle)
            
            if success:
                # 更新位置状态
                if self.tracker_core.fast_response_mode:
                    self.tracker_core.current_x_position = int(target_x_angle / self.tracker_core.angle_per_unit)
                    self.tracker_core.current_y_position = int(target_y_angle / self.tracker_core.angle_per_unit)
                    self.tracker_core.servo_moving = False
                else:
                    self.tracker_core.servo_moving = True
                    self.tracker_core.move_start_time = current_time
                    self.tracker_core.target_x_angle = target_x_angle
                    self.tracker_core.target_y_angle = target_y_angle
                    self.tracker_core.current_x_position = int(target_x_angle / self.tracker_core.angle_per_unit)
                    self.tracker_core.current_y_position = int(target_y_angle / self.tracker_core.angle_per_unit)
                
                # 移动时关闭激光
                self.tracker_core.control_laser(False)
                
                # 定期输出日志
                if self.tracker_core.frame_count % 15 == 0:
                    target_center = self.config['target_center']
                    offset_x = center_x - target_center[0]
                    offset_y = center_y - target_center[1]
                    
                    logger.info(f"PID控制: 检测=({center_x:.0f},{center_y:.0f}), "
                              f"偏差=({offset_x:+.0f},{offset_y:+.0f})px, "
                              f"角度增量=({delta_x_angle:+.2f}°,{delta_y_angle:+.2f}°)")
            else:
                logger.error("❌ PID舵机移动指令发送失败")
            
            self.tracker_core.last_control_time = current_time
            
        except Exception as e:
            logger.error(f"PID控制方法异常: {e}")
    
    def update_config(self, new_config: Dict[str, Any]):
        """更新PID配置"""
        with self.lock:
            self.config.update(new_config)
            
            # 更新PID参数
            if 'x_gains' in new_config or 'y_gains' in new_config:
                self.pid_controller.set_gains(
                    self.config.get('x_gains', (2.5, 0.1, 0.8)),
                    self.config.get('y_gains', (2.5, 0.1, 0.8))
                )
            
            # 更新其他参数
            if 'target_center' in new_config:
                self.pid_controller.set_target_center(*new_config['target_center'])
            
            if 'deadzone_x' in new_config or 'deadzone_y' in new_config:
                self.pid_controller.set_deadzone(
                    new_config.get('deadzone_x', 8),
                    new_config.get('deadzone_y', 8)
                )
            
            logger.info("PID配置已更新")
    
    def get_status(self) -> Dict[str, Any]:
        """获取PID跟踪器状态"""
        with self.lock:
            return {
                'active': self.active,
                'config': self.config,
                'pid_status': self.pid_controller.get_status()
            }
