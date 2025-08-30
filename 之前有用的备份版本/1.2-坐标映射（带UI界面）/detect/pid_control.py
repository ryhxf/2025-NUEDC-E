import time
import logging

logger = logging.getLogger("PID_Controller")

class PIDController:
    """PID控制器类"""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_min: float = -180, output_max: float = 180,
                 integral_limit: float = 100):
        """
        初始化PID控制器
        :param kp: 比例增益
        :param ki: 积分增益
        :param kd: 微分增益
        :param output_min: 输出最小值
        :param output_max: 输出最大值
        :param integral_limit: 积分限幅
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        
        # 内部状态
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
        # 统计信息
        self.error_history = []
        self.output_history = []
        
        logger.info(f"PID控制器初始化: Kp={kp}, Ki={ki}, Kd={kd}")
    
    def update(self, current_value: float, target_value: float) -> float:
        """
        更新PID控制器
        :param current_value: 当前值
        :param target_value: 目标值
        :return: 控制输出
        """
        current_time = time.time()
        
        # 计算误差
        error = target_value - current_value
        
        # 计算时间间隔
        if self.last_time is None:
            dt = 0.0
        else:
            dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.001  # 避免除零
        
        # 比例项
        proportional = self.kp * error
        
        # 积分项
        self.integral += error * dt
        # 积分限幅
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
        
        integral_term = self.ki * self.integral
        
        # 微分项
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0.0
        
        derivative_term = self.kd * derivative
        
        # 计算总输出
        output = proportional + integral_term + derivative_term
        
        # 输出限幅
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        
        # 更新状态
        self.previous_error = error
        self.last_time = current_time
        
        # 记录历史
        self.error_history.append(error)
        self.output_history.append(output)
        
        # 保持历史记录长度
        if len(self.error_history) > 100:
            self.error_history.pop(0)
            self.output_history.pop(0)
        
        logger.debug(f"PID更新: 误差={error:.2f}, 输出={output:.2f}")
        
        return output
    
    def reset(self):
        """重置PID控制器状态"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.error_history.clear()
        self.output_history.clear()
        logger.info("PID控制器已重置")
    
    def set_gains(self, kp: float, ki: float, kd: float):
        """动态调整PID参数"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        logger.info(f"PID参数更新: Kp={kp}, Ki={ki}, Kd={kd}")
    
    def get_status(self) -> dict:
        """获取控制器状态"""
        return {
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd,
            'integral': self.integral,
            'previous_error': self.previous_error,
            'output_range': [self.output_min, self.output_max],
            'integral_limit': self.integral_limit
        }

class DualAxisPIDController:
    """双轴PID控制器 - 用于X轴和Y轴同时控制"""
    
    def __init__(self, x_params: dict, y_params: dict):
        """
        初始化双轴PID控制器
        :param x_params: X轴PID参数 {'kp': 1.0, 'ki': 0.1, 'kd': 0.05, 'output_min': -180, 'output_max': 180}
        :param y_params: Y轴PID参数
        """
        self.x_pid = PIDController(**x_params)
        self.y_pid = PIDController(**y_params)
        
        # 目标中心点 (图像中心)
        self.target_x = 320  # 默认640x480图像的中心
        self.target_y = 240
        
        logger.info("双轴PID控制器初始化完成")
    
    def set_target_center(self, x: int, y: int):
        """设置目标中心点"""
        self.target_x = x
        self.target_y = y
        logger.info(f"目标中心点设置为: ({x}, {y})")
    
    def update(self, current_x: float, current_y: float) -> tuple[float, float]:
        """
        更新双轴控制器
        :param current_x: 当前X坐标
        :param current_y: 当前Y坐标
        :return: (x_output, y_output) 控制输出
        """
        x_output = self.x_pid.update(current_x, self.target_x)
        y_output = self.y_pid.update(current_y, self.target_y)
        
        return x_output, y_output
    
    def reset(self):
        """重置所有PID控制器"""
        self.x_pid.reset()
        self.y_pid.reset()
        logger.info("双轴PID控制器已重置")
    
    def get_status(self) -> dict:
        """获取控制器状态"""
        return {
            'target': [self.target_x, self.target_y],
            'x_pid': self.x_pid.get_status(),
            'y_pid': self.y_pid.get_status()
        }

class ServoPositionController:
    """舵机位置控制器 - 将PID输出转换为舵机角度"""
    
    def __init__(self, x_servo_id: int, y_servo_id: int, 
                 x_center_angle: float = 180, y_center_angle: float = 180,
                 x_range: float = 90, y_range: float = 90):
        """
        初始化舵机位置控制器
        :param x_servo_id: X轴舵机ID
        :param y_servo_id: Y轴舵机ID  
        :param x_center_angle: X轴中心角度
        :param y_center_angle: Y轴中心角度
        :param x_range: X轴活动范围 (±范围)
        :param y_range: Y轴活动范围 (±范围)
        """
        self.x_servo_id = x_servo_id
        self.y_servo_id = y_servo_id
        
        self.x_center_angle = x_center_angle
        self.y_center_angle = y_center_angle
        
        self.x_range = x_range
        self.y_range = y_range
        
        # 当前角度
        self.current_x_angle = x_center_angle
        self.current_y_angle = y_center_angle
        
        logger.info(f"舵机位置控制器初始化: X舵机ID={x_servo_id}, Y舵机ID={y_servo_id}")
    
    def update_angles(self, x_delta: float, y_delta: float) -> tuple[float, float]:
        """
        根据PID输出更新舵机角度
        :param x_delta: X轴增量
        :param y_delta: Y轴增量
        :return: (x_angle, y_angle) 新的舵机角度
        """
        # 更新角度
        self.current_x_angle += x_delta * 0.1  # 调整响应速度
        self.current_y_angle += y_delta * 0.1
        
        # 角度限制
        self.current_x_angle = max(self.x_center_angle - self.x_range, 
                                  min(self.x_center_angle + self.x_range, self.current_x_angle))
        self.current_y_angle = max(self.y_center_angle - self.y_range,
                                  min(self.y_center_angle + self.y_range, self.current_y_angle))
        
        return self.current_x_angle, self.current_y_angle
    
    def reset_to_center(self):
        """重置到中心位置"""
        self.current_x_angle = self.x_center_angle
        self.current_y_angle = self.y_center_angle
        logger.info("舵机位置重置到中心")
    
    def get_servo_commands(self, x_delta: float, y_delta: float) -> list[dict]:
        """
        获取舵机控制命令
        :param x_delta: X轴增量
        :param y_delta: Y轴增量
        :return: 舵机命令列表
        """
        x_angle, y_angle = self.update_angles(x_delta, y_delta)
        
        commands = [
            {
                'servo_id': self.x_servo_id,
                'angle': x_angle,
                'speed': 1000
            },
            {
                'servo_id': self.y_servo_id,
                'angle': y_angle,
                'speed': 1000
            }
        ]
        
        return commands

def load_velocity_control_config() -> dict:
    """为速度控制模式加载优化的PID配置"""
    return {
        # 速度控制模式下的优化PID参数
        'x_pid_params': {
            'kp': 1.2,    # 适当增加比例增益，提高响应速度
            'ki': 0.1,    # 保持积分增益，消除稳态误差
            'kd': 0.2,    # 增加微分增益，减少超调
            'output_min': -100,  # 扩大输出范围
            'output_max': 100,
            'integral_limit': 50  # 限制积分饱和
        },
        'y_pid_params': {
            'kp': 1.2,
            'ki': 0.1,
            'kd': 0.2,
            'output_min': -100,
            'output_max': 100,
            'integral_limit': 50
        }
    }

if __name__ == "__main__":
    # 测试PID控制器
    logging.basicConfig(level=logging.INFO)
    
    # 单轴PID测试
    pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
    
    print("单轴PID测试:")
    target = 100
    current = 50
    
    for i in range(10):
        output = pid.update(current, target)
        current += output * 0.1  # 模拟系统响应
        print(f"步骤 {i+1}: 当前值={current:.2f}, 输出={output:.2f}")
        time.sleep(0.1)
    
    # 双轴PID测试
    print("\n双轴PID测试:")
    dual_pid = DualAxisPIDController(
        x_params={'kp': 1.0, 'ki': 0.1, 'kd': 0.05},
        y_params={'kp': 1.0, 'ki': 0.1, 'kd': 0.05}
    )
    
    dual_pid.set_target_center(320, 240)
    
    current_x, current_y = 200, 150
    
    for i in range(10):
        x_out, y_out = dual_pid.update(current_x, current_y)
        current_x += x_out * 0.01
        current_y += y_out * 0.01
        print(f"步骤 {i+1}: 位置=({current_x:.1f}, {current_y:.1f}), 输出=({x_out:.2f}, {y_out:.2f})")
        time.sleep(0.1)
