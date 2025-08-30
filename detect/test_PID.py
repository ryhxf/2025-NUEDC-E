#!/usr/bin/env python3

import time
import logging
import sys
import os
import threading
import random
import matplotlib.pyplot as plt
import numpy as np

# 添加模块路径
sys.path.append('/root/square/detect')
sys.path.append('/root/square/all_duoji')

# 导入PID控制模块
from pid_control import PIDController, DualAxisPIDController, PIDSquareTracker

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='[%(name)s] [%(asctime)s] [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("PIDTest")

class MockTracker:
    """模拟跟踪器核心 - 用于测试PID控制器"""
    
    def __init__(self):
        # 模拟参数
        self.is_tracking = False
        self.frame_count = 0
        self.servo_connected = True
        self.fast_response_mode = True
        
        # 模拟舵机状态
        self.current_x_position = 1024  # 90度位置
        self.current_y_position = 512   # 45度位置
        self.angle_per_unit = 360.0 / 4096
        self.x_servo_id = 1
        self.y_servo_id = 2
        
        # 控制相关
        self.control_interval = 0.02
        self.last_control_time = 0
        self.servo_moving = False
        self.move_start_time = 0
        self.target_x_angle = None
        self.target_y_angle = None
        
        # 模拟舵机控制器
        self.servo_controller = MockServoController()
        
        # 激光控制
        self.laser_shots = 0
        
        logger.info("模拟跟踪器核心已初始化")
    
    def control_laser(self, should_fire: bool):
        """模拟激光控制"""
        if should_fire:
            self.laser_shots += 1
            logger.debug(f"🔴 模拟激光射击 - 第{self.laser_shots}次")
        else:
            logger.debug("🔵 模拟激光关闭")
    
    def move_servo_to_angle_fast(self, target_x_angle: float, target_y_angle: float) -> bool:
        """模拟舵机移动"""
        success = self.servo_controller.write_servo_position(self.x_servo_id, target_x_angle, 30000)
        success &= self.servo_controller.write_servo_position(self.y_servo_id, target_y_angle, 30000)
        
        if success:
            # 更新位置
            self.current_x_position = int(target_x_angle / self.angle_per_unit)
            self.current_y_position = int(target_y_angle / self.angle_per_unit)
            logger.debug(f"模拟舵机移动: X={target_x_angle:.1f}°, Y={target_y_angle:.1f}°")
        
        return success

class MockServoController:
    """模拟舵机控制器"""
    
    def __init__(self):
        self.current_angles = {1: 90.0, 2: 45.0}  # 初始角度
        logger.debug("模拟舵机控制器已初始化")
    
    def write_servo_position(self, servo_id: int, angle: float, speed: int) -> bool:
        """模拟写入舵机位置"""
        # 添加一些随机延时模拟真实情况
        time.sleep(random.uniform(0.001, 0.005))
        
        # 限制角度范围
        angle = max(0, min(180, angle))
        self.current_angles[servo_id] = angle
        
        logger.debug(f"模拟舵机{servo_id}: {angle:.1f}°, 速度={speed}")
        return True
    
    def read_servo_position(self, servo_id: int) -> float:
        """模拟读取舵机位置"""
        return self.current_angles.get(servo_id, 90.0)

def test_basic_pid_controller():
    """测试基础PID控制器"""
    logger.info("="*50)
    logger.info("测试1: 基础PID控制器")
    logger.info("="*50)
    
    # 创建PID控制器
    pid = PIDController(kp=2.0, ki=0.1, kd=0.5, output_limits=(-10, 10))
    
    # 模拟系统响应
    target = 100.0  # 目标值
    current = 0.0   # 当前值
    dt = 0.02       # 时间步长
    
    time_data = []
    target_data = []
    current_data = []
    output_data = []
    
    logger.info(f"目标值: {target}, 初始值: {current}")
    
    for i in range(200):  # 运行4秒
        current_time = i * dt
        
        # 计算误差
        error = target - current
        
        # PID控制器输出
        output = pid.update(error, current_time)
        
        # 模拟系统响应（一阶惯性环节）
        current += output * dt * 0.8  # 系统增益
        current *= 0.98  # 系统阻尼
        
        # 记录数据
        time_data.append(current_time)
        target_data.append(target)
        current_data.append(current)
        output_data.append(output)
        
        if i % 50 == 0:
            logger.info(f"时间: {current_time:.2f}s, 当前值: {current:.2f}, 误差: {error:.2f}, 输出: {output:.2f}")
    
    # 计算性能指标
    final_error = abs(target - current)
    settling_time = None
    
    for i, val in enumerate(current_data):
        if abs(target - val) < target * 0.05:  # 5%误差范围
            settling_time = time_data[i]
            break
    
    logger.info(f"最终误差: {final_error:.2f}")
    logger.info(f"调节时间: {settling_time:.2f}s" if settling_time else "未稳定")
    
    return time_data, target_data, current_data, output_data

def test_dual_axis_pid_controller():
    """测试双轴PID控制器"""
    logger.info("="*50)
    logger.info("测试2: 双轴PID控制器")
    logger.info("="*50)
    
    # 创建双轴PID控制器
    dual_pid = DualAxisPIDController(
        x_gains=(2.0, 0.1, 0.5),
        y_gains=(2.0, 0.1, 0.5),
        output_limits=(-15.0, 15.0),
        sample_time=0.02
    )
    
    # 设置目标中心点
    target_x, target_y = 320, 240
    dual_pid.set_target_center(target_x, target_y)
    dual_pid.set_deadzone(5, 5)
    dual_pid.enable()
    
    # 模拟检测到的目标位置
    test_positions = [
        (100, 100),   # 左上角
        (500, 100),   # 右上角
        (500, 400),   # 右下角
        (100, 400),   # 左下角
        (320, 240),   # 中心
    ]
    
    logger.info(f"目标中心: ({target_x}, {target_y})")
    
    for i, (detected_x, detected_y) in enumerate(test_positions):
        logger.info(f"\n--- 测试位置 {i+1}: ({detected_x}, {detected_y}) ---")
        
        # 模拟连续控制
        for step in range(20):
            current_time = time.time()
            
            delta_x, delta_y, in_deadzone = dual_pid.update(detected_x, detected_y, current_time)
            
            if in_deadzone:
                logger.info(f"步骤{step+1}: 在死区内，停止移动")
                break
            else:
                logger.info(f"步骤{step+1}: 角度增量=({delta_x:+.2f}°, {delta_y:+.2f}°)")
            
            # 模拟目标逐渐移向中心
            detected_x += (target_x - detected_x) * 0.1
            detected_y += (target_y - detected_y) * 0.1
            
            time.sleep(0.02)
    
    # 获取状态信息
    status = dual_pid.get_status()
    logger.info(f"\n双轴PID状态:")
    logger.info(f"  控制次数: {status['control_count']}")
    logger.info(f"  目标中心: {status['target_center']}")
    logger.info(f"  死区: {status['deadzone']}")

def test_pid_square_tracker():
    """测试PID方块跟踪器"""
    logger.info("="*50)
    logger.info("测试3: PID方块跟踪器集成测试")
    logger.info("="*50)
    
    # 创建模拟跟踪器核心
    mock_tracker = MockTracker()
    
    # 创建PID跟踪器
    pid_config = {
        'x_gains': (2.0, 0.1, 0.5),
        'y_gains': (2.0, 0.1, 0.5),
        'output_limits': (-15.0, 15.0),
        'sample_time': 0.02,
        'deadzone_x': 8,
        'deadzone_y': 8,
        'target_center': (320, 240)
    }
    
    pid_tracker = PIDSquareTracker(mock_tracker, pid_config)
    
    # 启动PID跟踪
    pid_tracker.start_pid_tracking()
    
    # 模拟目标检测序列
    detection_sequence = [
        (280, 200),   # 轻微偏移
        (250, 180),   # 较大偏移
        (200, 150),   # 大偏移
        (300, 220),   # 接近中心
        (320, 240),   # 正中心
        (340, 260),   # 另一侧偏移
        (320, 240),   # 回到中心
    ]
    
    logger.info("开始模拟目标检测序列...")
    
    for i, (center_x, center_y) in enumerate(detection_sequence):
        logger.info(f"\n--- 检测序列 {i+1}: 目标位置 ({center_x}, {center_y}) ---")
        
        # 模拟视觉检测结果
        vision_result = {
            'method': 'pure_yolo',
            'confidence': 0.85,
            'bbox': [center_x-20, center_y-20, center_x+20, center_y+20]
        }
        
        # 更新帧计数
        mock_tracker.frame_count += 1
        
        # 调用PID控制方法
        mock_tracker.process_coordinate_mapping_control(center_x, center_y, vision_result)
        
        time.sleep(0.1)  # 模拟处理间隔
    
    # 停止PID跟踪
    pid_tracker.stop_pid_tracking()
    
    # 显示统计信息
    status = pid_tracker.get_status()
    logger.info(f"\nPID跟踪器状态:")
    logger.info(f"  活动状态: {status['active']}")
    logger.info(f"  配置: {status['config']}")
    logger.info(f"  激光射击次数: {mock_tracker.laser_shots}")

def test_performance_comparison():
    """测试不同PID参数的性能对比"""
    logger.info("="*50)
    logger.info("测试4: PID参数性能对比")
    logger.info("="*50)
    
    # 不同的PID参数组合
    pid_configs = [
        ("保守型", (1.0, 0.05, 0.2)),
        ("标准型", (2.0, 0.1, 0.5)),
        ("激进型", (4.0, 0.3, 1.0)),
        ("仅P控制", (2.0, 0.0, 0.0)),
        ("PI控制", (2.0, 0.1, 0.0)),
    ]
    
    target = 100.0
    initial_value = 0.0
    simulation_time = 3.0
    dt = 0.02
    steps = int(simulation_time / dt)
    
    results = {}
    
    for name, (kp, ki, kd) in pid_configs:
        logger.info(f"\n测试配置: {name} (Kp={kp}, Ki={ki}, Kd={kd})")
        
        pid = PIDController(kp=kp, ki=ki, kd=kd, output_limits=(-20, 20))
        current = initial_value
        
        time_data = []
        response_data = []
        
        for i in range(steps):
            current_time = i * dt
            error = target - current
            output = pid.update(error, current_time)
            
            # 模拟系统响应
            current += output * dt * 0.5
            current *= 0.99
            
            time_data.append(current_time)
            response_data.append(current)
        
        # 计算性能指标
        final_error = abs(target - current)
        overshoot = max(response_data) - target if max(response_data) > target else 0
        
        # 计算调节时间（进入5%误差范围的时间）
        settling_time = simulation_time
        for i, val in enumerate(response_data):
            if abs(target - val) < target * 0.05:
                settling_time = time_data[i]
                break
        
        results[name] = {
            'final_error': final_error,
            'overshoot': overshoot,
            'settling_time': settling_time,
            'time_data': time_data,
            'response_data': response_data
        }
        
        logger.info(f"  最终误差: {final_error:.2f}")
        logger.info(f"  超调量: {overshoot:.2f}")
        logger.info(f"  调节时间: {settling_time:.2f}s")
    
    # 显示性能对比表
    logger.info("\n--- 性能对比表 ---")
    logger.info(f"{'配置':<10} {'最终误差':<10} {'超调量':<10} {'调节时间':<10}")
    logger.info("-" * 45)
    for name, metrics in results.items():
        logger.info(f"{name:<10} {metrics['final_error']:<10.2f} {metrics['overshoot']:<10.2f} {metrics['settling_time']:<10.2f}")
    
    return results

def test_real_time_performance():
    """测试实时性能"""
    logger.info("="*50)
    logger.info("测试5: 实时性能测试")
    logger.info("="*50)
    
    # 创建双轴PID控制器
    dual_pid = DualAxisPIDController(
        x_gains=(2.0, 0.1, 0.5),
        y_gains=(2.0, 0.1, 0.5),
        output_limits=(-15.0, 15.0),
        sample_time=0.01  # 高频率控制
    )
    
    dual_pid.set_target_center(320, 240)
    dual_pid.set_deadzone(3, 3)
    dual_pid.enable()
    
    # 性能测量
    update_times = []
    iterations = 1000
    
    logger.info(f"执行{iterations}次PID更新，测量性能...")
    
    start_time = time.time()
    
    for i in range(iterations):
        # 模拟随机目标位置
        detected_x = 320 + random.uniform(-50, 50)
        detected_y = 240 + random.uniform(-50, 50)
        
        update_start = time.time()
        delta_x, delta_y, in_deadzone = dual_pid.update(detected_x, detected_y)
        update_end = time.time()
        
        update_times.append((update_end - update_start) * 1000)  # 转换为毫秒
    
    total_time = time.time() - start_time
    
    # 计算性能统计
    avg_update_time = np.mean(update_times)
    max_update_time = np.max(update_times)
    min_update_time = np.min(update_times)
    std_update_time = np.std(update_times)
    
    logger.info(f"性能测试结果:")
    logger.info(f"  总耗时: {total_time:.3f}s")
    logger.info(f"  平均更新时间: {avg_update_time:.3f}ms")
    logger.info(f"  最大更新时间: {max_update_time:.3f}ms")
    logger.info(f"  最小更新时间: {min_update_time:.3f}ms")
    logger.info(f"  时间标准差: {std_update_time:.3f}ms")
    logger.info(f"  理论最大频率: {1000/avg_update_time:.1f}Hz")
    
    return update_times

def run_all_tests():
    """运行所有测试"""
    logger.info("🚀 开始PID控制器完整测试套件")
    logger.info("="*60)
    
    try:
        # 测试1: 基础PID控制器
        time_data, target_data, current_data, output_data = test_basic_pid_controller()
        
        # 测试2: 双轴PID控制器
        test_dual_axis_pid_controller()
        
        # 测试3: PID方块跟踪器
        test_pid_square_tracker()
        
        # 测试4: 性能对比
        comparison_results = test_performance_comparison()
        
        # 测试5: 实时性能
        performance_data = test_real_time_performance()
        
        logger.info("="*60)
        logger.info("✅ 所有PID测试完成")
        logger.info("="*60)
        
        # 生成测试报告
        generate_test_report(time_data, current_data, comparison_results, performance_data)
        
    except Exception as e:
        logger.error(f"❌ 测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()

def generate_test_report(time_data, response_data, comparison_results, performance_data):
    """生成测试报告"""
    logger.info("📊 生成测试报告...")
    
    try:
        # 创建图形报告
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # 图1: 基础PID响应
        ax1.plot(time_data, response_data, 'b-', label='PID响应')
        ax1.axhline(y=100, color='r', linestyle='--', label='目标值')
        ax1.set_title('基础PID控制器响应')
        ax1.set_xlabel('时间 (s)')
        ax1.set_ylabel('输出值')
        ax1.legend()
        ax1.grid(True)
        
        # 图2: 性能对比
        names = list(comparison_results.keys())
        final_errors = [comparison_results[name]['final_error'] for name in names]
        
        ax2.bar(names, final_errors)
        ax2.set_title('不同PID参数最终误差对比')
        ax2.set_ylabel('最终误差')
        ax2.tick_params(axis='x', rotation=45)
        
        # 图3: 调节时间对比
        settling_times = [comparison_results[name]['settling_time'] for name in names]
        ax3.bar(names, settling_times)
        ax3.set_title('不同PID参数调节时间对比')
        ax3.set_ylabel('调节时间 (s)')
        ax3.tick_params(axis='x', rotation=45)
        
        # 图4: 实时性能分布
        ax4.hist(performance_data, bins=50, alpha=0.7)
        ax4.set_title('PID更新时间分布')
        ax4.set_xlabel('更新时间 (ms)')
        ax4.set_ylabel('频次')
        ax4.axvline(x=np.mean(performance_data), color='r', linestyle='--', 
                   label=f'平均值: {np.mean(performance_data):.2f}ms')
        ax4.legend()
        
        plt.tight_layout()
        
        # 保存图片
        report_file = '/root/square/detect/pid_test_report.png'
        plt.savefig(report_file, dpi=300, bbox_inches='tight')
        logger.info(f"📈 测试报告图片已保存: {report_file}")
        
        # 保存文本报告
        report_text_file = '/root/square/detect/pid_test_report.txt'
        with open(report_text_file, 'w', encoding='utf-8') as f:
            f.write("PID控制器测试报告\n")
            f.write("="*50 + "\n\n")
            
            f.write("1. 基础PID控制器测试\n")
            f.write(f"   最终响应值: {response_data[-1]:.2f}\n")
            f.write(f"   目标值: 100.0\n")
            f.write(f"   最终误差: {abs(100 - response_data[-1]):.2f}\n\n")
            
            f.write("2. 性能对比测试\n")
            for name, metrics in comparison_results.items():
                f.write(f"   {name}:\n")
                f.write(f"     最终误差: {metrics['final_error']:.2f}\n")
                f.write(f"     超调量: {metrics['overshoot']:.2f}\n")
                f.write(f"     调节时间: {metrics['settling_time']:.2f}s\n")
            f.write("\n")
            
            f.write("3. 实时性能测试\n")
            f.write(f"   平均更新时间: {np.mean(performance_data):.3f}ms\n")
            f.write(f"   最大更新时间: {np.max(performance_data):.3f}ms\n")
            f.write(f"   最小更新时间: {np.min(performance_data):.3f}ms\n")
            f.write(f"   理论最大频率: {1000/np.mean(performance_data):.1f}Hz\n")
        
        logger.info(f"📄 测试报告文本已保存: {report_text_file}")
        
    except ImportError:
        logger.warning("⚠️ matplotlib未安装，跳过图形报告生成")
    except Exception as e:
        logger.error(f"❌ 生成测试报告失败: {e}")

if __name__ == "__main__":
    print("🔬 PID控制器测试程序")
    print("="*50)
    
    try:
        run_all_tests()
        print("\n🎉 测试完成！检查日志了解详细结果。")
    except KeyboardInterrupt:
        print("\n⏹️ 测试被用户中断")
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        import traceback
        traceback.print_exc()
