#!/usr/bin/env python3
"""
串口快速响应测试脚本
测试预加载模式下的指令响应时间
"""

import time
import serial
import threading
import sys

def test_quick_response():
    """测试快速响应功能"""
    print("? 串口快速响应测试")
    print("="*50)
    
    try:
        # 连接串口
        ser = serial.Serial('/dev/ttyS1', 115200, timeout=1)
        print(f"? 串口连接成功: {ser.name}")
        
        # 测试指令列表
        test_commands = [
            ('@2.normal_test\r\n', '正常模式'),
            ('@4.laser_test\r\n', '激光常亮模式'),
            ('@2.normal_again\r\n', '再次正常模式'),
            ('@5.laser_again\r\n', '再次激光模式'),
        ]
        
        print(f"\n? 将发送 {len(test_commands)} 个测试指令...")
        print("?? 正在测试指令响应时间...\n")
        
        total_response_time = 0
        successful_responses = 0
        
        for i, (command, description) in enumerate(test_commands):
            print(f"? 测试 {i+1}/{len(test_commands)}: {description}")
            
            # 发送指令并计时
            start_time = time.time()
            ser.write(command.encode('utf-8'))
            
            # 等待响应
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            response_time = (time.time() - start_time) * 1000
            
            if response:
                total_response_time += response_time
                successful_responses += 1
                status = "? 成功" if "OK:" in response else "?? 响应"
                print(f"   {status} | 耗时: {response_time:.1f}ms | 回复: {response}")
            else:
                print(f"   ? 无响应 | 耗时: {response_time:.1f}ms")
            
            # 指令间隔
            time.sleep(0.5)
        
        # 统计结果
        print(f"\n? 测试结果统计:")
        print(f"   总指令数: {len(test_commands)}")
        print(f"   成功响应: {successful_responses}")
        print(f"   成功率: {successful_responses/len(test_commands)*100:.1f}%")
        
        if successful_responses > 0:
            avg_response_time = total_response_time / successful_responses
            print(f"   平均响应时间: {avg_response_time:.1f}ms")
            
            if avg_response_time < 100:
                print("? 响应速度: 优秀 (< 100ms)")
            elif avg_response_time < 500:
                print("? 响应速度: 良好 (< 500ms)")
            elif avg_response_time < 1000:
                print("?? 响应速度: 一般 (< 1s)")
            else:
                print("? 响应速度: 较慢 (≥ 1s)")
        
        ser.close()
        print(f"\n? 测试完成")
        
    except Exception as e:
        print(f"? 测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("请确保已启动 uart.py 并完成预加载")
    input("按回车键开始测试...")
    test_quick_response()
