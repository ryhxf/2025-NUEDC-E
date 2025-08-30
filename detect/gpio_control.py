#!/usr/bin/env python3
import Hobot.GPIO as GPIO
import time
import subprocess
import os
import sys
import signal

# GPIO配置
GPIO_PIN = 18
GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_PIN, GPIO.IN)

def signal_handler(signal, frame):
    print("\n正在清理GPIO资源...")
    GPIO.cleanup()
    sys.exit(0)

def is_uart_running():
    """检查uart.py是否正在运行"""
    try:
        subprocess.check_output(["pgrep", "-f", "uart.py"])
        return True
    except subprocess.CalledProcessError:
        return False

def start_uart():
    """启动uart.py程序"""
    try:
        subprocess.Popen(["/usr/bin/python3", "/root/square/detect/uart.py"])
        print("uart.py 已启动")
    except Exception as e:
        print(f"启动uart.py失败: {e}")

def stop_uart():
    """停止uart.py程序"""
    try:
        subprocess.run(["pkill", "-f", "uart.py"])
        print("uart.py 已停止")
    except Exception as e:
        print(f"停止uart.py失败: {e}")

def main():
    prev_value = None
    
    print("GPIO控制程序启动! 按 CTRL+C 退出")
    print(f"监控GPIO引脚: {GPIO_PIN}")
    
    try:
        while True:
            # 读取GPIO引脚状态
            current_value = GPIO.input(GPIO_PIN)
            
            # 只在状态改变时执行操作
            if current_value != prev_value:
                if current_value == GPIO.HIGH:
                    print("GPIO HIGH - 检查并启动 uart.py")
                    if not is_uart_running():
                        start_uart()
                    else:
                        print("uart.py 已经在运行中")
                else:
                    print("GPIO LOW - 检查并停止 uart.py")
                    if is_uart_running():
                        stop_uart()
                    else:
                        print("uart.py 已经停止")
                
                prev_value = current_value
            
            time.sleep(0.5)  # 减少检查间隔提高响应速度
            
    except Exception as e:
        print(f"程序运行出错: {e}")
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    # 设置信号处理器，优雅地处理Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    main()