#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO
import time

def signal_handler(signal, frame):
    sys.exit(0)

# ����ʹ�õ�GPIOͨ��Ϊ37
output_pin = 37 # BOARD ���� 37

def main():
    # ���ùܽű���ģʽΪӲ����� BOARD
    GPIO.setmode(GPIO.BOARD)
    # ����Ϊ���ģʽ�����ҳ�ʼ��Ϊ�ߵ�ƽ
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    # ��¼��ǰ�ܽ�״̬
    curr_value = GPIO.HIGH
    print("Starting demo now! Press CTRL+C to exit")
    try:
        # ���1��ʱ�䣬ѭ������LED������
        while True:
            time.sleep(1)
            GPIO.output(output_pin, curr_value)
            curr_value ^= GPIO.HIGH
    finally:
        GPIO.cleanup()

if __name__=='__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()