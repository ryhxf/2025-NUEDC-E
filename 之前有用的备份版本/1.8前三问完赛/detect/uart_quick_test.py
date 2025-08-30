#!/usr/bin/env python3
"""
���ڿ�����Ӧ���Խű�
����Ԥ����ģʽ�µ�ָ����Ӧʱ��
"""

import time
import serial
import threading
import sys

def test_quick_response():
    """���Կ�����Ӧ����"""
    print("? ���ڿ�����Ӧ����")
    print("="*50)
    
    try:
        # ���Ӵ���
        ser = serial.Serial('/dev/ttyS1', 115200, timeout=1)
        print(f"? �������ӳɹ�: {ser.name}")
        
        # ����ָ���б�
        test_commands = [
            ('@2.normal_test\r\n', '����ģʽ'),
            ('@4.laser_test\r\n', '���ⳣ��ģʽ'),
            ('@2.normal_again\r\n', '�ٴ�����ģʽ'),
            ('@5.laser_again\r\n', '�ٴμ���ģʽ'),
        ]
        
        print(f"\n? ������ {len(test_commands)} ������ָ��...")
        print("?? ���ڲ���ָ����Ӧʱ��...\n")
        
        total_response_time = 0
        successful_responses = 0
        
        for i, (command, description) in enumerate(test_commands):
            print(f"? ���� {i+1}/{len(test_commands)}: {description}")
            
            # ����ָ���ʱ
            start_time = time.time()
            ser.write(command.encode('utf-8'))
            
            # �ȴ���Ӧ
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            response_time = (time.time() - start_time) * 1000
            
            if response:
                total_response_time += response_time
                successful_responses += 1
                status = "? �ɹ�" if "OK:" in response else "?? ��Ӧ"
                print(f"   {status} | ��ʱ: {response_time:.1f}ms | �ظ�: {response}")
            else:
                print(f"   ? ����Ӧ | ��ʱ: {response_time:.1f}ms")
            
            # ָ����
            time.sleep(0.5)
        
        # ͳ�ƽ��
        print(f"\n? ���Խ��ͳ��:")
        print(f"   ��ָ����: {len(test_commands)}")
        print(f"   �ɹ���Ӧ: {successful_responses}")
        print(f"   �ɹ���: {successful_responses/len(test_commands)*100:.1f}%")
        
        if successful_responses > 0:
            avg_response_time = total_response_time / successful_responses
            print(f"   ƽ����Ӧʱ��: {avg_response_time:.1f}ms")
            
            if avg_response_time < 100:
                print("? ��Ӧ�ٶ�: ���� (< 100ms)")
            elif avg_response_time < 500:
                print("? ��Ӧ�ٶ�: ���� (< 500ms)")
            elif avg_response_time < 1000:
                print("?? ��Ӧ�ٶ�: һ�� (< 1s)")
            else:
                print("? ��Ӧ�ٶ�: ���� (�� 1s)")
        
        ser.close()
        print(f"\n? �������")
        
    except Exception as e:
        print(f"? ����ʧ��: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("��ȷ�������� uart.py �����Ԥ����")
    input("���س�����ʼ����...")
    test_quick_response()
