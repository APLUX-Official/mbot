#!/usr/bin/env python3
import serial
import time

# 修改为你的串口号和波特率
#SERIAL_PORT_LEFT = '/dev/ttyUSB1'
#SERIAL_PORT_RIGHT = '/dev/ttyUSB2'
SERIAL_PORT_LEFT = '/dev/drive_left'
SERIAL_PORT_RIGHT = '/dev/drive_right'
BAUDRATE = 115200

# 轮子方向系数（如需测试方向，可都设为1）
LEFT_WHEEL_DIR = 1
RIGHT_WHEEL_DIR = 1

def open_serial(port):
    try:
        ser = serial.Serial(port, BAUDRATE, timeout=0.5)
        print(f"成功打开串口: {port}")
        return ser
    except serial.SerialException as e:
        print(f"未能打开串口 {port}: {e}")
        return None

def main():
    ser_left = open_serial(SERIAL_PORT_LEFT)
    ser_right = open_serial(SERIAL_PORT_RIGHT)
    if not ser_left or not ser_right:
        print("串口打开失败，退出。")
        return
    print("开始读取编码器数据，按Ctrl+C退出...")
    try:
        while True:
            # 只读取左轮串口缓冲区最新一行
            if ser_left.in_waiting > 0:
                try:
                    while ser_left.in_waiting > 0:
                        line = ser_left.readline().decode('utf-8', errors='ignore').replace('\x00', '').strip()
                    if line:
                        print(f"[左轮原始] {line}")
                except Exception as e:
                    print(f"[左轮] 读取错误: {e}")
            # 只读取右轮串口缓冲区最新一行
            if ser_right.in_waiting > 0:
                try:
                    while ser_right.in_waiting > 0:
                        line = ser_right.readline().decode('utf-8', errors='ignore').replace('\x00', '').strip()
                    if line:
                        print(f"[右轮原始] {line}")
                except Exception as e:
                    print(f"[右轮] 读取错误: {e}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n退出。")
    finally:
        if ser_left: ser_left.close()
        if ser_right: ser_right.close()

if __name__ == '__main__':
    main()
