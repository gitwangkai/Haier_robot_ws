#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机角度读取器
支持左右臂舵机角度读取，互不影响

使用方法：
from servo_angle_reader import ServoReader

# 右臂
right_reader = ServoReader('/dev/ttyUSB0')
angle = right_reader.get_angle(1)
right_reader.close()

# 左臂
left_reader = ServoReader('/dev/ttyUSB10')
angle = left_reader.get_angle(1)
left_reader.close()
"""

import serial
import time

# 舵机协议常量
FRAME_HEADER = 0x55
CMD_READ_POS = 28      # 0x1C 读取位置

class ServoReader:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=0.2):
        """初始化串口连接"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"✓ 成功连接到串口: {port}")
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            raise
    
    def checksum(self, data):
        """计算校验和"""
        return (~sum(data[2:])) & 0xFF
    
    def send_read_command(self, servo_id, cmd):
        """发送读取命令并获取响应"""
        packet = [FRAME_HEADER, FRAME_HEADER, servo_id, 3, cmd]
        packet.append(self.checksum(packet))
        
        # 清空缓冲区
        self.ser.flushInput()
        self.ser.write(bytes(packet))
        
        # 等待响应
        time.sleep(0.05)  # 给舵机足够的响应时间
        
        # 读取响应
        if self.ser.in_waiting > 0:
            data = self.ser.read(self.ser.in_waiting)
            return list(data) if data else None
        return None
    
    def read_position(self, servo_id):
        """读取舵机位置"""
        response = self.send_read_command(servo_id, CMD_READ_POS)
        
        if response and len(response) >= 7:
            if (response[0] == 0x55 and response[1] == 0x55 and 
                response[2] == servo_id and response[4] == CMD_READ_POS):
                position = response[5] + (response[6] << 8)
                return position
        return None
    
    def get_angle(self, servo_id):
        """获取舵机角度"""
        position = self.read_position(servo_id)
        if position is not None:
            angle = (position / 1000.0) * 240.0
            return round(angle, 2)
        return None
    
    def get_all_angles(self, servo_ids=None):
        """获取所有舵机的角度"""
        if servo_ids is None:
            servo_ids = [1, 2, 3, 4, 5, 6, 7]
        
        angles = {}
        for servo_id in servo_ids:
            angle = self.get_angle(servo_id)
            angles[servo_id] = angle
        return angles
    
    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"🔌 串口 {self.port} 已关闭")

def read_arm_angles(port='/dev/ttyUSB0', servo_ids=None):
    """读取指定串口上所有舵机的角度
    
    Args:
        port (str): 串口端口
        servo_ids (list): 舵机ID列表，默认 [1,2,3,4,5,6,7]
    
    Returns:
        dict: {servo_id: angle} 的字典，angle 为 None 表示读取失败
    """
    if servo_ids is None:
        servo_ids = [1, 2, 3, 4, 5, 6, 7]
    
    reader = None
    try:
        reader = ServoReader(port)
        angles = reader.get_all_angles(servo_ids)
        return angles
    except Exception as e:
        print(f"读取角度失败: {e}")
        return {sid: None for sid in servo_ids}
    finally:
        if reader:
            reader.close()

def main():
    """测试函数"""
    import sys
    
    if len(sys.argv) < 2:
        print("使用方法: python3 servo_angle_reader.py <port>")
        print("例如: python3 servo_angle_reader.py /dev/ttyUSB0")
        return
    
    port = sys.argv[1]
    
    angles = read_arm_angles(port)
    print(f"舵机角度: {angles}")

if __name__ == "__main__":
    while True:
        main()
        time.sleep(0.1)
