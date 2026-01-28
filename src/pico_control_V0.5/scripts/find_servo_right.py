#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HTD-85H 舵机扫描工具
扫描串口上所有连接的舵机，显示ID、位置、电压、温度等信息

使用方法：
python3 find_servo.py --port /dev/ttyUSB0
python3 find_servo.py --range 1-50  # 扫描ID 1到50
"""

import serial
import time
import argparse
import sys

# 舵机协议常量
FRAME_HEADER = 0x55
CMD_READ_POS = 28      # 0x1C 读取位置
CMD_READ_VIN = 27      # 0x1B 读取电压
CMD_READ_TEMP = 26     # 0x1A 读取温度
CMD_READ_ID = 14       # 0x0E 读取ID

class ServoScanner:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=0.2):
        """初始化串口连接"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"✓ 成功连接到串口: {port}")
            print(f"⚙️  波特率: {baudrate}, 超时: {timeout}s")
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            sys.exit(1)
    
    def checksum(self, data):
        """计算校验和"""
        return (~sum(data[2:])) & 0xFF
    
    def send_read_command(self, servo_id, cmd, debug=False):
        """发送读取命令并获取响应"""
        packet = [FRAME_HEADER, FRAME_HEADER, servo_id, 3, cmd]
        #print("发送读取数据（send_read_command）：", packet)
        packet.append(self.checksum(packet))
        
        if debug:
            print(f"发送到ID{servo_id}: {' '.join([f'{b:02X}' for b in packet])}")
        
        # 清空缓冲区
        self.ser.flushInput()
        self.ser.write(bytes(packet))
        
        # 等待响应
        time.sleep(0.05)  # 给舵机足够的响应时间
        
        # 读取响应
        if self.ser.in_waiting > 0 or 1 == 1:
            data = self.ser.read(self.ser.in_waiting)
            print("收到原始数据（ser.read.send_read_command）：", data)
            print(f"数据大小: {len(data)} 字节")
            print(f"数据内容: {data}")
            print(f"收到响应(send_read_command): {data.hex().upper()}")
            if debug:
                print(f"收到响应: {data.hex().upper()}")
            return list(data) if data else None
        return None
    
    def read_position(self, servo_id, debug=False):
        """读取舵机位置"""
        response = self.send_read_command(servo_id, CMD_READ_POS, debug)
        
        if response and len(response) >= 7:
            if (response[0] == 0x55 and response[1] == 0x55 and 
                response[2] == servo_id and response[4] == CMD_READ_POS):
                position = response[5] + (response[6] << 8)
                return position
        return None
    
    def read_voltage(self, servo_id, debug=False):
        """读取舵机电压"""
        response = self.send_read_command(servo_id, CMD_READ_VIN, debug)
        
        if response and len(response) >= 7:
            if (response[0] == 0x55 and response[1] == 0x55 and 
                response[2] == servo_id and response[4] == CMD_READ_VIN):
                voltage_raw = response[5] + (response[6] << 8)
                voltage = voltage_raw / 1000.0  # 转换为伏特
                return voltage
        return None
    
    def read_temperature(self, servo_id, debug=False):
        """读取舵机温度"""
        response = self.send_read_command(servo_id, CMD_READ_TEMP, debug)
        
        if response and len(response) >= 6:
            if (response[0] == 0x55 and response[1] == 0x55 and 
                response[2] == servo_id and response[4] == CMD_READ_TEMP):
                temperature = response[5]  # 温度值
                return temperature
        return None
    
    def ping_servo(self, servo_id, debug=False):
        """ping舵机，检查是否在线"""
        position = self.read_position(servo_id, debug)
        return position is not None
    
    def scan_servo_detailed(self, servo_id, debug=False):
        """详细扫描单个舵机信息"""
        if debug:
            print(f"\n📡 扫描舵机ID {servo_id}...")
        
        # 读取位置
        position = self.read_position(servo_id, debug)
        if position is None:
            return None
        
        # 读取电压
        voltage = self.read_voltage(servo_id, debug)
        
        # 读取温度
        temperature = self.read_temperature(servo_id, debug)
        
        # 计算角度
        angle = (position / 1000.0) * 240.0 if position is not None else None
        
        servo_info = {
            'id': servo_id,
            'position': position,
            'angle': angle,
            'voltage': voltage,
            'temperature': temperature,
            'online': True
        }
        
        return servo_info
    
    def scan_range(self, start_id=1, end_id=253, fast_mode=False, debug=False):
        """扫描指定范围的舵机"""
        print(f"\n🔍 开始扫描舵机 (ID: {start_id}-{end_id})")
        print(f"⚡ 模式: {'快速模式' if fast_mode else '详细模式'}")
        print("=" * 80)
        
        found_servos = []
        scan_count = 0
        
        # 表头
        if fast_mode:
            print(f"{'ID':<4} {'状态':<8}")
            print("-" * 15)
        else:
            print(f"{'ID':<4} {'位置':<8} {'角度':<8} {'电压':<8} {'温度':<8} {'状态':<8}")
            print("-" * 60)
        
        for servo_id in range(start_id, end_id + 1):
            scan_count += 1
            
            if fast_mode:
                # 快速模式：只检查在线状态
                online = self.ping_servo(servo_id, debug)
                status = "✓ 在线" if online else "✗ 离线"
                print(f"{servo_id:<4} {status:<8}")
                
                if online:
                    found_servos.append(servo_id)
            else:
                # 详细模式：读取所有信息
                servo_info = self.scan_servo_detailed(servo_id, debug)
                
                if servo_info:
                    pos_str = f"{servo_info['position']}" if servo_info['position'] is not None else "N/A"
                    angle_str = f"{servo_info['angle']:.1f}°" if servo_info['angle'] is not None else "N/A"
                    volt_str = f"{servo_info['voltage']:.2f}V" if servo_info['voltage'] is not None else "N/A"
                    temp_str = f"{servo_info['temperature']}°C" if servo_info['temperature'] is not None else "N/A"
                    
                    print(f"{servo_id:<4} {pos_str:<8} {angle_str:<8} {volt_str:<8} {temp_str:<8} {'✓ 在线':<8}")
                    found_servos.append(servo_info)
                else:
                    print(f"{servo_id:<4} {'N/A':<8} {'N/A':<8} {'N/A':<8} {'N/A':<8} {'✗ 离线':<8}")
            
            # 显示进度
            if scan_count % 20 == 0 or servo_id == end_id:
                progress = (scan_count / (end_id - start_id + 1)) * 100
                print(f"📊 扫描进度: {progress:.1f}% ({scan_count}/{end_id - start_id + 1})")
        
        return found_servos
    
    def quick_scan(self, id_list=None, debug=False):
        """快速扫描指定ID列表"""
        if id_list is None:
            id_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]  # 默认扫描前10个
        
        print(f"\n⚡ 快速扫描指定舵机: {id_list}")
        print("=" * 50)
        
        found_servos = []
        
        print(f"{'ID':<4} {'位置':<8} {'角度':<8} {'状态':<8}")
        print("-" * 35)
        
        for servo_id in id_list:
            position = self.read_position(servo_id, debug)
            
            if position is not None:
                angle = (position / 1000.0) * 240.0
                print(f"{servo_id:<4} {position:<8} {angle:.1f}°{'':<3} {'✓ 在线':<8}")
                found_servos.append({'id': servo_id, 'position': position, 'angle': angle})
            else:
                print(f"{servo_id:<4} {'N/A':<8} {'N/A':<8} {'✗ 离线':<8}")
        
        return found_servos
    
    def auto_detect_servos(self, max_id=50, timeout_per_servo=0.1):
        """自动检测舵机，优化扫描速度"""
        print(f"\n🚀 自动检测舵机 (最大ID: {max_id})")
        print("=" * 40)
        
        # 临时设置更短的超时时间
        original_timeout = self.ser.timeout
        self.ser.timeout = timeout_per_servo
        
        found_servos = []
        
        for servo_id in range(1, max_id + 1):
            # 发送位置读取命令
            packet = [FRAME_HEADER, FRAME_HEADER, servo_id, 3, CMD_READ_POS]
            packet.append(self.checksum(packet))
            #print("发送读取数据：", packet)
            self.ser.flushInput()
            self.ser.write(bytes(packet))
            time.sleep(0.02)  # 短暂等待
            
            # 检查响应
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                print("收到原始数据：", data)
                print(f"收到响应: {data.hex().upper()}")
                if len(data) >= 7 and data[0] == 0x55 and data[1] == 0x55 and data[2] == servo_id:
                    position = data[5] + (data[6] << 8)
                    angle = (position / 1000.0) * 240.0
                    found_servos.append({'id': servo_id, 'position': position, 'angle': angle})
                    print(f"✓ 发现舵机ID {servo_id}: 位置={position}, 角度={angle:.1f}°")
            
            # 每扫描10个显示一次进度
            if servo_id % 10 == 0:
                print(f"📊 已扫描: {servo_id}/{max_id}")
        
        # 恢复原始超时时间
        self.ser.timeout = original_timeout
        
        return found_servos
    
    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("\n🔌 串口已关闭")

def parse_range(range_str):
    """解析范围字符串，如 "1-50" 或 "1,3,5-10" """
    ids = set()
    
    for part in range_str.split(','):
        if '-' in part:
            start, end = map(int, part.split('-'))
            ids.update(range(start, end + 1))
        else:
            ids.add(int(part))
    
    return sorted(list(ids))

def main():
    parser = argparse.ArgumentParser(description='HTD-85H 舵机扫描工具')
    parser.add_argument('--port', default='/dev/arm_right', help='串口设备')
    parser.add_argument('--baudrate', type=int, default=115200, help='波特率')
    parser.add_argument('--timeout', type=float, default=0.2, help='超时时间(秒)')
    parser.add_argument('--range', default='1-7', help='扫描范围 (例如: 1-50, 1,3,5-10)')
    parser.add_argument('--ids', help='快速扫描指定ID (例如: 1,2,3,4,5,6)')
    parser.add_argument('--fast', action='store_true', help='快速模式 (只检查在线状态)')
    parser.add_argument('--auto', action='store_true', help='自动检测模式 (优化速度)')
    parser.add_argument('--debug', action='store_true', help='调试模式')
    
    args = parser.parse_args()
    
    print("🔍 HTD-85H 舵机扫描工具")
    print("=" * 40)
    
    scanner = None
    try:
        scanner = ServoScanner(args.port, args.baudrate, args.timeout)
        
        if args.ids:
            # 快速扫描指定ID
            id_list = [int(x.strip()) for x in args.ids.split(',')]
            found_servos = scanner.quick_scan(id_list, args.debug)
            
        elif args.auto:
            # 自动检测模式
            max_id = 50
            if args.range:
                ids = parse_range(args.range)
                max_id = max(ids) if ids else 50
            found_servos = scanner.auto_detect_servos(max_id)
            
        else:
            # 范围扫描
            ids = parse_range(args.range)
            start_id, end_id = min(ids), max(ids)
            found_servos = scanner.scan_range(start_id, end_id, args.fast, args.debug)
        
        # 显示总结
        print("\n" + "=" * 60)
        print("📋 扫描结果总结")
        print("=" * 60)
        
        if found_servos:
            print(f"✅ 发现 {len(found_servos)} 个舵机:")
            
            if args.fast:
                print(f"   ID列表: {found_servos}")
            else:
                for servo in found_servos:
                    if isinstance(servo, dict):
                        print(f"   ID {servo['id']}: 位置={servo['position']}, 角度={servo.get('angle', 'N/A')}")
                    else:
                        print(f"   ID {servo}")
                        
            # 生成ID列表用于复制
            if isinstance(found_servos[0], dict):
                id_list = [servo['id'] for servo in found_servos]
            else:
                id_list = found_servos
            print(f"\n📋 ID列表 (复制用): {','.join(map(str, id_list))}")
        else:
            print("❌ 未发现任何舵机")
            print("\n🔧 故障排除建议:")
            print("1. 检查舵机供电 (6-12V)")
            print("2. 检查信号线连接 (白线→TXD)")
            print("3. 检查GND共地连接") 
            print("4. 尝试不同的波特率: --baudrate 9600")
            print("5. 增加超时时间: --timeout 0.5")
            
    except KeyboardInterrupt:
        print("\n⏹️  扫描被用户中断")
    except Exception as e:
        print(f"❌ 扫描出错: {e}")
    finally:
        if scanner:
            scanner.close()

if __name__ == "__main__":
    main()
