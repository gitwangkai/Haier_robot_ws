#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机角度录制器
实时录制舵机角度数据，保存为 JSON 格式

使用方法：
python3 servo_recorder.py <串口> <输出文件名>
例如：
python3 servo_recorder.py /dev/ttyUSB0 my_recording.json

退出方式：
 连按q 键停止录制
"""

import sys
import json
import time
import signal
import select
from servo_angle_reader import ServoReader

class ServoRecorder:
    def __init__(self, port='/dev/ttyUSB0', servo_ids=None):
        """初始化录制器
        
        Args:
            port (str): 串口端口
            servo_ids (list): 要录制的舵机ID列表，默认 [1,2,3,4,5,6,7]
        """
        self.port = port
        self.servo_ids = servo_ids if servo_ids else [1, 2, 3, 4, 5, 6, 7]
        self.reader = None
        self.recorded_data = []
        self.start_time = None
        self.is_recording = False
        self.last_valid_angles = {}  # 保存上次成功读取的角度
        self.should_stop = False  # 停止标志
        
    def start(self):
        """开始录制"""
        try:
            self.reader = ServoReader(self.port)
            self.start_time = time.time()
            self.is_recording = True
            print(f"✓ 开始录制舵机角度 (串口: {self.port})")
            print(f"  舵机ID: {self.servo_ids}")
            print(f"\n退出方式:")
            print(f"   连按q停止录制并保存")
        except Exception as e:
            print(f"✗ 初始化失败: {e}")
            raise
    
    def record_frame(self):
        """录制一帧数据"""
        if not self.is_recording or self.should_stop:
            return False
        
        try:
            # 计算相对时间
            current_time = time.time() - self.start_time
            
            # 读取所有舵机角度
            angles = {}
            for servo_id in self.servo_ids:
                servo_key = f"{servo_id:03d}"
                position = self.reader.read_position(servo_id)
                
                if position is not None:
                    # 读取成功，保存到 last_valid_angles
                    angles[servo_key] = position
                    self.last_valid_angles[servo_key] = position
                else:
                    # 读取失败，使用上次成功的值
                    if servo_key in self.last_valid_angles:
                        angles[servo_key] = self.last_valid_angles[servo_key]
                    elif self.recorded_data:
                        # 如果 last_valid_angles 中没有，从上一帧获取
                        last_angles = self.recorded_data[-1]["right_arm_angles"]
                        angles[servo_key] = last_angles.get(servo_key, 500)
                    else:
                        # 第一帧且读取失败，使用默认中位值
                        angles[servo_key] = 500
            
            # 构造数据帧
            frame = {
                "time": round(current_time, 4),
                "right_arm_angles": angles
            }
            
            self.recorded_data.append(frame)
            
            # 显示录制进度
            if len(self.recorded_data) % 10 == 0:
                print(f"⏺ 已录制: {len(self.recorded_data)} 帧, "
                      f"时长: {current_time:.2f}s, "
                      f"当前角度: {angles}")
            
            return True
            
        except Exception as e:
            print(f"✗ 录制出错: {e}")
            return False
    
    def save(self, filename):
        """保存录制的数据到 JSON 文件
        
        Args:
            filename (str): 输出文件名
        """
        try:
            # 确保文件名以 .json 结尾
            if not filename.endswith('.json'):
                filename += '.json'
            
            # 如果没有指定路径，保存到 arm_data 目录
            if '/' not in filename:
                filename = f"arm_data/{filename}"
            
            # 保存为 JSON 格式
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(self.recorded_data, f, indent=2, ensure_ascii=False)
            
            print(f"\n✓ 录制完成！")
            print(f"  总帧数: {len(self.recorded_data)}")
            if self.recorded_data:
                total_time = self.recorded_data[-1]['time']
                print(f"  总时长: {total_time:.2f} 秒")
                print(f"  平均帧率: {len(self.recorded_data)/total_time:.2f} FPS")
            print(f"  保存位置: {filename}")
            
        except Exception as e:
            print(f"✗ 保存失败: {e}")
            raise
    
    def stop(self):
        """停止录制"""
        self.should_stop = True
        self.is_recording = False
        if self.reader:
            self.reader.close()
        print("\n⏹ 停止录制")


def check_user_input():
    """检查用户输入（非阻塞）"""
    if select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline().strip().lower()
        if line in ['q', 'quit', 'exit', 'stop']:
            return True
    return False


def signal_handler(sig, frame):
    """处理 Ctrl+C 和其他信号"""
    print(f"\n\n⚠ 接收到停止信号 (signal {sig})...")
    global recorder, should_exit
    should_exit = True
    if recorder:
        recorder.should_stop = True
        recorder.stop()
    # 强制退出
    sys.exit(0)


def main():
    """主函数"""
    global recorder, should_exit
    should_exit = False
    recorder = None
    
    # 检查命令行参数
    if len(sys.argv) < 3:
        print("使用方法: python3 servo_recorder.py <串口> <输出文件名>")
        print("\n示例:")
        print("  python3 servo_recorder.py /dev/ttyUSB0 test_recording.json")
        print("  python3 servo_recorder.py /dev/ttyUSB0 my_data")
        print("\n可选参数:")
        print("  --servo-ids 1,2,3,4,5,6,7  指定要录制的舵机ID")
        print("\n说明:")
        print("  - 连按q停止录制并保存")
        print("  - 如果文件名不包含路径，将自动保存到 arm_data/ 目录")
        print("  - 如果文件名不以 .json 结尾，将自动添加")
        return
    
    port = sys.argv[1]
    output_file = sys.argv[2]
    
    # 解析舵机ID参数
    servo_ids = [1, 2, 3, 4, 5, 6, 7]
    if len(sys.argv) > 3 and sys.argv[3] == '--servo-ids':
        if len(sys.argv) > 4:
            servo_ids = [int(x) for x in sys.argv[4].split(',')]
    
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 创建录制器
    recorder = ServoRecorder(port, servo_ids)
    recorder.output_file = output_file
    
    try:
        # 开始录制
        recorder.start()
        
        # 持续录制
        frame_count = 0
        while not should_exit:
            # 检查用户输入
            if check_user_input():
                print("\n⚠ 用户请求停止录制")
                should_exit = True
                break
            
            # 录制一帧
            if not recorder.record_frame():
                break
            
            frame_count += 1
            # 每100帧提示一次可以输入 q 退出
            if frame_count % 100 == 0:
                print(f"    (连按q停止录制并保存)")
            
            time.sleep(0.07)  # 约 14 FPS，根据您的 JSON 数据时间间隔
            
    except KeyboardInterrupt:
        # Ctrl+C 已被 signal_handler 处理
        print("\n⚠ 收到键盘中断 (Ctrl+C)")
        should_exit = True
    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n正在保存数据...")
        if recorder and recorder.is_recording:
            recorder.stop()
        if recorder and recorder.recorded_data:
            try:
                recorder.save(output_file)
            except Exception as e:
                print(f"✗ 保存失败: {e}")
                # 尝试保存到当前目录
                try:
                    backup_file = f"backup_{int(time.time())}.json"
                    with open(backup_file, 'w') as f:
                        json.dump(recorder.recorded_data, f, indent=2)
                    print(f"✓ 数据已备份到: {backup_file}")
                except:
                    pass
        print("\n✓ 程序已退出")


if __name__ == "__main__":
    main()
