#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多舵机臂控制测试系统 (最终UI增强版)
"""

import serial
import time
import sys
import os
from datetime import datetime
import json
from filter_tool import moving_average_filter_and_save # 导入滤波函数

# (常量定义)
FRAME_HEADER = 0x55
CMD_READ_POS = 28
CMD_UNLOAD = 31
CMD_MOVE = 1
CMD_READ_TEMP = 26
RIGHT_ARM_PORT = "/dev/ttyUSB2"
LEFT_ARM_PORT = "/dev/ttyUSB1"
SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]

RIGHT_ARM_IDS = SERVO_IDS
LEFT_ARM_IDS = SERVO_IDS

class MultiServoController:
    # (MultiServoController 类的代码保持不变，此处为简洁省略)
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, servo_ids=None, connect_serial=True, arm_name='right'):
        self.port = port
        self.baudrate = baudrate
        self.servo_ids = servo_ids or SERVO_IDS
        self.running = False
        self.ser = None
        self.arm_name = arm_name
        if connect_serial:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.1)
                print(f"✓ 成功连接到串口: {port}")
                print(f"📡 {arm_name} 臂舵机ID: {self.servo_ids}")
            except Exception as e:
                print(f"✗ {arm_name} 臂串口连接失败: {e}")
                self.ser = None
        else:
            print(f"📊 {arm_name} 臂进入数据处理模式（无串口连接）")

    def checksum(self, data):
        return (~sum(data[2:])) & 0xFF

    def send_command(self, servo_id, cmd, data=None, debug=False):
        if not self.ser: return False
        if data is None: data = []
        packet = [FRAME_HEADER, FRAME_HEADER, servo_id, 3 + len(data), cmd]
        packet.extend(data)
        packet.append(self.checksum(packet))
        if debug: print(f"发送到舵机{servo_id}: {' '.join([f'{b:02X}' for b in packet])}")
        try:
            self.ser.write(bytes(packet))
            return True
        except Exception as e:
            # print(f"发送命令到舵机 {servo_id} 失败: {e}")
            return False

    def read_response(self, debug=False):
        if self.ser and self.ser.in_waiting > 0:
            data = self.ser.read(self.ser.in_waiting)
            if debug: print(f"收到响应: {data.hex().upper()}")
            return list(data)
        return None

    def send_read_command(self, servo_id, cmd, debug=False):
        if not self.ser: return None
        packet = [FRAME_HEADER, FRAME_HEADER, servo_id, 3, cmd]
        packet.append(self.checksum(packet))
        self.ser.flushInput()
        self.ser.write(bytes(packet))
        time.sleep(0.005) # 优化读取延时
        return self.read_response(debug)

    def read_position(self, servo_id, debug=False):
        response = self.send_read_command(servo_id, CMD_READ_POS, debug)
        if response and len(response) >= 7 and response[0:2] == [0x55, 0x55] and response[4] == CMD_READ_POS:
            return response[5] + (response[6] << 8)
        return None

    def read_all_positions(self, debug=False):
        positions = {}
        for servo_id in self.servo_ids:
            positions[servo_id] = self.read_position(servo_id, debug)
        return positions

    def read_temperature(self, servo_id, debug=False):
        response = self.send_read_command(servo_id, CMD_READ_TEMP, debug)
        if response and len(response) >= 6 and response[0:2] == [0x55, 0x55] and response[4] == CMD_READ_TEMP:
            return response[5]
        return None

    def read_all_temperatures(self, debug=False):
        temperatures = {}
        for servo_id in self.servo_ids:
            temperatures[servo_id] = self.read_temperature(servo_id, debug)
        return temperatures

    def unload_all_servos(self, debug=False):
        print(f"🔓 正在卸载 {self.arm_name} 臂所有舵机...")
        for servo_id in self.servo_ids:
            self.send_command(servo_id, CMD_UNLOAD, [0], debug)
            time.sleep(0.01)
        print(f"🎯 {self.arm_name} 臂所有舵机已卸力。")

    def load_all_servos(self, debug=False):
        print(f"🔒 正在装载 {self.arm_name} 臂所有舵机...")
        for servo_id in self.servo_ids:
            self.send_command(servo_id, CMD_UNLOAD, [1], debug)
            time.sleep(0.01)
        print(f"🎯 {self.arm_name} 臂所有舵机已上力。")

    def move_servo(self, servo_id, position, duration=100, debug=False):
        position = max(0, min(1000, int(position)))
        pos_low = position & 0xFF
        pos_high = (position >> 8) & 0xFF
        time_low = duration & 0xFF
        time_high = (duration >> 8) & 0xFF
        data = [pos_low, pos_high, time_low, time_high]
        return self.send_command(servo_id, CMD_MOVE, data, debug)

    def close(self):
        if self.ser and self.ser.is_open:
            print(f"🔌 正在关闭串口 {self.port}...")
            self.ser.close()


def get_action_file_path(filename):
    arm_data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_data")
    if not os.path.exists(arm_data_dir):
        os.makedirs(arm_data_dir)
    return os.path.join(arm_data_dir, filename)

# ★★★ 修正版：健壮且清晰的输入辅助函数 ★★★
def get_valid_input(prompt, options, default=None):
    """一个健壮的辅助函数，用于获取用户输入，直到输入有效为止。
       options 可以是列表或字典。
    """
    if isinstance(options, dict):
        # 如果是字典，生成 'key:value' 格式的提示
        options_str = " / ".join([f"{k}:{v}" for k, v in options.items()])
        valid_keys = options.keys()
    else:
        # 如果是列表，保持原有格式
        options_str = "/".join(options)
        valid_keys = options

    prompt_full = f"{prompt} ({options_str})"
    if default:
        default_text = options.get(default, default) if isinstance(options, dict) else default
        prompt_full += f" [默认: {default_text}]"
    prompt_full += ": "

    while True:
        user_input = input(prompt_full).strip()
        if not user_input and default:
            print(f"-> 使用默认选项: {default}")
            return default
        if user_input in valid_keys:
            return user_input
        else:
            print(f"❌ 无效输入。请输入以下选项之一: {', '.join(valid_keys)}")


def interactive_menu_dual(right_controller, left_controller):

    def record_action():
        print("\n🎬 录制模式选择:")
        # ★ 使用字典让提示更清晰
        record_options = {'1': '只录右臂', '2': '只录左臂', '3': '双臂'}
        record_mode = get_valid_input("请选择", record_options, default='3')

        filename = input("请输入保存的文件名 (如: wave_hand.json): ").strip()
        if not filename:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"record_{timestamp}.json"
            print(f"-> 使用默认文件名: {filename}")
        
        # (后续逻辑不变...)
        interval = 0.05
        filepath = get_action_file_path(filename)
        print(f"准备录制... 文件将保存至: {filepath}")
        input("请将机械臂摆放到起始姿态，然后按 Enter 开始...")
        if record_mode == '1': right_controller.unload_all_servos()
        elif record_mode == '2': left_controller.unload_all_servos()
        else:
            right_controller.unload_all_servos(); left_controller.unload_all_servos()
        print("录制开始！按 Ctrl+C 结束录制。")
        recorded_data, start_time = [], time.time()
        try:
            while True:
                loop_start, current_time = time.time(), time.time() - start_time
                entry = {"time": round(current_time, 4)}
                status_line = []
                if record_mode in ['1', '3']:
                    pos = right_controller.read_all_positions()
                    if any(p is not None for p in pos.values()):
                        entry["right_arm_angles"] = {f"{s:03d}": p for s, p in pos.items() if p is not None}
                        status_line.append(f"右臂:{len(entry['right_arm_angles'])}")
                if record_mode in ['2', '3']:
                    pos = left_controller.read_all_positions()
                    if any(p is not None for p in pos.values()):
                        entry["left_arm_angles"] = {f"{s:03d}": p for s, p in pos.items() if p is not None}
                        status_line.append(f"左臂:{len(entry['left_arm_angles'])}")
                if len(entry) > 1: recorded_data.append(entry)
                print(f"\r时长:{current_time:.2f}s|录制:{len(recorded_data)}帧|{'|'.join(status_line)}", end=" ")
                time.sleep(max(0, interval - (time.time() - loop_start)))
        except KeyboardInterrupt:
            print(f"\n\n📹 录制结束。共 {len(recorded_data)} 帧。")
        finally:
            if record_mode == '1': right_controller.load_all_servos()
            elif record_mode == '2': left_controller.load_all_servos()
            else:
                right_controller.load_all_servos(); left_controller.load_all_servos()
        if recorded_data:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(recorded_data, f, ensure_ascii=False, indent=2)
            print(f"💾 动作已保存: {filepath}")

    def playback_action():
        filename = input("请输入要播放的JSON文件名 (如: dual.json): ").strip()
        if not filename:
            print("❌ 文件名不能为空"); return
        filepath = get_action_file_path(filename)
        if not os.path.exists(filepath):
            print(f"❌ 文件不存在: {filepath}"); return
        try:
            with open(filepath, 'r') as f: data = json.load(f)
            if not data: print("❌ 动作文件为空。"); return
        except Exception as e:
            print(f"❌ 读取文件失败: {e}"); return
            
        speed = float(input("请输入播放速度倍数 [默认1.0]: ").strip() or 1.0)
        
        print("\n🎥 播放模式选择:")
        # ★ 使用字典让提示更清晰
        play_options = {'1': '普通播放', '2': '平滑播放'}
        play_mode = get_valid_input("请选择", play_options, default='2')

        if play_mode == '1': playback_normal(data, speed)
        else: playback_smooth(data, speed)
            
    def playback_normal(action_data, speed):
        print("\n▶️ 开始普通播放...")
        right_controller.load_all_servos(); left_controller.load_all_servos()
        last_time = action_data[0].get("time", 0)
        try:
            for i, frame in enumerate(action_data):
                current_time = frame.get("time", 0)
                time_diff = (current_time - last_time) / speed
                if time_diff > 0: time.sleep(time_diff)
                status_line = []
                if "right_arm_angles" in frame:
                    for s, p in frame["right_arm_angles"].items(): right_controller.move_servo(int(s), p, int(time_diff*1000))
                    status_line.append("右")
                if "left_arm_angles" in frame:
                    for s, p in frame["left_arm_angles"].items(): left_controller.move_servo(int(s), p, int(time_diff*1000))
                    status_line.append("左")
                print(f"\r播放中 {i+1}/{len(action_data)}|{'&'.join(status_line)}", end="")
                last_time = current_time
        except KeyboardInterrupt: print("\n⏹️ 播放停止。")
        print("\n🎯 播放完成。")

    def playback_smooth(action_data, speed):
        print("\n▶️ 开始平滑播放...")
        CONTROL_FREQUENCY, MAX_SPEED_UNITS_PER_SEC = 50, 800
        max_delta, loop_interval = MAX_SPEED_UNITS_PER_SEC / CONTROL_FREQUENCY, 1.0 / CONTROL_FREQUENCY
        right_controller.load_all_servos(); left_controller.load_all_servos()

        current_r = {s: 500 for s in RIGHT_ARM_IDS}; current_l = {s: 500 for s in LEFT_ARM_IDS}
        if action_data[0].get("right_arm_angles"): current_r.update({int(k):v for k,v in action_data[0]["right_arm_angles"].items()})
        if action_data[0].get("left_arm_angles"): current_l.update({int(k):v for k,v in action_data[0]["left_arm_angles"].items()})
        for s, p in current_r.items(): right_controller.move_servo(s, p, 1000)
        for s, p in current_l.items(): left_controller.move_servo(s, p, 1000)
        time.sleep(1.2)

        start_time, action_idx = time.time(), 0
        try:
            while action_idx < len(action_data):
                loop_start = time.time()
                elapsed = (time.time() - start_time) * speed
                while action_idx < len(action_data)-1 and action_data[action_idx+1]["time"] < elapsed: action_idx += 1
                
                target, status = action_data[action_idx], []
                target_r = {int(k): v for k, v in target.get("right_arm_angles", {}).items()}
                target_l = {int(k): v for k, v in target.get("left_arm_angles", {}).items()}

                if target_r:
                    for s, t_pos in target_r.items():
                        delta = max(-max_delta, min(max_delta, t_pos - current_r[s]))
                        current_r[s] += delta; right_controller.move_servo(s, int(current_r[s]), 0)
                    status.append("右")
                if target_l:
                    for s, t_pos in target_l.items():
                        delta = max(-max_delta, min(max_delta, t_pos - current_l[s]))
                        current_l[s] += delta; left_controller.move_servo(s, int(current_l[s]), 0)
                    status.append("左")

                print(f"\r平滑播放 {action_idx+1}/{len(action_data)}|{'&'.join(status)}", end="")
                time.sleep(max(0, loop_interval - (time.time() - loop_start)))
                
                if action_idx == len(action_data)-1:
                    r_done = all(abs(target_r.get(s, c) - c) <= max_delta for s, c in current_r.items())
                    l_done = all(abs(target_l.get(s, c) - c) <= max_delta for s, c in current_l.items())
                    if r_done and l_done: break
        except KeyboardInterrupt: print("\n⏹️ 播放停止。")
        print("\n🎯 平滑播放完成。")

    def sync_control(master, slave, master_name, slave_name):
        print(f"\n🔄 {master_name} 控制 {slave_name} [镜像同步模式] 启动！")
        CONTROL_FREQUENCY, MAX_SPEED_UNITS_PER_SEC, MIRROR_IDS = 50, 800, {1,2,3,4,5,6}
        max_delta, loop_interval = MAX_SPEED_UNITS_PER_SEC/CONTROL_FREQUENCY, 1.0/CONTROL_FREQUENCY
        print(f"参数: {CONTROL_FREQUENCY}Hz, {MAX_SPEED_UNITS_PER_SEC}units/s, 镜像ID:{sorted(list(MIRROR_IDS))}")
        master.unload_all_servos(); slave.load_all_servos()
        print("正在初始化姿态..."); time.sleep(0.1)
        raw_master = master.read_all_positions()
        if not any(p is not None for p in raw_master.values()):
            print("\n❌ 错误: 无法读取主臂位置！"); master.load_all_servos(); return
        last_sent = {}
        for sid in slave.servo_ids:
            m_pos = raw_master.get(sid, 500)
            s_target = 1000 - m_pos if sid in MIRROR_IDS else m_pos
            last_sent[sid] = s_target; slave.move_servo(sid, s_target, 1000)
        time.sleep(1.2); print("初始化完成！请移动主臂 (Ctrl+C 退出)。")
        try:
            while True:
                loop_start = time.time()
                raw_master = master.read_all_positions()
                if not any(p is not None for p in raw_master.values()): continue
                status = []
                for sid in slave.servo_ids:
                    m_pos = raw_master.get(sid)
                    if m_pos is None: continue
                    s_target = 1000 - m_pos if sid in MIRROR_IDS else m_pos
                    last_s = last_sent.get(sid, s_target)
                    delta = max(-max_delta, min(max_delta, s_target - last_s))
                    new_s = int(last_s + delta)
                    slave.move_servo(sid, new_s, 0); last_sent[sid] = new_s
                    status.append(f"{sid}:{new_s}")
                print(f"\r同步中... {slave_name} 指令: {'|'.join(status)}   ", end="")
                time.sleep(max(0, loop_interval - (time.time() - loop_start)))
        except KeyboardInterrupt: print(f"\n⏹️ 同步控制已退出。")
        finally: master.load_all_servos()
    


    # ============================  主菜单循环  ===============================
    print("\n" + "="*50); print("🤖 双臂多舵机控制系统 (最终UI增强版)"); print("="*50)
    while True:
        print("\n请选择功能:")
        print("1. 🔓 卸力       2. 🔒 上力"); print("3. 🎬 录制动作"); print("4. 🎥 播放动作")
        print("5. 👁️ 查看位置"); print("6. 🔄 镜像同步"); print("0. 🚪 退出")
        choice = input("\n请输入选择 (0-6): ").strip()

        if choice == '0': break
        elif choice in ['1', '2']:
            action_text = "卸力" if choice == '1' else "上力"
            arm_options = {'1': '右臂', '2': '左臂', '3': '双臂'}
            arm = get_valid_input(f"选择要{action_text}的手臂", arm_options, default='3')
            
            do_right = arm in ['1', '3']
            do_left = arm in ['2', '3']
            
            if choice == '1': # 卸力
                if do_right: right_controller.unload_all_servos()
                if do_left: left_controller.unload_all_servos()
            else: # 上力
                if do_right: right_controller.load_all_servos()
                if do_left: left_controller.load_all_servos()
        elif choice == '3': record_action()
        elif choice == '4': playback_action()
        elif choice == '5':
            print("\n--- 右臂状态 ---"); print(f"{'ID' :<5}{'位置':<7}{'角度(°)' :<10}{'温度(°C)':<5}")
            pos_r, temp_r = right_controller.read_all_positions(), right_controller.read_all_temperatures()
            for sid in RIGHT_ARM_IDS:
                p, t = pos_r.get(sid), temp_r.get(sid, 'N/A')
                angle = f"{(p/1000*240):.1f}" if p is not None else "N/A"
                if not any([p is not None for p in pos_r.values()]):
                    print(f"❌ 无法读取右臂位置数据，请检查连接。")
                    break
                print(f"{sid:<5}{p if p is not None else 'N/A':<7}{angle:<10}{t:<5}")
            print("\n--- 左臂状态 ---"); print(f"{'ID' :<5}{'位置':<7}{'角度(°)' :<10}{'温度(°C)':<5}")
            pos_l, temp_l = left_controller.read_all_positions(), left_controller.read_all_temperatures()
            for sid in LEFT_ARM_IDS:
                p, t = pos_l.get(sid), temp_l.get(sid, 'N/A')
                angle = f"{(p/1000*240):.1f}" if p is not None else "N/A"
                if not any([p is not None for p in pos_l.values()]):
                    print(f"❌ 无法读取左臂位置数据，请检查连接。")
                    break
                print(f"{sid:<5}{p if p is not None else 'N/A':<7}{angle:<10}{t:<5}")
        elif choice == '6':
            sync_options = {'1': '左臂控制右臂', '2': '右臂控制左臂'}
            sync_choice = get_valid_input("选择同步模式", sync_options, default='1')
            if sync_choice == '1': sync_control(left_controller, right_controller, "左臂", "右臂")
            else: sync_control(right_controller, left_controller, "右臂", "左臂")

def main():
    print("\n双臂控制测试系统 启动")
    baudrate = 115200
    right_controller, left_controller = None, None
    try:
        right_controller = MultiServoController(RIGHT_ARM_PORT, baudrate, RIGHT_ARM_IDS, arm_name='right')
        left_controller = MultiServoController(LEFT_ARM_PORT, baudrate, LEFT_ARM_IDS, arm_name='left')
        if not right_controller.ser or not left_controller.ser:
            raise serial.SerialException("串口连接失败")
        print("\n🛠️ 正在初始化所有舵机到中间位置(500)...")
        for sid in RIGHT_ARM_IDS: right_controller.move_servo(sid, 500, 1000); time.sleep(0.02)
        for sid in LEFT_ARM_IDS: left_controller.move_servo(sid, 500, 1000); time.sleep(0.02)
        time.sleep(1.2); print("✅ 初始化完成\n")
        interactive_menu_dual(right_controller, left_controller)
    except (serial.SerialException, FileNotFoundError, KeyboardInterrupt) as e:
        if not isinstance(e, KeyboardInterrupt):
            print(f"\n❌ 程序错误: {e}")
    finally:
        print("\n程序正在退出...")
        if right_controller and right_controller.ser:
             right_controller.load_all_servos(); right_controller.close()
        if left_controller and left_controller.ser:
             left_controller.load_all_servos(); left_controller.close()
        print("程序已安全退出。")

if __name__ == "__main__":
    main()