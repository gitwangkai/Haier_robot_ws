#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 必须在所有其他导入之前进行 eventlet monkey patch
import eventlet
eventlet.monkey_patch()

"""
Web版双臂多舵机控制系统
基于 Flask 和 Socket.IO
"""

import os
import time
import json
import serial
from datetime import datetime
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import threading
from filter_tool import combined_ema_deadzone_filter

# --- 全局配置 ---
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret_key_for_robot_arm_!@#'
socketio = SocketIO(app, async_mode='eventlet', cors_allowed_origins='*')

# --- 硬件和路径常量 ---
RIGHT_ARM_PORT = "/dev/arm_right"
LEFT_ARM_PORT = "/dev/ttyUSB10"
BAUDRATE = 115200
SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]
ARM_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_data")

# --- 全局变量 ---
right_controller = None
left_controller = None
recording_thread = None
playback_thread = None
sync_thread = None 
recording_active = False
playback_active = False
sync_active = False 
arms_initialized = False # 新增：确保初始化只执行一次
init_thread = None
init_lock = threading.Lock()

# --- 辅助函数 ---
def log_action(message, level='info'):
    """辅助函数，用于记录操作并发送到前端"""
    socketio.emit('status_update', {'msg': message, 'level': level})

# --- 舵机控制核心类 (从 bofangluzhiyouhua.py 迁移) ---
class MultiServoController:
    def __init__(self, port, baudrate, servo_ids, arm_name='unknown', connect_serial=False):
        self.port = port
        self.baudrate = baudrate
        self.servo_ids = servo_ids or []
        self.arm_name = arm_name
        self.ser = None
        log_action(f'ⓘ {arm_name} 臂进入数据处理模式（每次调用时打开串口）')

    def checksum(self, data):
        return (~sum(data[2:])) & 0xFF

    def send_command(self, servo_id, cmd, data=None):
        if data is None: data = []
        packet = [0x55, 0x55, servo_id, 3 + len(data), cmd]
        packet.extend(data)
        packet.append(self.checksum(packet))
        try:
            with serial.Serial(self.port, self.baudrate, timeout=0.1) as ser:
                ser.write(bytes(packet))
                return True
        except Exception:
            return False

    def read_response(self):
        # 此方法已不再使用，因为读取在 send_read_command 中完成
        return None

    def send_read_command(self, servo_id, cmd):
        packet = [0x55, 0x55, servo_id, 3, cmd]
        packet.append(self.checksum(packet))
        try:
            with serial.Serial(self.port, self.baudrate, timeout=0.1) as ser:
                ser.flushInput()
                ser.write(bytes(packet))
                time.sleep(0.005)
                if ser.in_waiting > 0:
                    return list(ser.read(ser.in_waiting))
        except Exception:
            pass
        return None

    def read_position(self, servo_id):
        response = self.send_read_command(servo_id, 28) # CMD_READ_POS
        if response and len(response) >= 7 and response[0:2] == [0x55, 0x55] and response[4] == 28:
            return response[5] + (response[6] << 8)
        return None

    def read_voltage(self, servo_id):
        response = self.send_read_command(servo_id, 27) # CMD_READ_VIN
        if response and len(response) >= 7 and response[0:2] == [0x55, 0x55] and response[4] == 27:
            return (response[5] + (response[6] << 8)) / 1000.0
        return None

    def read_temperature(self, servo_id):
        response = self.send_read_command(servo_id, 26) # CMD_READ_TEMP
        if response and len(response) >= 6 and response[0:2] == [0x55, 0x55] and response[4] == 26:
            return response[5]
        return None

    def read_load(self, servo_id):
        """读取舵机负载。返回值为 0-1000，数值越大负载越大。"""
        response = self.send_read_command(servo_id, 29) # CMD_READ_LOAD (通常是29)
        if response and len(response) >= 7 and response[0:2] == [0x55, 0x55] and response[4] == 29:
            return response[5] + (response[6] << 8)
        return None

    def read_all_statuses(self):
        statuses = {}
        for sid in self.servo_ids:
            pos = self.read_position(sid)
            # 为了加快速度，只有在舵机在线时才读取其他信息
            if pos is not None:
                statuses[sid] = {
                    'pos': pos,
                    'volt': self.read_voltage(sid),
                    'temp': self.read_temperature(sid),
                    'load': self.read_load(sid) # 新增：读取负载
                }
            else:
                statuses[sid] = {'pos': None, 'volt': None, 'temp': None, 'load': None}
        return statuses

    def read_all_positions(self):
        return {sid: self.read_position(sid) for sid in self.servo_ids}

    def move_servo(self, servo_id, position, duration=100):
        position = max(0, min(1000, int(position)))
        data = [position & 0xFF, (position >> 8) & 0xFF, duration & 0xFF, (duration >> 8) & 0xFF]
        return self.send_command(servo_id, 1, data) # CMD_MOVE

    def set_unload(self, servo_id, unload):
        # 修正：0为卸力，1为上力
        return self.send_command(servo_id, 31, [0 if unload else 1])

    def unload_all_servos(self):
        log_action(f'🔓 正在卸载 {self.arm_name} 臂所有舵机...')
        for sid in self.servo_ids:
            self.set_unload(sid, True)
            time.sleep(0.01)
        log_action(f'🎯 {self.arm_name} 臂所有舵机已卸力。')

    def load_all_servos(self):
        log_action(f'🔒 正在装载 {self.arm_name} 臂所有舵机...')
        for sid in self.servo_ids:
            self.set_unload(sid, False)
            time.sleep(0.01)
        log_action(f'🎯 {self.arm_name} 臂所有舵机已上力。')

    def close(self):
        # 串口每次调用后自动关闭，无需手动关闭
        socketio.emit('status_update', {'msg': f'🔌 串口 {self.port} 使用完毕，已释放'})

# --- Flask 路由 ---
@app.route('/')
def index():
    return render_template('index.html')

def initialize_arms_to_default_pose(force=False):
    """将双臂移动到预设的初始姿态"""
    global arms_initialized
    if arms_initialized and not force:
        log_action('姿态已经初始化，跳过。')
        return

    log_action('⚙️ 正在初始化双臂到预设姿态...')
    
    # 初始化角度数组 [120, 120, 120, 210, 120, 120, 80] 对应舵机1-7
    init_angles = [120, 120, 120, 120, 120, 120, 80]
    
    def set_pose(controller, arm_name):
        if not controller:
            log_action(f'⚠️ {arm_name} 控制器未创建，跳过姿态初始化。', 'warning')
            return
        
        log_action(f'正在设置 {arm_name} (IDs: {controller.servo_ids}) 姿态...')
        for i, sid in enumerate(controller.servo_ids):
            if i < len(init_angles):
                angle = init_angles[i]
                # 将角度转换为位置: position = (angle / 240.0) * 1000
                target_pos = int((angle / 240.0) * 1000)
                log_action(f'设置舵机 {sid} 到角度 {angle}° (位置 {target_pos})')
            else:
                target_pos = 500  # 默认中间位置
            controller.move_servo(sid, target_pos, 1000)
            time.sleep(0.02) # 发送指令间短暂停顿
    
    # 确保控制器已创建
    if right_controller and left_controller:
        set_pose(right_controller, "右臂")
        set_pose(left_controller, "左臂")
        
        # 等待舵机移动到目标位置
        log_action('等待舵机移动...')
        time.sleep(1.2)
        log_action('✅ 双臂姿态初始化完成。')
        arms_initialized = True
        # 更新一次状态
        socketio.emit('force_refresh') # 通知前端刷新状态和文件
    else:
        log_action('控制器尚未完全初始化，无法设置姿态。', 'error')


# --- Background Initializer ---
def do_initialization():
    """在后台线程中执行耗时的硬件初始化。"""
    global right_controller, left_controller
    
    with init_lock:
        if right_controller is None: # 检查确保只初始化一次
            log_action('首次连接，正在初始化机械臂控制器...')
            right_controller = MultiServoController(RIGHT_ARM_PORT, BAUDRATE, SERVO_IDS, 'right')
            left_controller = MultiServoController(LEFT_ARM_PORT, BAUDRATE, SERVO_IDS, 'left')
            
            # 尝试连接右臂
            right_connected = False
            try:
                with serial.Serial(RIGHT_ARM_PORT, BAUDRATE, timeout=0.1) as ser:
                    log_action(f'✓ 右臂串口连接成功: {RIGHT_ARM_PORT}')
                    right_connected = True
            except Exception as e:
                log_action(f'✗ 右臂串口连接失败: {e}', 'error')
            
            # 尝试连接左臂
            left_connected = False
            try:
                with serial.Serial(LEFT_ARM_PORT, BAUDRATE, timeout=0.1) as ser:
                    log_action(f'✓ 左臂串口连接成功: {LEFT_ARM_PORT}')
                    left_connected = True
            except Exception as e:
                log_action(f'✗ 左臂串口连接失败: {e}', 'error')
            
            if right_connected or left_connected:
                initialize_arms_to_default_pose()
                log_action('✅ 硬件初始化完成。')
                socketio.emit('init_complete', {'success': True})
            else:
                log_action('❌ 两个机械臂都无法连接，初始化失败。', 'error')
                socketio.emit('init_complete', {'success': False, 'message': '无法连接到任何一个串口。'})
        else:
            log_action('控制器实例已存在，跳过初始化。')
            socketio.emit('init_complete', {'success': True})


# --- Socket.IO 事件处理 ---
@socketio.on('connect')
def handle_connect():
    """客户端连接时调用的函数。"""
    global init_thread
    if not os.path.exists(ARM_DATA_DIR):
        os.makedirs(ARM_DATA_DIR)
    
    log_action(f'客户端 {request.sid} 已连接。')
    emit('connection_established')

    # 使用锁和全局线程变量确保初始化只在后台为第一个连接的客户端运行一次
    with init_lock:
        if init_thread is None:
            init_thread = socketio.start_background_task(target=do_initialization)

@socketio.on('reset_pose')
def handle_reset_pose():
    """处理重置位姿的请求"""
    log_action('⚙️ 正在重置双臂到初始位姿...')
    # 调用姿态初始化函数，并强制执行
    initialize_arms_to_default_pose(force=True)
    log_action('✅ 双臂位姿已重置。')

@socketio.on('disconnect')
def handle_disconnect():
    # 注意：断开连接事件中没有请求上下文，所以不能用 request.sid
    log_action('一个客户端已断开连接。')

@socketio.on('get_status')
def handle_get_status():
    log_action('请求刷新状态...')
    status = {'right': {}, 'left': {}}
    if right_controller:
        status['right'] = right_controller.read_all_statuses()
    if left_controller:
        status['left'] = left_controller.read_all_statuses()
    socketio.emit('arm_status', status)
    log_action('状态已发送。')

@socketio.on('manage_power')
def handle_manage_power(data):
    target = data.get('target')
    action = data.get('action') # 'load' or 'unload'
    log_action(f'请求对 {target} 执行 {action} 操作...')

    is_unload = (action == 'unload')

    controllers = []
    if target == 'left_all' or target == 'both_all':
        controllers.append(left_controller)
    if target == 'right_all' or target == 'both_all':
        controllers.append(right_controller)

    if controllers:
        for controller in controllers:
            if is_unload:
                controller.unload_all_servos()
            else:
                controller.load_all_servos()
    elif '_' in target:
        try:
            arm, sid_str = target.split('_')
            sid = int(sid_str)
            controller = left_controller if arm == 'left' else right_controller
            if controller:
                log_action(f"正在对 {controller.arm_name} 臂的舵机 {sid} 执行 {'卸力' if is_unload else '上力'}...")
                controller.set_unload(sid, is_unload)
        except ValueError:
            log_action(f"错误：无效的目标格式 '{target}'", 'error')
            return
    
    time.sleep(0.1) # 等待操作完成
    handle_get_status()

@socketio.on('get_action_files')
def handle_get_action_files():
    log_action('请求刷新动作文件列表...')
    files = []
    if os.path.exists(ARM_DATA_DIR):
        files = [f for f in os.listdir(ARM_DATA_DIR) if f.endswith('.json')]
    emit('action_files_list', {'files': sorted(files, reverse=True)})
    log_action('文件列表已发送。')

def record_worker(filename, arms_to_record):
    global recording_active
    filepath = os.path.join(ARM_DATA_DIR, filename)
    log_action(f'录制开始！文件: {filename}。按“停止录制”结束。')
    
    recorded_data = []
    start_time = time.time()
    interval = 0.05 # 20Hz

    while recording_active:
        loop_start = time.time()
        current_time = time.time() - start_time
        entry = {"time": round(current_time, 4)}
        
        if 'right' in arms_to_record:
            pos = right_controller.read_all_positions()
            if any(p is not None for p in pos.values()):
                entry["right_arm_angles"] = {f"{s:03d}": p for s, p in pos.items() if p is not None}
        
        if 'left' in arms_to_record:
            pos = left_controller.read_all_positions()
            if any(p is not None for p in pos.values()):
                entry["left_arm_angles"] = {f"{s:03d}": p for s, p in pos.items() if p is not None}

        if len(entry) > 1:
            recorded_data.append(entry)
        
        socketio.emit('recording_progress', {'time': f'{current_time:.2f}s', 'frames': len(recorded_data)})
        time.sleep(max(0, interval - (time.time() - loop_start)))

    if recorded_data:
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(recorded_data, f, ensure_ascii=False, indent=2)
        log_action(f'💾 录制结束，动作已保存: {filename}')
        # 修复：直接在后台线程中更新文件列表，而不是调用 handle_get_action_files
        try:
            files = [f for f in os.listdir(ARM_DATA_DIR) if f.endswith('.json')]
            socketio.emit('action_files_updated', {'files': files})
            log_action("动作文件列表已刷新。")
        except Exception as e:
            log_action(f"错误：刷新动作文件列表失败: {e}")
    else:
        log_action('录制结束，但未捕获到任何数据。', 'warning')

@socketio.on('start_recording')
def handle_start_recording(data):
    global recording_thread, recording_active
    if recording_active or playback_active or sync_active:
        log_action('无法开始录制：另一个操作正在进行中。', 'error')
        emit('recording_failed', {'error': '另一个操作正在进行中。'})
        return


    filename = data.get('filename')
    arms_to_record = data.get('arms', [])
    log_action(f'请求开始录制。手臂: {arms_to_record}, 文件名: {filename or "自动生成"}')
    if not filename:
        filename = f"record_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    
    filepath = os.path.join(ARM_DATA_DIR, filename)
    if os.path.exists(filepath):
        log_action(f'错误：文件 {filename} 已存在，无法覆盖。', 'error')
        emit('recording_failed', {'error': f'文件 {filename} 已存在。'})
        return

    if 'right' in arms_to_record and right_controller: right_controller.unload_all_servos()
    if 'left' in arms_to_record and left_controller: left_controller.unload_all_servos()

    recording_active = True
    recording_thread = threading.Thread(target=record_worker, args=(filename, arms_to_record))
    recording_thread.start()
    emit('recording_started', {'filename': filename})

@socketio.on('stop_recording')
def handle_stop_recording():
    global recording_thread, recording_active
    log_action('请求停止录制...')
    if not (recording_thread and recording_thread.is_alive()):
        log_action('当前没有正在进行的录制。', 'warning')
        return

    recording_active = False
    recording_thread.join()
    recording_thread = None
    
    if right_controller: right_controller.load_all_servos()
    if left_controller: left_controller.load_all_servos()
    
    emit('recording_stopped')


@socketio.on('delete_action_file')
def handle_delete_action_file(data):
    """处理删除动作文件的请求"""
    filename = data.get('filename')
    if not filename:
        log_action('错误：删除请求未提供文件名。', 'error')
        return

    filepath = os.path.join(ARM_DATA_DIR, filename)
    if os.path.exists(filepath):
        try:
            os.remove(filepath)
            log_action(f'🗑️ 文件已删除: {filename}')
            # 刷新所有客户端的文件列表
            handle_get_action_files()
        except Exception as e:
            log_action(f'❌ 删除文件失败: {e}', 'error')
    else:
        log_action(f'错误：要删除的文件 {filename} 不存在。', 'error')


@socketio.on('rename_action_file')
def handle_rename_action_file(data):
    """处理重命名动作文件的请求"""
    old_filename = data.get('old_filename')
    new_filename = data.get('new_filename')

    if not old_filename or not new_filename:
        log_action('错误：重命名请求缺少新旧文件名。', 'error')
        return

    if not new_filename.endswith('.json'):
        new_filename += '.json'

    old_filepath = os.path.join(ARM_DATA_DIR, old_filename)
    new_filepath = os.path.join(ARM_DATA_DIR, new_filename)

    if not os.path.exists(old_filepath):
        log_action(f'错误：源文件 {old_filename} 不存在。', 'error')
        return

    if os.path.exists(new_filepath):
        log_action(f'错误：目标文件 {new_filename} 已存在。', 'error')
        emit('rename_failed', {'error': f'文件 {new_filename} 已存在。'})
        return

    try:
        os.rename(old_filepath, new_filepath)
        log_action(f'✏️ 文件已重命名: {old_filename} -> {new_filename}')
        # 刷新所有客户端的文件列表
        handle_get_action_files()
    except Exception as e:
        log_action(f'❌ 重命名文件失败: {e}', 'error')


def playback_worker(filename, speed):
    global playback_active, playback_thread
    filepath = os.path.join(ARM_DATA_DIR, filename)
    if not os.path.exists(filepath):
        log_action(f'❌ 文件不存在: {filename}', 'error')
        playback_active = False
        playback_thread = None
        socketio.emit('playback_stopped')
        return

    try:
        with open(filepath, 'r') as f:
            action_data = json.load(f)
    except Exception as e:
        log_action(f'❌ 读取文件失败: {e}', 'error')
        playback_active = False
        playback_thread = None
        socketio.emit('playback_stopped')
        return

    if not action_data:
        log_action('❌ 动作文件为空。', 'warning')
        playback_active = False
        playback_thread = None
        socketio.emit('playback_stopped')
        return

    log_action(f'▶️ 开始平滑播放: {filename}')
    if right_controller: right_controller.load_all_servos()
    if left_controller: left_controller.load_all_servos()

    CONTROL_FREQUENCY, MAX_SPEED_UNITS_PER_SEC = 50, 800
    max_delta, loop_interval = MAX_SPEED_UNITS_PER_SEC / CONTROL_FREQUENCY, 1.0 / CONTROL_FREQUENCY

    current_r = {s: 500 for s in SERVO_IDS}
    current_l = {s: 500 for s in SERVO_IDS}
    
    # 初始化到第一帧
    first_frame = action_data[0]
    if "right_arm_angles" in first_frame and right_controller:
        initial_pos = {int(k): v for k, v in first_frame["right_arm_angles"].items()}
        current_r.update(initial_pos)
        for s, p in initial_pos.items(): right_controller.move_servo(s, p, 1000)
    if "left_arm_angles" in first_frame and left_controller:
        initial_pos = {int(k): v for k, v in first_frame["left_arm_angles"].items()}
        current_l.update(initial_pos)
        for s, p in initial_pos.items(): left_controller.move_servo(s, p, 1000)
    
    time.sleep(1.2)

    start_time, action_idx = time.time(), 0
    total_frames = len(action_data)

    while playback_active and action_idx < total_frames:
        loop_start = time.time()
        elapsed = (time.time() - start_time) * speed
        
        while action_idx < total_frames - 1 and action_data[action_idx + 1]["time"] < elapsed:
            action_idx += 1
        
        target = action_data[action_idx]
        target_r = {int(k): v for k, v in target.get("right_arm_angles", {}).items()}
        target_l = {int(k): v for k, v in target.get("left_arm_angles", {}).items()}

        if target_r and right_controller:
            for s, t_pos in target_r.items():
                delta = max(-max_delta, min(max_delta, t_pos - current_r.get(s, 500)))
                current_r[s] += delta
                right_controller.move_servo(s, int(current_r[s]), 0)
        
        if target_l and left_controller:
            for s, t_pos in target_l.items():
                delta = max(-max_delta, min(max_delta, t_pos - current_l.get(s, 500)))
                current_l[s] += delta
                left_controller.move_servo(s, int(current_l[s]), 0)

        socketio.emit('playback_progress', {'current': action_idx + 1, 'total': total_frames})
        
        time.sleep(max(0, loop_interval - (time.time() - loop_start)))
        
        # 检查是否到达最后一帧并接近目标位置
        if action_idx == total_frames - 1:
            r_done = all(abs(target_r.get(s, c) - c) <= max_delta for s, c in current_r.items())
            l_done = all(abs(target_l.get(s, c) - c) <= max_delta for s, c in current_l.items())
            if r_done and l_done:
                break
    
    if playback_active: # 正常结束
        log_action('🎯 播放完成。')
    else: # 被中断
        log_action('⏹️ 播放已由用户停止。')
        
    # 清理播放状态
    playback_active = False
    playback_thread = None
    socketio.emit('playback_stopped')


@socketio.on('start_playback')
def handle_start_playback(data):
    global playback_thread, playback_active
    
    # 清理可能存在的僵尸线程
    if playback_thread and not playback_thread.is_alive():
        playback_active = False
        playback_thread = None
    
    if recording_active or playback_active or sync_active:
        log_action('无法开始播放：另一个操作正在进行中。', 'error')
        emit('playback_failed', {'error': '另一个操作正在进行中，请等待当前操作完成。'})
        return

    filename = data.get('filename')
    speed = float(data.get('speed', 1.0))
    log_action(f'请求开始播放文件: {filename} (速度: {speed}x)')
    if not filename:
        log_action('请选择一个文件进行播放。', 'error')
        return

    playback_active = True
    playback_thread = threading.Thread(target=playback_worker, args=(filename, speed))
    playback_thread.start()
    emit('playback_started', {'filename': filename})

@socketio.on('stop_playback')
def handle_stop_playback():
    global playback_thread, playback_active
    log_action('请求停止播放...')
    
    if not playback_active:
        log_action('当前没有正在进行的播放。', 'warning')
        emit('playback_stopped')
        return
    
    # 保存当前线程引用，避免竞态条件
    current_thread = playback_thread
    
    if current_thread and current_thread.is_alive():
        playback_active = False
        try:
            current_thread.join(timeout=2.0)  # 等待最多2秒
            if current_thread.is_alive():
                log_action('播放线程未能及时停止，可能需要强制终止。', 'warning')
        except Exception as e:
            log_action(f'停止播放时发生错误: {e}', 'error')
    
    # 清理状态
    playback_active = False
    playback_thread = None
    emit('playback_stopped')

@socketio.on('filter_file')
def handle_filter_file(data):
    filename = data.get('filename')
    smoothing_factor = float(data.get('smoothingFactor', 0.3))
    deadzone_threshold = int(data.get('deadzoneThreshold', 2))
    log_action(f'请求对文件 {filename} 进行滤波 (平滑: {smoothing_factor}, 死区: {deadzone_threshold})')
    
    if not filename:
        log_action('请选择要滤波的文件。', 'error')
        return

    filepath = os.path.join(ARM_DATA_DIR, filename)
    base, ext = os.path.splitext(filepath)
    filtered_filepath = f"{base}_filtered{ext}"
    
    log_action(f'🔬 正在对 {filename} 进行滤波...')
    try:
        filtered_data = combined_ema_deadzone_filter(filepath, smoothing_factor, deadzone_threshold)
        with open(filtered_filepath, 'w', encoding='utf-8') as f:
            json.dump(filtered_data, f, ensure_ascii=False, indent=2)
        log_action(f'✅ 滤波完成! 平滑数据已保存到: {os.path.basename(filtered_filepath)}')
        handle_get_action_files() # 刷新文件列表
    except Exception as e:
        log_action(f'❌ 滤波失败: {e}', 'error')

def sync_worker(master, slave, master_name, slave_name):
    global sync_active
    log_action(f"🔄 {master_name} ({master.port}) 控制 {slave_name} ({slave.port}) [镜像同步模式] 启动！")
    
    MIRROR_IDS = {1, 2, 3, 4, 5, 6}
    CONTROL_FREQUENCY = 50
    MAX_SPEED_UNITS_PER_SEC = 800
    max_delta = MAX_SPEED_UNITS_PER_SEC / CONTROL_FREQUENCY
    loop_interval = 1.0 / CONTROL_FREQUENCY

    master.unload_all_servos()
    slave.load_all_servos()
    
    log_action("正在初始化同步姿态...")
    time.sleep(0.2)
    
    raw_master = master.read_all_positions()
    if not any(p is not None for p in raw_master.values()):
        log_action(f"❌ 错误: 无法读取 {master_name} 位置！", 'error')
        master.load_all_servos()
        socketio.emit('sync_stopped')
        return

    last_sent = {}
    for sid in slave.servo_ids:
        m_pos = raw_master.get(sid, 500)
        s_target = 1000 - m_pos if sid in MIRROR_IDS else m_pos
        last_sent[sid] = s_target
        slave.move_servo(sid, s_target, 1000)
    
    time.sleep(1.2)
    log_action("初始化完成！请移动主臂。")

    while sync_active:
        loop_start = time.time()
        raw_master = master.read_all_positions()
        if not any(p is not None for p in raw_master.values()):
            continue
            
        for sid in slave.servo_ids:
            m_pos = raw_master.get(sid)
            if m_pos is None: continue
            
            s_target = 1000 - m_pos if sid in MIRROR_IDS else m_pos
            last_s = last_sent.get(sid, s_target)
            delta = max(-max_delta, min(max_delta, s_target - last_s))
            new_s = int(last_s + delta)
            slave.move_servo(sid, new_s, 0)
            last_sent[sid] = new_s
        
        # 减少状态更新频率，避免刷屏
        # socketio.emit('sync_update', {'master': raw_master, 'slave': last_sent})
        time.sleep(max(0, loop_interval - (time.time() - loop_start)))

    master.load_all_servos()
    log_action("⏹️ 同步控制已退出。")
    socketio.emit('sync_stopped')

@socketio.on('start_sync')
def handle_start_sync(data):
    global sync_thread, sync_active
    if recording_active or playback_active or sync_active:
        log_action('无法启动同步：另一个操作正在进行中。', 'error')
        emit('sync_stopped') # Use existing event to reset UI
        return

    mode = data.get('mode', 'left_to_right')
    log_action(f'请求开始同步，模式: {mode}')
    if mode == 'left_to_right':
        master, slave = left_controller, right_controller
        master_name, slave_name = "左臂", "右臂"
    else:
        master, slave = right_controller, left_controller
        master_name, slave_name = "右臂", "左臂"

    if not (master and master.ser and slave and slave.ser):
        log_action('无法启动同步：至少一个机械臂未连接。', 'error')
        return

    sync_active = True
    sync_thread = threading.Thread(target=sync_worker, args=(master, slave, master_name, slave_name))
    sync_thread.start()
    emit('sync_started')

@socketio.on('stop_sync')
def handle_stop_sync():
    global sync_thread, sync_active
    log_action('请求停止同步...')
    if not (sync_thread and sync_thread.is_alive()):
        log_action('当前没有正在进行的同步。', 'warning')
        return
    
    sync_active = False
    sync_thread.join()
    sync_thread = None
    # sync_worker 内部会发送 sync_stopped

@socketio.on('set_servo_position')
def handle_set_servo_position(data):
    """处理从前端滑块设置单个舵机位置的请求"""
    arm = data.get('arm')
    servo_id = data.get('servo_id')
    position = data.get('position')

    if not all([arm, servo_id, position is not None]):
        log_action('错误：设置舵机位置的参数不完整。', 'error')
        return

    controller = left_controller if arm == 'left' else right_controller
    
    if controller:
        log_action(f"正在设置 {controller.arm_name} 臂舵机 #{servo_id} 到位置 {position}")
        controller.move_servo(servo_id, position, duration=1000) # 设置一个平滑的移动时间
        # 移动完成后更新状态
        socketio.sleep(1.05) # 等待舵机移动完成 (略长于duration)
        handle_get_status()
    else:
        log_action(f'错误：{arm} 臂控制器未创建，无法设置舵机。', 'error')


# --- 主程序入口 ---
if __name__ == '__main__':
    print("🤖 启动 Web 控制服务器...")
    print("请在浏览器中打开 http://127.0.0.1:8082")
    socketio.run(app, host='0.0.0.0', port=8082, debug=False)
