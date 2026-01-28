#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
控制接口模块
提供前端按钮调用的四个核心功能接口：
1. 开始播放
2. 停止播放
3. 重置位姿
4. 刷新状态
"""

from flask import request
from flask_socketio import emit
import threading
import os
import json
import time


class ControlAPI:
    """控制接口类，封装四个核心功能"""
    
    def __init__(self, socketio, right_controller, left_controller, arm_data_dir):
        """
        初始化控制接口
        
        Args:
            socketio: Socket.IO实例
            right_controller: 右臂控制器
            left_controller: 左臂控制器
            arm_data_dir: 动作数据目录
        """
        self.socketio = socketio
        self.right_controller = right_controller
        self.left_controller = left_controller
        self.arm_data_dir = arm_data_dir
        
        # 全局状态变量
        self.recording_active = False
        self.playback_active = False
        self.sync_active = False
        self.playback_thread = None
        self.arms_initialized = False
        
        # 初始化角度数组
        self.init_angles = [120, 120, 120, 120, 120, 120, 80]
    
    def log_action(self, message, level='info'):
        """记录操作并发送到前端"""
        self.socketio.emit('status_update', {'msg': message, 'level': level})
    
    # ==================== 功能1: 开始播放 ====================
    
    def start_playback(self, data):
        """
        开始播放动作文件
        
        Args:
            data: 包含filename和speed的字典
                - filename: 动作文件名
                - speed: 播放速度（可选，默认1.0）
        
        Returns:
            dict: 操作结果
        """
        filename = data.get('filename')
        speed = float(data.get('speed', 1.0))
        
        if not filename:
            self.log_action('❌ 错误：未提供文件名', 'error')
            emit('playback_failed', {'error': '未提供文件名'})
            return {'success': False, 'error': '未提供文件名'}
        
        # 检查是否有其他操作正在进行
        if self.recording_active or self.playback_active or self.sync_active:
            self.log_action('❌ 无法开始播放：另一个操作正在进行中', 'error')
            emit('playback_failed', {'error': '另一个操作正在进行中，请等待当前操作完成。'})
            return {'success': False, 'error': '另一个操作正在进行中'}
        
        # 检查文件是否存在
        filepath = os.path.join(self.arm_data_dir, filename)
        if not os.path.exists(filepath):
            self.log_action(f'❌ 文件不存在: {filename}', 'error')
            emit('playback_failed', {'error': f'文件 {filename} 不存在'})
            return {'success': False, 'error': f'文件 {filename} 不存在'}
        
        # 启动播放线程
        self.playback_active = True
        self.playback_thread = threading.Thread(
            target=self._playback_worker, 
            args=(filename, speed)
        )
        self.playback_thread.start()
        
        self.log_action(f'▶️ 开始播放: {filename} (速度: {speed}x)')
        emit('playback_started', {'filename': filename, 'speed': speed})
        
        return {'success': True, 'filename': filename, 'speed': speed}
    
    def _playback_worker(self, filename, speed):
        """播放工作线程"""
        filepath = os.path.join(self.arm_data_dir, filename)
        
        # 读取文件
        try:
            with open(filepath, 'r') as f:
                action_data = json.load(f)
        except Exception as e:
            self.log_action(f'❌ 读取文件失败: {e}', 'error')
            self.playback_active = False
            self.playback_thread = None
            self.socketio.emit('playback_stopped')
            return
        
        if not action_data:
            self.log_action('❌ 动作文件为空', 'warning')
            self.playback_active = False
            self.playback_thread = None
            self.socketio.emit('playback_stopped')
            return
        
        # 上力
        if self.right_controller:
            self.right_controller.load_all_servos()
        if self.left_controller:
            self.left_controller.load_all_servos()
        
        # 控制参数
        CONTROL_FREQUENCY = 50
        MAX_SPEED_UNITS_PER_SEC = 800
        max_delta = MAX_SPEED_UNITS_PER_SEC / CONTROL_FREQUENCY
        loop_interval = 1.0 / CONTROL_FREQUENCY
        
        # 初始化当前位置
        current_r = {s: 500 for s in range(1, 8)}
        current_l = {s: 500 for s in range(1, 8)}
        
        # 初始化到第一帧
        first_frame = action_data[0]
        if "right_arm_angles" in first_frame and self.right_controller:
            initial_pos = {int(k): v for k, v in first_frame["right_arm_angles"].items()}
            current_r.update(initial_pos)
            for s, p in initial_pos.items():
                self.right_controller.move_servo(s, p, 1000)
        
        if "left_arm_angles" in first_frame and self.left_controller:
            initial_pos = {int(k): v for k, v in first_frame["left_arm_angles"].items()}
            current_l.update(initial_pos)
            for s, p in initial_pos.items():
                self.left_controller.move_servo(s, p, 1000)
        
        time.sleep(1.0)  # 等待初始化完成
        
        # 播放动作
        total_frames = len(action_data)
        action_idx = 1
        
        while self.playback_active and action_idx < total_frames:
            loop_start = time.time()
            
            frame = action_data[action_idx]
            prev_frame = action_data[action_idx - 1]
            time_diff = frame.get('time', 0) - prev_frame.get('time', 0)
            if time_diff <= 0:
                time_diff = 0.05
            
            # 计算插值步数
            steps = int(max(1, time_diff * CONTROL_FREQUENCY * speed))
            
            for step in range(steps):
                if not self.playback_active:
                    break
                
                alpha = (step + 1) / steps
                
                # 右臂插值
                if "right_arm_angles" in frame and self.right_controller:
                    for sid_str, target_pos in frame["right_arm_angles"].items():
                        sid = int(sid_str)
                        if sid in current_r:
                            start_pos = current_r[sid]
                            delta = target_pos - start_pos
                            max_move = max_delta / speed
                            
                            if abs(delta) > max_move:
                                delta = max_move if delta > 0 else -max_move
                            
                            new_pos = start_pos + delta
                            current_r[sid] = new_pos
                            self.right_controller.move_servo(sid, int(new_pos), 20)
                
                # 左臂插值
                if "left_arm_angles" in frame and self.left_controller:
                    for sid_str, target_pos in frame["left_arm_angles"].items():
                        sid = int(sid_str)
                        if sid in current_l:
                            start_pos = current_l[sid]
                            delta = target_pos - start_pos
                            max_move = max_delta / speed
                            
                            if abs(delta) > max_move:
                                delta = max_move if delta > 0 else -max_move
                            
                            new_pos = start_pos + delta
                            current_l[sid] = new_pos
                            self.left_controller.move_servo(sid, int(new_pos), 20)
                
                # 发送进度
                progress = (action_idx + alpha) / total_frames
                self.socketio.emit('playback_progress', {
                    'current': action_idx + 1,
                    'total': total_frames,
                    'progress': progress
                })
                
                # 控制循环频率
                elapsed = time.time() - loop_start
                sleep_time = max(0, loop_interval - elapsed)
                time.sleep(sleep_time)
                loop_start = time.time()
            
            action_idx += 1
        
        # 播放结束
        if self.playback_active:
            self.log_action(f'✅ 播放完成: {filename}')
        
        self.playback_active = False
        self.playback_thread = None
        self.socketio.emit('playback_stopped')
    
    # ==================== 功能2: 停止播放 ====================
    
    def stop_playback(self):
        """
        停止播放
        
        Returns:
            dict: 操作结果
        """
        if not self.playback_active:
            self.log_action('⚠️ 当前没有正在进行的播放', 'warning')
            emit('playback_stopped')
            return {'success': True, 'message': '没有正在进行的播放'}
        
        self.log_action('⏹️ 正在停止播放...')
        
        # 设置停止标志
        self.playback_active = False
        
        # 等待线程结束
        if self.playback_thread and self.playback_thread.is_alive():
            self.playback_thread.join(timeout=2.0)
        
        self.playback_thread = None
        self.playback_active = False
        
        self.log_action('✅ 播放已停止')
        emit('playback_stopped')
        
        return {'success': True, 'message': '播放已停止'}
    
    # ==================== 功能3: 重置位姿 ====================
    
    def reset_pose(self):
        """
        重置双臂到初始位姿
        
        Returns:
            dict: 操作结果
        """
        self.log_action('⚙️ 正在重置双臂到初始位姿...')
        
        # 设置右臂姿态
        if self.right_controller:
            self.log_action(f'正在设置右臂 (IDs: {self.right_controller.servo_ids}) 姿态...')
            for i, sid in enumerate(self.right_controller.servo_ids):
                if i < len(self.init_angles):
                    angle = self.init_angles[i]
                    target_pos = int((angle / 240.0) * 1000)
                    self.right_controller.move_servo(sid, target_pos, 1000)
                    time.sleep(0.02)
        
        # 设置左臂姿态
        if self.left_controller:
            self.log_action(f'正在设置左臂 (IDs: {self.left_controller.servo_ids}) 姿态...')
            for i, sid in enumerate(self.left_controller.servo_ids):
                if i < len(self.init_angles):
                    angle = self.init_angles[i]
                    target_pos = int((angle / 240.0) * 1000)
                    self.left_controller.move_servo(sid, target_pos, 1000)
                    time.sleep(0.02)
        
        # 等待舵机移动
        self.log_action('等待舵机移动...')
        time.sleep(1.2)
        
        self.arms_initialized = True
        self.log_action('✅ 双臂位姿已重置')
        emit('pose_reset_complete', {'success': True})
        
        # 刷新状态
        self.refresh_status()
        
        return {'success': True, 'message': '双臂位姿已重置'}
    
    # ==================== 功能4: 刷新状态 ====================
    
    def refresh_status(self):
        """
        刷新机械臂状态
        
        Returns:
            dict: 包含左右臂状态的字典
        """
        self.log_action('请求刷新状态...')
        
        status = {'right': {}, 'left': {}}
        
        # 读取右臂状态
        if self.right_controller:
            status['right'] = self.right_controller.read_all_statuses()
        
        # 读取左臂状态
        if self.left_controller:
            status['left'] = self.left_controller.read_all_statuses()
        
        # 发送状态到前端
        self.socketio.emit('arm_status', status)
        self.log_action('状态已发送')
        
        return status


# ==================== Socket.IO 事件注册 ====================

def register_control_events(socketio, control_api):
    """
    注册控制接口的Socket.IO事件
    
    Args:
        socketio: Socket.IO实例
        control_api: ControlAPI实例
    """
    
    @socketio.on('start_playback')
    def handle_start_playback(data):
        """处理开始播放事件"""
        return control_api.start_playback(data)
    
    @socketio.on('stop_playback')
    def handle_stop_playback():
        """处理停止播放事件"""
        return control_api.stop_playback()
    
    @socketio.on('reset_pose')
    def handle_reset_pose():
        """处理重置位姿事件"""
        return control_api.reset_pose()
    
    @socketio.on('get_status')
    def handle_get_status():
        """处理刷新状态事件"""
        return control_api.refresh_status()
