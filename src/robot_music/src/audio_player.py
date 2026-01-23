#!/usr/bin/env python3
import pygame
import os
import rclpy
from rclpy.node import Node
import time

class AudioPlayer:
    def __init__(self):
        """初始化音频播放器"""
        try:
            pygame.mixer.init()
            self.initialized = True
            self.use_mock = False
            print("Audio player initialized successfully")
        except Exception as e:
            print(f"Failed to initialize audio player: {e}")
            print("Using mock audio player instead")
            self.initialized = True
            self.use_mock = True
            self.playing = False
            self.current_audio = None
    
    def play_audio(self, audio_path):
        """
        播放指定路径的音频文件
        
        Args:
            audio_path (str): 音频文件路径
            
        Returns:
            bool: 播放是否成功
        """
        if not os.path.exists(audio_path):
            print(f"Audio file not found: {audio_path}")
            return False
        
        if self.use_mock:
            # 使用模拟音频播放器
            print(f"[Mock] Playing audio: {audio_path}")
            self.playing = True
            self.current_audio = audio_path
            # 模拟播放3秒
            # 移除 time.sleep 调用，避免阻塞服务回调线程
            return True
        else:
            # 使用实际音频播放器
            try:
                pygame.mixer.music.load(audio_path)
                pygame.mixer.music.play()
                print(f"Playing audio: {audio_path}")
                return True
            except Exception as e:
                print(f"Failed to play audio: {e}")
                return False
    
    def stop_audio(self):
        """
        停止当前播放的音频
        
        Returns:
            bool: 停止是否成功
        """
        if self.use_mock:
            # 使用模拟音频播放器
            print("[Mock] Audio stopped")
            self.playing = False
            self.current_audio = None
            return True
        else:
            # 使用实际音频播放器
            try:
                pygame.mixer.music.stop()
                print("Audio stopped")
                return True
            except Exception as e:
                print(f"Failed to stop audio: {e}")
                return False
    
    def is_playing(self):
        """
        检查是否正在播放音频
        
        Returns:
            bool: 是否正在播放
        """
        if self.use_mock:
            # 使用模拟音频播放器
            return self.playing
        else:
            # 使用实际音频播放器
            try:
                return pygame.mixer.music.get_busy()
            except Exception as e:
                print(f"Failed to check if playing: {e}")
                return False
    
    def play_audio_from_directory(self, directory, filename):
        """
        从指定目录播放音频文件
        
        Args:
            directory (str): 音频文件目录
            filename (str): 音频文件名
            
        Returns:
            bool: 播放是否成功
        """
        audio_path = os.path.join(directory, filename)
        return self.play_audio(audio_path)
    
    def get_initialization_status(self):
        """
        获取音频播放器初始化状态
        
        Returns:
            bool: 初始化是否成功
        """
        return self.initialized
    
    def is_using_mock(self):
        """
        检查是否使用模拟音频播放器
        
        Returns:
            bool: 是否使用模拟音频播放器
        """
        return self.use_mock
