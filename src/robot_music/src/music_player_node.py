#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from robot_music.srv import PlayAudio
import os
import sys

# 添加当前文件所在目录到Python路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from audio_player import AudioPlayer

class MusicPlayerNode(Node):
    def __init__(self):
        """初始化音乐播放器节点"""
        super().__init__('music_player_node')
        
        # 初始化音频播放器
        self.audio_player = AudioPlayer()
        
        # 检查音频播放器初始化状态
        if self.audio_player.is_using_mock():
            self.get_logger().info("Using mock audio player (no audio device detected)")
        elif self.audio_player.get_initialization_status():
            self.get_logger().info("Audio player initialized successfully")
        else:
            self.get_logger().warn("Audio player initialization failed. Audio playback will not be available.")
        
        # 音频文件默认目录
        self.default_audio_dir = self.declare_parameter('audio_dir', '/home/aidlux/Haier_robot_ws/src/config/music').value
        os.makedirs(self.default_audio_dir, exist_ok=True)
        self.get_logger().info(f"Default audio directory: {self.default_audio_dir}")
        
        # 创建ROS服务
        self.play_audio_service = self.create_service(PlayAudio, 'play_audio', self.handle_play_audio)
        self.stop_audio_service = self.create_service(Trigger, 'stop_audio', self.handle_stop_audio)
        self.check_playing_service = self.create_service(Trigger, 'check_audio_playing', self.handle_check_playing)
        
        # 创建ROS话题订阅
        self.audio_topic_subscriber = self.create_subscription(String, 'play_audio_topic', self.handle_play_audio_topic, 10)
        
        self.get_logger().info("Music player node initialized and ready")
    
    def handle_play_audio(self, req, resp):
        """
        处理播放音频服务请求
        
        Args:
            req: PlayAudio服务请求
            resp: PlayAudio服务响应
            
        Returns:
            PlayAudioResponse: 服务响应
        """
        if req.use_default_dir:
            audio_path = os.path.join(self.default_audio_dir, req.audio_file)
        else:
            audio_path = req.audio_file
        
        success = self.audio_player.play_audio(audio_path)
        resp.success = success
        resp.message = "Audio played successfully" if success else "Failed to play audio"
        
        return resp
    
    def handle_stop_audio(self, req, resp):
        """
        处理停止音频服务请求
        
        Args:
            req: Trigger服务请求
            resp: Trigger服务响应
            
        Returns:
            TriggerResponse: 服务响应
        """
        success = self.audio_player.stop_audio()
        resp.success = success
        resp.message = "Audio stopped successfully" if success else "Failed to stop audio"
        
        return resp
    
    def handle_check_playing(self, req, resp):
        """
        处理检查音频播放状态服务请求
        
        Args:
            req: Trigger服务请求
            resp: Trigger服务响应
            
        Returns:
            TriggerResponse: 服务响应
        """
        is_playing = self.audio_player.is_playing()
        resp.success = True
        resp.message = "Audio is playing" if is_playing else "Audio is not playing"
        
        return resp
    
    def handle_play_audio_topic(self, msg):
        """
        处理播放音频话题消息
        
        Args:
            msg: String消息，包含音频文件名
        """
        audio_file = msg.data
        audio_path = os.path.join(self.default_audio_dir, audio_file)
        self.audio_player.play_audio(audio_path)
    
    def run(self):
        """
        运行节点
        """
        rclpy.spin(self)

if __name__ == '__main__':
    try:
        rclpy.init()
        node = MusicPlayerNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
