#!/usr/bin/env python3
"""
示例脚本：展示如何从其他功能包调用robot_music的音频播放功能
"""
import rclpy
from rclpy.node import Node
from robot_music.srv import PlayAudio
from std_srvs.srv import Trigger
import time

class MusicClientExample:
    def __init__(self):
        """初始化音乐客户端示例"""
        rclpy.init()
        self.node = Node('music_client_example')
        
        # 等待服务可用
        self.wait_for_services()
        
        # 创建服务客户端
        self.play_audio_client = self.node.create_client(PlayAudio, 'play_audio')
        self.stop_audio_client = self.node.create_client(Trigger, 'stop_audio')
        self.check_playing_client = self.node.create_client(Trigger, 'check_audio_playing')
    
    def wait_for_services(self):
        """等待音频服务可用"""
        self.node.get_logger().info("Waiting for audio services...")
        
        # 等待服务可用
        while not self.play_audio_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('play_audio service not available, waiting again...')
        while not self.stop_audio_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('stop_audio service not available, waiting again...')
        while not self.check_playing_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('check_audio_playing service not available, waiting again...')
        
        self.node.get_logger().info("Audio services available")
    
    def play_welcome_music(self):
        """播放欢迎音乐"""
        self.node.get_logger().info("Playing welcome music")
        req = PlayAudio.Request()
        req.audio_file = 'welcome.mp3'
        req.use_default_dir = True
        future = self.play_audio_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().success
    
    def play_alert_sound(self):
        """播放警报声音"""
        self.node.get_logger().info("Playing alert sound")
        req = PlayAudio.Request()
        req.audio_file = 'alert.mp3'
        req.use_default_dir = True
        future = self.play_audio_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().success
    
    def stop_current_music(self):
        """停止当前播放的音乐"""
        self.node.get_logger().info("Stopping current music")
        req = Trigger.Request()
        future = self.stop_audio_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().success
    
    def check_music_status(self):
        """检查音乐播放状态"""
        req = Trigger.Request()
        future = self.check_playing_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        is_playing = "is playing" in response.message.lower()
        self.node.get_logger().info(f"Music status: {'Playing' if is_playing else 'Not playing'}")
        return is_playing
    
    def shutdown(self):
        """关闭节点"""
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    """主函数"""
    try:
        example = MusicClientExample()
        
        # 示例1: 播放欢迎音乐
        example.play_welcome_music()
        time.sleep(2)  # 播放2秒
        
        # 检查播放状态
        example.check_music_status()
        
        # 停止播放
        example.stop_current_music()
        
        # 示例2: 播放警报声音
        example.play_alert_sound()
        time.sleep(1)  # 播放1秒
        
        # 停止播放
        example.stop_current_music()
        
        example.node.get_logger().info("Music client example completed")
        
    except Exception as e:
        example.node.get_logger().error(f"Service call failed: {e}")
    finally:
        example.shutdown()

if __name__ == '__main__':
    main()
