#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_music.srv import PlayAudio
from std_srvs.srv import Trigger
import os
import time

def test_audio_player():
    """测试音频播放器功能"""
    rclpy.init()
    node = Node('test_audio_player')
    
    # 等待服务可用
    node.get_logger().info("Waiting for audio services...")
    
    # 创建服务客户端
    play_audio_client = node.create_client(PlayAudio, 'play_audio')
    stop_audio_client = node.create_client(Trigger, 'stop_audio')
    check_playing_client = node.create_client(Trigger, 'check_audio_playing')
    
    # 等待服务可用
    while not play_audio_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('play_audio service not available, waiting again...')
    while not stop_audio_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('stop_audio service not available, waiting again...')
    while not check_playing_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('check_audio_playing service not available, waiting again...')
    
    node.get_logger().info("Audio services available")
    
    try:
        # 测试1: 播放默认目录中的音频文件
        node.get_logger().info("Test 1: Playing audio from default directory")
        
        # 确保默认音频目录存在
        default_audio_dir = '/home/aidlux/Haier_robot_ws/src/config/music'
        os.makedirs(default_audio_dir, exist_ok=True)
        
        # 创建一个简单的测试音频文件（如果不存在）
        test_audio_file = os.path.join(default_audio_dir, '自我介绍.mp3')
        if not os.path.exists(test_audio_file):
            node.get_logger().warn("Test audio file not found. Please place an audio file named '自我介绍.mp3' in the audio directory.")
        else:
            # 播放测试音频
            req = PlayAudio.Request()
            req.audio_file = '自我介绍.mp3'
            req.use_default_dir = True
            future = play_audio_client.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            node.get_logger().info(f"Play audio response: {response.success}, {response.message}")
            
            # 检查是否正在播放
            time.sleep(1)  # 等待1秒
            req = Trigger.Request()
            future = check_playing_client.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            node.get_logger().info(f"Is playing response: {response.success}, {response.message}")
            
            # 停止播放
            time.sleep(3)  # 播放3秒后停止
            req = Trigger.Request()
            future = stop_audio_client.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            node.get_logger().info(f"Stop audio response: {response.success}, {response.message}")
        
        # 测试2: 直接指定音频文件路径
        node.get_logger().info("Test 2: Playing audio with direct path")
        
        # 这里可以替换为实际的音频文件路径进行测试
        # test_path = '/home/aidlux/Haier_robot_ws/src/config/music/自我介绍.mp3'
        test_path = '/home/aidlux/Haier_robot_ws/src/config/music/自我介绍.wav'
        if os.path.exists(test_path):
            req = PlayAudio.Request()
            req.audio_file = test_path
            req.use_default_dir = False
            future = play_audio_client.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            node.get_logger().info(f"Play audio with direct path response: {response.success}, {response.message}")
            
            # 停止播放
            time.sleep(2)  # 播放2秒后停止
            req = Trigger.Request()
            future = stop_audio_client.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            node.get_logger().info(f"Stop audio response: {response.success}, {response.message}")
        else:
            node.get_logger().warn(f"Test audio file not found at: {test_path}")
        
        node.get_logger().info("Audio player test completed")
        
    except Exception as e:
        node.get_logger().error(f"Service call failed: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    test_audio_player()
