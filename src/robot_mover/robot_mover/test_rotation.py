#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试旋转服务接口的脚本
"""

import rclpy
from rclpy.node import Node
from robot_mover.srv import Rotate

class RotationTestClient(Node):
    def __init__(self):
        super().__init__('rotation_test_client')
        self.client = self.create_client(Rotate, '/robot_mover/rotate')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待旋转服务上线...')
        
        self.get_logger().info('旋转服务已连接')

    def send_rotation_request(self, duration=5.0):
        request = Rotate.Request()
        request.duration = duration  # 设置旋转时间
        self.get_logger().info(f'发送旋转请求... 持续时间: {duration} 秒')
        future = self.client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'旋转服务调用成功: {response.message}')
            else:
                self.get_logger().warn(f'旋转服务调用失败: {response.message}')
        else:
            self.get_logger().error('旋转服务调用失败: 无响应')

def main(args=None):
    rclpy.init(args=args)
    client = RotationTestClient()
    
    try:
        client.send_rotation_request(duration=5.0)  # 设置旋转时间为5秒
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
