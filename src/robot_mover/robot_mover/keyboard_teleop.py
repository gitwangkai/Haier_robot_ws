#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘控制机器人移动的 ROS2 节点。
使用 WASD 键控制前进/后退/左转/右转，空格键停止，'q' 退出。
发布 Twist 消息到 /cmd_vel 话题。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import time

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        # 简单积分限制
        self.integral = max(-1.0, min(1.0, self.integral))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # 声明参数
        self.declare_parameter('linear_speed', 0.25)  # 线速度 (m/s)
        self.declare_parameter('angular_speed', 0.5)  # 角速度 (rad/s)
        
        # 发布器
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 速度参数
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # 当前速度和目标速度
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # PID控制器 for 线速度
        self.pid_linear = PID(Kp=0.25, Ki=0.05, Kd=0.005)
        
        # 保存原始终端设置，用于恢复
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('键盘控制节点启动。使用 WASD 控制移动，空格停止，q 退出。')
        self.get_logger().info(f'线速度: {self.linear_speed} m/s, 角速度: {self.angular_speed} rad/s')

    def get_key(self):
        """读取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    def run(self):
        """主循环"""
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key:
                    old_target_linear = self.target_linear
                    old_target_angular = self.target_angular
                    
                    if key == 'w':
                        self.target_linear = self.linear_speed
                        self.target_angular = 0.0
                        self.get_logger().info('前进')
                    elif key == 's':
                        self.target_linear = -self.linear_speed
                        self.target_angular = 0.0
                        self.get_logger().info('后退')
                    elif key == 'a':
                        self.target_linear = 0.0
                        self.target_angular = self.angular_speed
                        self.get_logger().info('左转')
                    elif key == 'd':
                        self.target_linear = 0.0
                        self.target_angular = -self.angular_speed
                        self.get_logger().info('右转')
                    elif key == ' ':
                        self.target_linear = 0.0
                        self.target_angular = 0.0
                        self.get_logger().info('停止')
                    elif key == 'q':
                        self.get_logger().info('退出')
                        break
                    else:
                        continue  # 忽略其他键
                    
                    # 检查目标是否改变
                    if self.target_linear != old_target_linear or self.target_angular != old_target_angular:
                        # 所有线速度变化都使用PID渐变
                        pass
                        # 转向立即
                        self.current_angular = self.target_angular
                
                # 使用PID调整线速度
                error_linear = self.target_linear - self.current_linear
                adjustment_linear = self.pid_linear.update(error_linear, 0.05)
                self.current_linear += adjustment_linear
                # 限制范围
                self.current_linear = max(-self.linear_speed, min(self.linear_speed, self.current_linear))
                
                # 角速度立即（转向不需要渐变）
                # 已经在上面设置了
                
                # 设置twist消息
                twist.linear.x = self.current_linear
                twist.angular.z = self.current_angular
                
                # 发布速度
                self.publisher.publish(twist)
                
                # 等待0.05秒
                time.sleep(0.05)
                
        except Exception as e:
            self.get_logger().error(f'错误: {e}')
        finally:
            # 退出前发送停止命令
            stop_twist = Twist()
            self.publisher.publish(stop_twist)
            self.get_logger().info('发送停止命令')
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()