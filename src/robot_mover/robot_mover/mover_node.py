#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# 消息类型
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
# 假设 BlazePose 发布的是归一化的人体中心点或 3D 坐标
# 这里为了通用，假设它发布一个标准 Point (x, y, z)
# x: 横向偏差, z: 深度/距离 (根据相机坐标系定义)

class HumanFollower(Node):
    def __init__(self):
        super().__init__('human_follower_node')

        # --- 参数设置 (可在 launch 文件中配置) ---
        self.declare_parameter('target_distance', 1.0) # 期望保持的距离 (米)
        self.declare_parameter('max_linear_speed', 0.5) # 最大线速度
        self.declare_parameter('max_angular_speed', 1.0) # 最大角速度
        self.declare_parameter('kp_linear', 0.5)       # 距离控制 P 参数
        self.declare_parameter('kp_angular', 2.0)      # 角度控制 P 参数
        
        # --- 订阅与发布 ---
        
        # 1. 订阅人体位置 (来自 BlazePose 节点)
        # 假设话题为 /blazepose/target_point
        self.create_subscription(
            Point, 
            '/blazepose/target_point', 
            self.pose_callback, 
            10
        )

        # 2. 订阅 Costmap/Keepout (用于区域过滤)
        # 注意：Nav2 的 Costmap 通常是 OccupancyGrid
        self.create_subscription(
            OccupancyGrid,
            '/keepout_filter_map',
            self.costmap_callback,
            QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )
        
        # 3. 发布速度控制
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- 内部变量 ---
        self.target_dist = self.get_parameter('target_distance').value
        self.current_map = None
        self.stop_flag = False

        self.get_logger().info('Human Follower Node Started. Waiting for BlazePose...')

    def costmap_callback(self, msg):
        """
        接收 Keepout Map。
        在简单的 Python 脚本中，要实时判断机器人是否在 Map 的黑色区域(Keepout)比较复杂，
        因为需要处理坐标变换 (Map -> Odom -> Base_link)。
        此处仅保存地图数据供后续逻辑查询。
        """
        self.current_map = msg
        # self.get_logger().info('Keepout Map received!')

    def check_safety(self):
        """
        在此处实现避障/区域过滤逻辑。
        返回: True (安全), False (危险，需要停车)
        """
        # 1. 简单的逻辑：如果收到了外部的停止信号（比如 Keepout 区域触发的事件）
        if self.stop_flag:
            return False
        
        # 2. (进阶) 你可以在这里加入 LaserScan 的判断逻辑
        # 如果前方 0.3m 有障碍物 -> return False
        
        return True

    def pose_callback(self, point_msg):
        """
        核心控制循环：接收到人体坐标后计算速度
        假设坐标系：X 为右，Z (或 Y) 为前方距离
        """
        twist = Twist()

        # 安全检查
        if not self.check_safety():
            self.get_logger().warn('Safety Stop Triggered!')
            self.vel_publisher.publish(twist) # 发送 0 速度
            return

        # 获取目标位置 (根据你的 BlazePose 输出调整)
        # 假设: point_msg.x 是横向偏移 (左负右正), point_msg.z 是距离
        person_x = point_msg.x
        person_dist = point_msg.z 

        # --- PID 控制 (这里简化为 P 控制) ---
        
        # 1. 角速度控制 (目的是让机器人正对人，即 x 趋近于 0)
        # 误差 = 0 - current_x
        error_angular = -person_x 
        angular_z = self.get_parameter('kp_angular').value * error_angular

        # 2. 线速度控制 (保持固定距离)
        # 误差 = 当前距离 - 期望距离
        error_distance = person_dist - self.target_dist
        linear_x = self.get_parameter('kp_linear').value * error_distance

        # --- 限制幅度 (Clamp) ---
        max_lin = self.get_parameter('max_linear_speed').value
        max_ang = self.get_parameter('max_angular_speed').value

        # 如果距离太近（比如小于 0.5米），强制倒车或停车
        if person_dist < 0.5:
             linear_x = 0.0
        else:
             linear_x = max(min(linear_x, max_lin), -max_lin)
        
        angular_z = max(min(angular_z, max_ang), -max_ang)

        # --- 只有在检测到人且距离合理时才移动 ---
        if person_dist > 0.1: # 简单的噪声过滤
            twist.linear.x = linear_x
            twist.angular.z = angular_z
        
        # 调试信息
        # self.get_logger().info(f'Dist: {person_dist:.2f}, Cmd -> Lin: {linear_x:.2f}, Ang: {angular_z:.2f}')

        self.vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = HumanFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点前发送停车指令
        stop_twist = Twist()
        node.vel_publisher.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()