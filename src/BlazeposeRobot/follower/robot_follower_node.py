#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Follower Node
-------------------
功能：
1. 基于视觉的人体跟随
2. 显式状态机（IDLE / FOLLOW / LOST / STOP / EMERGENCY）
3. PID 连续速度控制
4. 速度斜坡（加速度限制，防抖）
5. 通过 ROS2 /cmd_vel 输出控制

只需要在视觉循环中调用 update() 
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from enum import Enum, auto

from utils.geometry import calculate_angle


# =========================================================
# 1. 跟随状态枚举
# =========================================================
class FollowState(Enum):
    IDLE = auto()        # 没有目标
    FOLLOW = auto()      # 正常跟随
    LOST = auto()        # 短时丢失
    STOP = auto()        # 距离过近 / 丢失过久
    EMERGENCY = auto()   # 紧急状态（摔倒等）


# =========================================================
# 2. PID 控制器
# =========================================================
class PID:
    """
    简单 PID 控制器，用于速度控制
    """
    def __init__(self, kp, ki=0.0, kd=0.0, limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        if self.limit is not None:
            output = max(-self.limit, min(self.limit, output))

        return output


# =========================================================
# 3. 速度斜坡 / 加速度限制器
# =========================================================
class VelocityLimiter:
    """
    对线速度 v 和角速度 w 做加速度限制
    防止速度突变，保护底盘
    """
    def __init__(self, max_lin_acc=0.2, max_ang_acc=0.6):
        self.max_lin_acc = max_lin_acc
        self.max_ang_acc = max_ang_acc

        self.v_last = 0.0
        self.w_last = 0.0

    def reset(self):
        self.v_last = 0.0
        self.w_last = 0.0

    def limit(self, v_target, w_target, dt):
        dv = v_target - self.v_last
        dw = w_target - self.w_last

        dv = max(-self.max_lin_acc * dt, min(self.max_lin_acc * dt, dv))
        dw = max(-self.max_ang_acc * dt, min(self.max_ang_acc * dt, dw))

        self.v_last += dv
        self.w_last += dw

        return self.v_last, self.w_last


# =========================================================
# 4. 跟随 ROS2 节点
# =========================================================
class RobotFollowerNode(Node):
    """
    你可以在任意视觉节点中创建该类实例，
    然后每一帧调用 update()
    """

    def __init__(self):
        super().__init__('robot_follower')

        # /cmd_vel 发布器
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 状态机
        self.state = FollowState.IDLE

        # 丢失处理
        self.lost_counter = 0
        self.lost_threshold = 10

        # 距离相关参数
        self.min_follow_dist_ratio = 0.7    # 太近就停
        self.target_dist_ratio = 0.4        # 理想跟随距离（bbox 高度比例）

        # PID 控制器
        self.pid_forward = PID(kp=0.8, limit=0.6)     # 线速度
        self.pid_turn = PID(kp=1.2, kd=0.3, limit=1.0)  # 角速度

        # 速度斜坡
        self.vel_limiter = VelocityLimiter()

        self.last_time = time.time()

        self.get_logger().info("Robot Follower Node Started")


    # =====================================================
    # 主更新接口（视觉线程调用）
    # =====================================================
    def update(self, bboxes, classes, pose_results, frame_w, frame_h):
        """
        每一帧调用一次
        """
        now = time.time()
        dt = max(0.01, now - self.last_time)
        self.last_time = now

        target_bbox = self.select_target(bboxes, classes, frame_w)
        failure = self.check_failure(pose_results)

        # ---------------- 状态判定 ----------------
        if failure:
            self.state = FollowState.EMERGENCY

        elif target_bbox is None:
            self.lost_counter += 1
            self.state = (
                FollowState.STOP
                if self.lost_counter >= self.lost_threshold
                else FollowState.LOST
            )

        else:
            self.lost_counter = 0
            bbox_h_ratio = (target_bbox[3] - target_bbox[1]) / frame_h

            if bbox_h_ratio > self.min_follow_dist_ratio:
                self.state = FollowState.STOP
            else:
                self.state = FollowState.FOLLOW

        # ---------------- 状态执行 ----------------
        if self.state == FollowState.FOLLOW:
            self.follow_with_pid(target_bbox, frame_w, frame_h, dt)
        else:
            self.stop()

        self.get_logger().info(f"[Follower] State: {self.state.name}")
        return failure


    # =====================================================
    # 跟随控制（PID + 速度斜坡）
    # =====================================================
    def follow_with_pid(self, bbox, frame_w, frame_h, dt):
        bx, by, bx2, by2 = bbox
        cx = (bx + bx2) / 2

        # 横向误差（用于角速度）
        error_x = (cx - frame_w / 2) / (frame_w / 2)

        # 距离误差（用于线速度）
        bbox_h_ratio = (by2 - by) / frame_h
        error_dist = self.target_dist_ratio - bbox_h_ratio

        # PID 计算目标速度
        v_target = self.pid_forward.update(error_dist, dt)
        w_target = self.pid_turn.update(error_x, dt)

        # 速度斜坡限制
        v, w = self.vel_limiter.limit(v_target, w_target, dt)

        self.publish_cmd(v, w)


    # =====================================================
    # Stop / Emergency 统一出口
    # =====================================================
    def stop(self):
        self.pid_forward.reset()
        self.pid_turn.reset()
        self.vel_limiter.reset()
        self.publish_cmd(0.0, 0.0)


    # =====================================================
    # 发布 /cmd_vel
    # =====================================================
    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


    # =====================================================
    # 目标选择：最靠近画面中心的人
    # =====================================================
    def select_target(self, bboxes, classes, frame_w):
        if bboxes is None or classes is None:
            return None

        candidates = []
        for i, cls in enumerate(classes):
            if cls == 0:  # person
                bx, by, bx2, by2 = bboxes[i]
                cx = (bx + bx2) / 2
                dist = abs(cx - frame_w / 2)
                candidates.append((dist, bboxes[i]))

        if not candidates:
            return None

        candidates.sort(key=lambda x: x[0])
        return candidates[0][1]


    # =====================================================
    # Failure（摔倒）检测
    # =====================================================
    def check_failure(self, pose_results):
        if pose_results is None or not pose_results.pose_landmarks:
            return False

        lm = pose_results.pose_landmarks.landmark
        angle = calculate_angle(
            [lm[23].x, lm[23].y],
            [lm[25].x, lm[25].y],
            [lm[27].x, lm[27].y]
        )
        return angle < 150
