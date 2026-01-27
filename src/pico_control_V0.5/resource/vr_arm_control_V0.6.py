#V0.2   修正4 6 关节为改装后的关节角度，与海尔目前的保持一致，但北京的臂4 6 关节为改装前 4关节210度超前
        #限制初始化时操作
#V0.21  修复不推摇杆时一直下发0速，影响导航
#V0.3   融合web端控制
#V0.41  到达角度限制后以及角度突变，从当前角度正解当前位姿，在正解基础上继续增量，提升可达性和稳定性
#V0.42  机械臂控制频率稳定在10HZ，方式改为定时器回调，提升稳定性
#       统一正解和逆解的角度限位计算方式，均按舵机角度-偏置计算，简化理解,ik加入限位
#V0.5   畸形解判断与修正，提升IK求解成功率，优化角度平滑滤波器，提升机械臂稳定性，目标不可达时不动保持上次成功位姿
#V0.1532 实现动态初始猜测，提升IK效率与平滑度,下发moveit控制，修正rpy位姿映射，限制数据突变，完善右臂功能
#V0.2   修正4 6 关节为改装后的关节角度，与海尔目前的保持一致，但北京的臂4 6 关节为改装前 4关节210度超前
        #限制初始化时操作
#V0.21  修复不推摇杆时一直下发0速，影响导航
#V0.3   融合web端控制
#V0.41  到达角度限制后以及角度突变，从当前角度正解当前位姿，在正解基础上继续增量，提升可达性和稳定性
#V0.42  机械臂控制频率稳定在10HZ，方式改为定时器回调，提升稳定性
#       有误差不执行角度指令，保持上次成功位姿
#V0.5   畸形解判断与修正，提升IK求解成功率，优化角度平滑滤波器，提升机械臂稳定性，目标不可达时不动保持上次成功位姿
#v0.6   修改限位，钳制后续增量控制，提升可达性
#v0.61  修复限位钳制时位姿漂移问题：触发限位后正解钳制角度更新真实位姿，保证位姿一致性
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import socket
import json
import serial
import numpy as np
import time
import os
from roboticstoolbox import DHRobot, RevoluteMDH
from scipy.spatial.transform import Rotation
import threading
import math
import subprocess


# ==============================================================================


#r是左右翻滚  p是前后俯仰  y是左右偏航

'''
右臂目标姿态(xyz): ['-7.5', '7.4', '57.7']  控制手柄向左翻滚，也就是手柄的r值，实际右臂末端是向上仰
右臂目标姿态(xyz): ['-33.6', '0.4', '-0.2'] 控制手柄向上仰，也就是手柄的p值，实际右臂末端是向右偏航
右臂目标姿态(xyz): ['0.5', '-27.5', '0.6']  控制手柄向左偏航，也就是手柄的y值，实际右臂末端是向右翻滚
'''
LEFT_ARM_PORT = '/dev/ttyUSB10'              # 左臂串口端口
RIGHT_ARM_PORT = '/dev/arm_right'            # 右臂串口端口
ARM_CONTROL_FREQ = 20                        # 机械臂控制频率（Hz）调整为V0.3的20Hz
CHASSIS_CONTROL_FREQ = 20                    # 底盘控制频率（Hz）
NORMAL_MOVE_TIME_MS = 100                    # 普通移动下发时间（ms）
RESET_MOVE_TIME_MS = 5000                    # 初始化/复位下发时间（ms）
IK_POSITION_TOLERANCE = 0.2                  # 逆解允许的最大末端误差（米）
IK_EXECUTION_TOLERANCE = 0.001               # 逆解执行允许的最大误差（米），误差大于此值不执行
POSITION_INCREMENT_SCALE = 0.5               # 末端位置增量缩放系数
ROTATION_INCREMENT_SCALE = 0.5               # 末端姿态增量缩放系数
MAX_LINEAR_VELOCITY = 0.2                    # 底盘最大线速度（m/s）
MAX_ANGULAR_VELOCITY = 0.5                   # 底盘最大角速度（rad/s）
MAX_LINEAR_ACCELERATION = 0.2                # 底盘最大线加速度（m/s^2）
MAX_ANGULAR_ACCELERATION = 0.5               # 底盘最大角加速度（rad/s^2）
SERVO_FRAME_HEADER = 0x55                    # 舵机协议帧头
SERVO_CMD_MOVE = 1                           # 舵机移动命令码
SERVO_CMD_UNLOAD = 31                        # 舵机卸力命令码
SERVO_CMD_READ_POS = 28                      # 舵机读取位置命令码
ANGLE_JUMP_THRESHOLD = 80                    # 单关节最大允许跳变（度），防止机械臂突然抡臂
LEFT_ARM_REVERSE_JOINTS = [5]            # 左臂需要反转的关节索引（0-based）
RIGHT_ARM_REVERSE_JOINTS = [5]           # 右臂需要反转的关节索引（0-based，与V0.3保持一致）
ROBOT_CONFIG = {
    "LEFT_INIT_POSE": [-0.26658, -0.20024, -0.15363],
    "RIGHT_INIT_POSE": [-0.26658, -0.20024, 0.15363],
    "SERVO_IDS": [1, 2, 3, 4, 5, 6, 7],
    "LEFT_INIT_ANGLES":  [120, 120, 120, 210, 120, 120, 80],
    "RIGHT_INIT_ANGLES": [120, 120, 120, 210, 120, 120, 80],
    "JOINT_OFFSETS": [120, 210, 210, 120, 120, 120],
    "SERVO_ANGLE_LIMITS": [
        (0, 240), (120, 240), (15, 235),
        (110, 210), (2, 230), (42, 200),
    ],
    #"SERVO_ANGLE_LIMITS": [
    #    (0, 240), (0, 240), (0, 240),
    #    (0, 240), (0, 240), (0, 240),
    #],
    "LEFT_ARM_JOINT_NAMES": ['L_joint1', 'L_joint2', 'L_joint3', 'L_joint4', 'L_joint5', 'L_joint6'],
    "RIGHT_ARM_JOINT_NAMES": ['R_joint1', 'R_joint2', 'R_joint3', 'R_joint4', 'R_joint5', 'R_joint6'],
}
# ==============================================================================


class VRArmController(Node):
    def __init__(self):
        super().__init__('vr_arm_controller_node')
        self.cmdvel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.left_traj_pub = self.create_publisher(JointTrajectory, '/L_group_controller/joint_trajectory', 10)
        self.right_traj_pub = self.create_publisher(JointTrajectory, '/R_group_controller/joint_trajectory', 10)
        
        self.left_init_pose = np.array(ROBOT_CONFIG["LEFT_INIT_POSE"])
        self.right_init_pose = np.array(ROBOT_CONFIG["RIGHT_INIT_POSE"])
        self.servo_ids = ROBOT_CONFIG["SERVO_IDS"]
        self.left_init_angles = ROBOT_CONFIG["LEFT_INIT_ANGLES"]
        self.right_init_angles = ROBOT_CONFIG["RIGHT_INIT_ANGLES"]
        self.joint_offsets = ROBOT_CONFIG["JOINT_OFFSETS"]
        self.servo_angle_limits = ROBOT_CONFIG["SERVO_ANGLE_LIMITS"]
        self.left_arm_joint_names = ROBOT_CONFIG["LEFT_ARM_JOINT_NAMES"]
        self.right_arm_joint_names = ROBOT_CONFIG["RIGHT_ARM_JOINT_NAMES"]
        
        self.left_current_pose = self.left_init_pose.copy()
        self.right_current_pose = self.right_init_pose.copy()
        self.last_left_pos = None
        self.last_right_pos = None
        self.base_orientation = Rotation.from_euler('zyx', [0, 0, 0])
        self.left_current_rot = self.base_orientation
        self.right_current_rot = self.base_orientation
        self.last_left_rot = None
        self.last_right_rot = None
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.target_joy_x = 0.0
        self.target_joy_y = 0.0
        self.chassis_control_period = 1.0 / CHASSIS_CONTROL_FREQ
        self.last_left_angles_sent = self.left_init_angles.copy()
        self.last_right_angles_sent = self.right_init_angles.copy()
        self.gripper_angle_threshold = 1.0
        self.left_arm_initialized = False
        self.right_arm_initialized = False
        self.position_scale = POSITION_INCREMENT_SCALE
        self.rotation_scale = ROTATION_INCREMENT_SCALE
        self.initial_guess = np.array([0, -np.pi/2, 0, 1.4835, 1.6581, 0])
        self.last_ik_solution_left = self.initial_guess.copy()
        self.last_ik_solution_right = self.initial_guess.copy()
        
        self.create_arm_models()
        self.chassis_timer = self.create_timer(self.chassis_control_period, self.chassis_control_callback)

    def send_angles_to_moveit(self, arm_publisher, joint_names, final_angles_deg, travel_time_ms, arm_name):
        """将最终舵机角度(0-240)转换为以120为零点的moveit!理论角度(弧度)并发布"""
        if len(final_angles_deg) < 6:
            self.get_logger().warn(f"[{arm_name} moveit!] 发送角度不足6个，已跳过。")
            return

        print(f'  [moveit! {arm_name}] 接收到的最终下发角度(deg): {[f"{a:.2f}" for a in final_angles_deg[:6]]}')
        
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        
        theoretical_angles_rad = []
        for i in range(6):
            # --- 核心修正点：以120度为新的零点 ---
            # 1. 获取最终的舵机角度
            servo_angle = final_angles_deg[i]
            
            # 2. 将其平移，使120度对应理论上的0度
            theoretical_angle_deg = servo_angle - 120.0
            
            # 3. 转换为弧度
            theoretical_angles_rad.append(math.radians(theoretical_angle_deg))

        point.positions = theoretical_angles_rad
        travel_time_sec = travel_time_ms / 1000.0
        point.time_from_start = Duration(sec=int(travel_time_sec), nanosec=int((travel_time_sec % 1) * 1e9))
        
        msg.points.append(point)
        arm_publisher.publish(msg)
        
        angles_rad_str = [f'{rad:+.2f}' for rad in theoretical_angles_rad]
        print(f"  [moveit! {arm_name}] 已发布轨迹, 目标(rad): {angles_rad_str}")
    
    def chassis_control_callback(self):
        if self.left_arm_initialized and self.right_arm_initialized:
            target_linear_vel = self.target_joy_y * MAX_LINEAR_VELOCITY
            target_angular_vel = -self.target_joy_x * MAX_ANGULAR_VELOCITY
            max_linear_change = MAX_LINEAR_ACCELERATION * self.chassis_control_period
            max_angular_change = MAX_ANGULAR_ACCELERATION * self.chassis_control_period
            linear_diff = np.clip(target_linear_vel - self.current_linear_vel, -max_linear_change, max_linear_change)
            angular_diff = np.clip(target_angular_vel - self.current_angular_vel, -max_angular_change, max_angular_change)
            self.current_linear_vel += linear_diff
            self.current_angular_vel += angular_diff
            # 只有当摇杆有输入时才发布速度
            if abs(self.target_joy_x) > 1e-3 or abs(self.target_joy_y) > 1e-3 or abs(self.current_linear_vel) > 1e-3 or abs(self.current_angular_vel) > 1e-3:
                twist = Twist()
                twist.linear.x = self.current_linear_vel
                twist.angular.z = self.current_angular_vel
                self.cmdvel_pub.publish(twist)
                print(f"[CMDVEL] 摇杆:[{self.target_joy_y:+.2f},{self.target_joy_x:+.2f}] -> 发布速度:[{self.current_linear_vel:+.2f},{self.current_angular_vel:+.2f}]")
        else:
            if self.current_linear_vel != 0.0 or self.current_angular_vel != 0.0:
                self.cmdvel_pub.publish(Twist())
                self.current_linear_vel = 0.0
                self.current_angular_vel = 0.0
            if abs(self.target_joy_x) > 1e-3 or abs(self.target_joy_y) > 1e-3:
                print("⚠️ 底盘已锁定，请先初始化双臂！")

    def create_arm_models(self):
        # 采用V0.3的宽松DH限位策略，确保IK求解连续性
        # 在IK层面使用宽松限位，在舵机层面进行严格安全保护
        print("� 采用V0.3的宽松DH限位策略，确保IK求解连续性...")
        
        self.left_arm = DHRobot([
            RevoluteMDH(d=-0.15363, a=0, alpha=0,         qlim=[0, 4.18879]),
            RevoluteMDH(d=0,        a=0, alpha=np.pi/2,   qlim=[0, 4.18879]),
            RevoluteMDH(d=-0.26658, a=0, alpha=-np.pi/2,  qlim=[0, 4.18879]),
            RevoluteMDH(d=0,        a=0, alpha=np.pi/2,   qlim=[0, 4.18879]),
            RevoluteMDH(d=-0.20024, a=0, alpha=-np.pi/2,  qlim=[0, 4.18879]),
            RevoluteMDH(d=0,        a=0, alpha=np.pi/2,   qlim=[0, 4.18879])
        ], name="Left-arm")
        
        self.right_arm = DHRobot([
            RevoluteMDH(d=0.15363,  a=0, alpha=0,         qlim=[0, 4.18879]),
            RevoluteMDH(d=0,        a=0, alpha=-np.pi/2,  qlim=[0, 4.18879]),
            RevoluteMDH(d=0.26658,  a=0, alpha=np.pi/2,   qlim=[0, 4.18879]),
            RevoluteMDH(d=0,        a=0, alpha=-np.pi/2,  qlim=[0, 4.18879]),
            RevoluteMDH(d=0.20024,  a=0, alpha=np.pi/2,   qlim=[0, 4.18879]),
            RevoluteMDH(d=0,        a=0, alpha=-np.pi/2,  qlim=[0, 4.18879])
        ], name="Right-arm")

    def servo_to_ik_angle(self, sent_angle, joint_index, arm_reverse_joints):
        """将舵机角度转换为IK理论角度（公共函数）"""
        offset = self.joint_offsets[joint_index]
        if joint_index in arm_reverse_joints:
            return offset - sent_angle
        else:
            return sent_angle - offset
    
    def ik_to_servo_angle(self, ik_angle, joint_index, arm_reverse_joints):
        """将IK理论角度转换为舵机角度（公共函数）"""
        offset = self.joint_offsets[joint_index]
        if joint_index in arm_reverse_joints:
            return offset - ik_angle
        else:
            return ik_angle + offset
    
    def reset_arm_pose_from_current_angles(self, arm_model, sent_angles, arm_reverse_joints):
        """从当前舵机角度正解位姿（公共函数）"""
        current_joint_angles = []
        for i, sent_angle in enumerate(sent_angles[:6]):
            angle = self.servo_to_ik_angle(sent_angle, i, arm_reverse_joints)
            current_joint_angles.append(angle)
        
        current_angles_rad = np.radians(current_joint_angles)
        fk_result = arm_model.fkine(current_angles_rad)
        return fk_result.t, Rotation.from_matrix(fk_result.R)

    def calculate_arm_ik(self, arm_model, target_position, target_rotation, q0):
        T_target = np.eye(4)
        T_target[:3,:3] = target_rotation.as_matrix()
        T_target[:3,3] = target_position
        ik_solution = arm_model.ikine_LM(Tep=T_target,q0=q0,ilimit=50)
        joint_angles_rad = ik_solution.q
        joint_angles_deg = np.round(np.rad2deg(joint_angles_rad),2).tolist()
        fk_result = arm_model.fkine(joint_angles_rad)
        position_error = np.linalg.norm(target_position-fk_result.t)
        
        # 检查IK解是否成功且在限位范围内
        if ik_solution.success:
            print(f"    [IK] ✅ 求解成功，位置误差: {position_error:.4f}m")
        else:
            print(f"    [IK] ⚠️ 求解失败或超出关节限位，位置误差: {position_error:.4f}m")
            
        return joint_angles_deg,position_error,joint_angles_rad

    def unload_all_servos(self, port, arm_name):
        print(f'🔓 正在卸载 {arm_name} 臂所有舵机...')
        try:
            with serial.Serial(port,baudrate=115200,timeout=1) as ser:
                for servo_id in self.servo_ids:
                    data=[0]
                    packet=[SERVO_FRAME_HEADER,SERVO_FRAME_HEADER,servo_id,3+len(data),SERVO_CMD_UNLOAD]
                    packet.extend(data)
                    checksum = (~sum(packet[2:])) & 0xFF
                    packet.append(checksum)
                    ser.write(bytes(packet))
                    time.sleep(0.01)
            print(f'🎯 {arm_name} 臂所有舵机已卸力。')
        except Exception as e:
            print(f'❌ {arm_name} 臂卸力失败: {e}')
    
    def _send_servo_command(self, port, angles, time_ms):
        def checksum(data): return (~sum(data[2:])) & 0xFF
        def angle_to_position(angle): return int(np.clip(angle,0,240)/240*1000)
        def send_move_command_packet(ser,servo_id,position,duration):
            pos_low=position&0xFF; pos_high=(position>>8)&0xFF
            time_low=duration&0xFF; time_high=(duration>>8)&0xFF
            packet=[SERVO_FRAME_HEADER,SERVO_FRAME_HEADER,servo_id,7,SERVO_CMD_MOVE,pos_low,pos_high,time_low,time_high]
            packet.append(checksum(packet))
            ser.write(bytes(packet))
        try:
            with serial.Serial(port,baudrate=115200,timeout=1) as ser:
                for idx,angle in enumerate(angles):
                    servo_id = self.servo_ids[idx]
                    position=angle_to_position(angle)
                    send_move_command_packet(ser,servo_id,position,time_ms)
        except Exception as e:
            arm_name="左臂" if "left" in port or "USB10" in port else "右臂"
            print(f"{arm_name}串口发送失败: {e}")

    def read_servo_angles(self, port, servo_ids):
        """读取指定舵机的当前角度"""
        def checksum(data): return (~sum(data[2:])) & 0xFF
        def read_servo_position(ser, servo_id):
            # 发送读取位置命令
            packet = [SERVO_FRAME_HEADER, SERVO_FRAME_HEADER, servo_id, 3, SERVO_CMD_READ_POS]
            packet.append(checksum(packet))
            
            ser.flushInput()
            ser.write(bytes(packet))
            time.sleep(0.05)  # 等待响应
            
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                if len(data) >= 7 and data[0] == 0x55 and data[1] == 0x55 and data[2] == servo_id:
                    position = data[5] + (data[6] << 8)
                    angle = (position / 1000.0) * 240.0  # 转换为角度
                    return angle
            return None
        
        current_angles = []
        try:
            with serial.Serial(port, baudrate=115200, timeout=0.2) as ser:
                for servo_id in servo_ids:
                    angle = read_servo_position(ser, servo_id)
                    if angle is not None:
                        current_angles.append(angle)
                    else:
                        # 如果读取失败，使用上次记录的角度
                        if port == LEFT_ARM_PORT:
                            current_angles.append(self.last_left_angles_sent[len(current_angles)])
                        else:
                            current_angles.append(self.last_right_angles_sent[len(current_angles)])
                        print(f"⚠️ 读取舵机ID{servo_id}失败，使用记录角度")
        except Exception as e:
            arm_name = "左臂" if "left" in port or "USB10" in port else "右臂"
            print(f"❌ {arm_name}舵机角度读取失败: {e}")
            # 返回记录的角度作为备用
            if port == LEFT_ARM_PORT:
                return self.last_left_angles_sent[:6]
            else:
                return self.last_right_angles_sent[:6]
        
        return current_angles

    def control_left_arm(self, angles, time_ms=100):
        self._send_servo_command(LEFT_ARM_PORT, angles, time_ms)
            
    def control_right_arm(self, angles, time_ms=100):
        self._send_servo_command(RIGHT_ARM_PORT, angles, time_ms)

    def process_arm_data(self, vr_data):
        left_primary = vr_data.get('isLeftPrimary', False)
        right_primary = vr_data.get('isRightPrimary', False)
        left_secondary = vr_data.get('isLeftSecondary', False)
        right_secondary = vr_data.get('isRightSecondary', False)
        
        self.target_joy_x = vr_data.get('rightJoystick', {}).get('x', 0.0)
        self.target_joy_y = vr_data.get('rightJoystick', {}).get('y', 0.0)

        if right_primary and right_secondary:
            print("\n" + "!"*15 + " 紧急停止触发！ " + "!"*15)
            self.target_joy_x = 0.0
            self.target_joy_y = 0.0
            self.unload_all_servos(LEFT_ARM_PORT, "左臂")
            self.unload_all_servos(RIGHT_ARM_PORT, "右臂")
            if self.left_arm_initialized or self.right_arm_initialized:
                print(" 安全锁已重置，松开按键后需要重新初始化。")
                self.left_arm_initialized = False
                self.right_arm_initialized = False
            return

        send_left_cmd = False
        send_right_cmd = False
        new_left_angles = self.last_left_angles_sent.copy()
        new_right_angles = self.last_right_angles_sent.copy()

        if left_secondary:
            print("正在初始化左臂...")
            self.left_current_pose = self.left_init_pose.copy()
            self.left_current_rot = self.base_orientation
            self.last_left_pos = None
            self.last_left_rot = None
            self.last_ik_solution_left = self.initial_guess.copy()
            new_left_angles = self.left_init_angles.copy()
            send_left_cmd = True
            if not self.left_arm_initialized:
                self.left_arm_initialized = True
                print("✅ 左臂已初始化，功能已解锁。")
            
        if right_secondary:
            print("正在初始化右臂...")
            self.right_current_pose = self.right_init_pose.copy()
            self.right_current_rot = self.base_orientation
            self.last_right_pos = None
            self.last_right_rot = None
            self.last_ik_solution_right = self.initial_guess.copy()
            new_right_angles = self.right_init_angles.copy()
            send_right_cmd = True
            if not self.right_arm_initialized:
                self.right_arm_initialized = True
                print("✅ 右臂已初始化，功能已解锁。")

        left_trigger = vr_data.get('leftTrigger', 0.0)
        right_trigger = vr_data.get('rightTrigger', 0.0)
        
        if self.left_arm_initialized:
            if left_primary:
                lp = vr_data.get('leftPosition', {})
                lq = vr_data.get('leftRotation', {})
                current_left_pos = np.array([lp.get(c, 0) for c in 'xyz'])
                current_left_rot = Rotation.from_quat([lq.get(c, 0) for c in 'xyzw'])
                if self.last_left_pos is not None and self.last_left_rot is not None:
                    pos_delta = (current_left_pos - self.last_left_pos) * self.position_scale
                    mapped_pos_delta = np.array([pos_delta[1], -pos_delta[2], pos_delta[0]])
                    
                    # 保存当前位姿状态，用于误差时回滚
                    prev_pose = self.left_current_pose.copy()
                    prev_rot = self.left_current_rot
                    
                    # 尝试更新位姿
                    self.left_current_pose += mapped_pos_delta
                    rot_delta = current_left_rot * self.last_left_rot.inv()
                    rot_delta_scaled = Rotation.from_rotvec(rot_delta.as_rotvec() * self.rotation_scale)
                    self.left_current_rot = rot_delta_scaled * self.left_current_rot
                    rpy_delta_deg = np.rad2deg(rot_delta_scaled.as_euler('xyz'))
                    print("左臂增量:")
                    print(f"  - Pos(dx,dy,dz):[{mapped_pos_delta[0]:>6.3f},{mapped_pos_delta[1]:>6.3f},{mapped_pos_delta[2]:>6.3f}]")
                    print(f"  - Rot(d_r,d_p,d_y):[{rpy_delta_deg[0]:>6.1f},{rpy_delta_deg[1]:>6.1f},{rpy_delta_deg[2]:>6.1f}]")
                    print(f"左臂目标位姿: {[f'{p:.3f}' for p in self.left_current_pose]}")
                    target_rpy_deg = np.rad2deg(self.left_current_rot.as_euler('zyx'))
                    print(f"左臂目标姿态(zyx): {[f'{p:.1f}' for p in target_rpy_deg]}")
                    try:
                        joint_angles, pos_error, solution_rad = self.calculate_arm_ik(self.left_arm, self.left_current_pose, self.left_current_rot, q0=self.last_ik_solution_left)
                        print(f"  [DEBUG] 左臂IK角度(deg): {[f'{a:.2f}' for a in joint_angles]}")
                        if pos_error <= IK_POSITION_TOLERANCE:
                            # 添加执行精度检查：误差过大时不执行角度指令，回滚位姿保持机械臂不动
                            if pos_error > IK_EXECUTION_TOLERANCE:
                                print(f"  -> 🟠 左臂IK误差过大({pos_error:.4f}m > {IK_EXECUTION_TOLERANCE}m)，回滚位姿保持机械臂不动")
                                # 回滚到上一次有效位姿
                                self.left_current_pose = prev_pose
                                self.left_current_rot = prev_rot
                            else:
                                self.last_ik_solution_left = solution_rad
                                unlimited_arm_angles = [angle + offset for angle, offset in zip(joint_angles, self.joint_offsets)]
                                limited_arm_angles = []
                                is_limited = False
                                for i, angle in enumerate(unlimited_arm_angles):
                                    min_val, max_val = self.servo_angle_limits[i]
                                    clipped_angle = np.clip(angle, min_val, max_val)
                                    if clipped_angle != angle: is_limited = True
                                    limited_arm_angles.append(clipped_angle)
                                if is_limited:
                                    print(f"  -> 🟡 安全限位触发(左臂)")
                                    print(f"     原始计算: {[f'{a:.2f}' for a in unlimited_arm_angles]}")
                                    print(f"     钳制后下发: {[f'{a:.2f}' for a in limited_arm_angles]}")
                                    # 🔧 修复：钳制后需要正解实际位姿，避免位姿漂移
                                    try:
                                        self.left_current_pose, self.left_current_rot = self.reset_arm_pose_from_current_angles(
                                            self.left_arm, limited_arm_angles, LEFT_ARM_REVERSE_JOINTS)
                                        print(f"     ✅ 已正解钳制后的真实位姿: {[f'{p:.3f}' for p in self.left_current_pose]}")
                                    except Exception as e:
                                        print(f"     ❌ 正解钳制位姿失败: {e}")
                                new_left_angles[:6] = limited_arm_angles
                                send_left_cmd = True
                        elif pos_error > IK_POSITION_TOLERANCE:
                            print(f"  -> 左臂目标不可达, 误差={pos_error:.4f}，回滚位姿并从实际舵机角度正解！")
                            # 回滚到上一次有效位姿
                            self.left_current_pose = prev_pose
                            self.left_current_rot = prev_rot
                            try:
                                actual_angles = self.read_servo_angles(LEFT_ARM_PORT, self.servo_ids[:6])
                                print(f"📖 读取左臂实际舵机角度: {[f'{a:.2f}' for a in actual_angles]}")
                                self.left_current_pose, self.left_current_rot = self.reset_arm_pose_from_current_angles(
                                    self.left_arm, actual_angles, LEFT_ARM_REVERSE_JOINTS)
                                print(f"📍 重置左臂位姿: {[f'{p:.3f}' for p in self.left_current_pose]}")
                            except Exception as e:
                                print(f"❌ 左臂位姿重置失败: {e}")
                    except Exception as e:
                        print(f"左臂IK计算失败: {e}")
                        # IK计算失败时也回滚位姿
                        self.left_current_pose = prev_pose
                        self.left_current_rot = prev_rot
                self.last_left_pos = current_left_pos
                self.last_left_rot = current_left_rot
            else:
                self.last_left_pos = None
                self.last_left_rot = None
            
            target_left_gripper = 80.0 + left_trigger * 40.0
            if abs(target_left_gripper - self.last_left_angles_sent[6]) > self.gripper_angle_threshold:
                final_gripper_angle = max(80, min(120, target_left_gripper))
                print(f"[左夹爪] 扳机:{left_trigger:.3f}->角度:{final_gripper_angle:.2f}")
                new_left_angles[6] = final_gripper_angle
                send_left_cmd = True
        elif left_primary or left_trigger > 0.05:
            print("⚠️ 左臂已锁定，请先按 Y 键初始化！")

        if self.right_arm_initialized:
            if right_primary:
                rp = vr_data.get('rightPosition', {})
                rq = vr_data.get('rightRotation', {})
                current_right_pos = np.array([rp.get(c, 0) for c in 'xyz'])
                # 修正：手柄四元数转欧拉角（xyz），再按xyz顺序重组为Rotation对象，实现rpy一一对应
                raw_right_rot = Rotation.from_quat([rq.get(c, 0) for c in 'xyzw'])
                hand_rpy = raw_right_rot.as_euler('xyz')
                # 物理轴顺序映射：机械臂roll=手柄yaw，pitch=手柄roll，yaw=手柄pitch
                arm_rpy = [-hand_rpy[1], hand_rpy[2], -hand_rpy[0]]
                current_right_rot = Rotation.from_euler('xyz', arm_rpy)
                if self.last_right_pos is not None and self.last_right_rot is not None:
                    pos_delta = (current_right_pos - self.last_right_pos) * self.position_scale
                    mapped_pos_delta = np.array([pos_delta[1], -pos_delta[2], pos_delta[0]])
                    
                    # 保存当前位姿状态，用于误差时回滚
                    prev_pose = self.right_current_pose.copy()
                    prev_rot = self.right_current_rot
                    
                    # 尝试更新位姿
                    self.right_current_pose += mapped_pos_delta
                    rot_delta = current_right_rot * self.last_right_rot.inv()
                    rot_delta_scaled = Rotation.from_rotvec(rot_delta.as_rotvec() * self.rotation_scale)
                    self.right_current_rot = rot_delta_scaled * self.right_current_rot
                    rpy_delta_deg = np.rad2deg(rot_delta_scaled.as_euler('xyz'))
                    print("右臂增量:")
                    print(f"  - Pos(dx,dy,dz):[{mapped_pos_delta[0]:>6.3f},{mapped_pos_delta[1]:>6.3f},{mapped_pos_delta[2]:>6.3f}]")
                    print(f"  - Rot(d_r,d_p,d_y):[{rpy_delta_deg[0]:>6.1f},{rpy_delta_deg[1]:>6.1f},{rpy_delta_deg[2]:>6.1f}]")
                    print(f"右臂目标位姿: {[f'{p:.3f}' for p in self.right_current_pose]}")

                    target_rpy_deg = np.rad2deg(self.right_current_rot.as_euler('xyz'))
                    print(f"右臂目标姿态(xyz): {[f'{p:.1f}' for p in target_rpy_deg]}")

                    try:
                        joint_angles, pos_error, solution_rad = self.calculate_arm_ik(self.right_arm, self.right_current_pose, self.right_current_rot, q0=self.last_ik_solution_right)
                        print(f"  [DEBUG] 右臂IK角度(deg): {[f'{a:.2f}' for a in joint_angles]}")
                        if pos_error <= IK_POSITION_TOLERANCE:
                            # 添加执行精度检查：误差过大时不执行角度指令，回滚位姿保持机械臂不动
                            if pos_error > IK_EXECUTION_TOLERANCE:
                                print(f"  -> 🟠 右臂IK误差过大({pos_error:.4f}m > {IK_EXECUTION_TOLERANCE}m)，回滚位姿保持机械臂不动")
                                # 回滚到上一次有效位姿
                                self.right_current_pose = prev_pose
                                self.right_current_rot = prev_rot
                            else:
                                self.last_ik_solution_right = solution_rad
                                unlimited_arm_angles = []
                                #对关节进行反转，使用V0.42的配置化反转逻辑
                                for i, angle in enumerate(joint_angles):
                                    offset = self.joint_offsets[i]
                                    if i in RIGHT_ARM_REVERSE_JOINTS: 
                                        final_angle = offset - angle
                                    else:
                                        final_angle = angle + offset
                                    unlimited_arm_angles.append(final_angle)
                                limited_arm_angles = []
                                is_limited = False
                                for i, angle in enumerate(unlimited_arm_angles):
                                    min_val, max_val = self.servo_angle_limits[i]
                                    clipped_angle = np.clip(angle, min_val, max_val)
                                    if clipped_angle != angle:
                                        is_limited = True
                                    limited_arm_angles.append(clipped_angle)
                                if is_limited:
                                    print(f"  -> 🟡 安全限位触发(右臂)")
                                    print(f"     原始计算: {[f'{a:.2f}' for a in unlimited_arm_angles]}")
                                    print(f"     钳制后下发: {[f'{a:.2f}' for a in limited_arm_angles]}")
                                    # 🔧 修复：钳制后需要正解实际位姿，避免位姿漂移
                                    try:
                                        self.right_current_pose, self.right_current_rot = self.reset_arm_pose_from_current_angles(
                                            self.right_arm, limited_arm_angles, RIGHT_ARM_REVERSE_JOINTS)
                                        print(f"     ✅ 已正解钳制后的真实位姿: {[f'{p:.3f}' for p in self.right_current_pose]}")
                                    except Exception as e:
                                        print(f"     ❌ 正解钳制位姿失败: {e}")
                                new_right_angles[:6] = limited_arm_angles
                                send_right_cmd = True
                        elif pos_error > IK_POSITION_TOLERANCE:
                            print(f"  -> 右臂目标不可达, 误差={pos_error:.4f}，回滚位姿并从实际舵机角度正解！")
                            # 回滚到上一次有效位姿
                            self.right_current_pose = prev_pose
                            self.right_current_rot = prev_rot
                            try:
                                actual_angles = self.read_servo_angles(RIGHT_ARM_PORT, self.servo_ids[:6])
                                print(f"📖 读取右臂实际舵机角度: {[f'{a:.2f}' for a in actual_angles]}")
                                self.right_current_pose, self.right_current_rot = self.reset_arm_pose_from_current_angles(
                                    self.right_arm, actual_angles, RIGHT_ARM_REVERSE_JOINTS)
                                print(f"📍 重置右臂位姿: {[f'{p:.3f}' for p in self.right_current_pose]}")
                            except Exception as e:
                                print(f"❌ 右臂位姿重置失败: {e}")
                    except Exception as e:
                        print(f"右臂IK计算失败: {e}")
                        # IK计算失败时也回滚位姿
                        self.right_current_pose = prev_pose
                        self.right_current_rot = prev_rot
                self.last_right_pos = current_right_pos
                self.last_right_rot = current_right_rot
            else:
                self.last_right_pos = None
                self.last_right_rot = None
            
            target_right_gripper = 80.0 + right_trigger * 40.0
            if abs(target_right_gripper - self.last_right_angles_sent[6]) > self.gripper_angle_threshold:
                final_gripper_angle = max(80, min(120, target_right_gripper))
                print(f"[右夹爪] 扳机:{right_trigger:.3f}->角度:{final_gripper_angle:.2f}")
                new_right_angles[6] = final_gripper_angle
                send_right_cmd = True
        elif right_primary or right_trigger > 0.05:
            print("⚠️ 右臂已锁定，请先按 B 键初始化！")

        if send_left_cmd:
            final_angles = [max(0, min(240, angle)) for angle in new_left_angles]
            # 仅在非初始化流程时做关节跳变检测
            if not left_secondary and self.last_left_angles_sent:
                for i, (last_a, new_a) in enumerate(zip(self.last_left_angles_sent[:6], final_angles[:6])):
                    if abs(new_a - last_a) > ANGLE_JUMP_THRESHOLD:
                        print(f"⚠️ 左臂关节{i+1}角度突变: {last_a:.2f} -> {new_a:.2f}，从实际舵机角度正解位姿！")
                        # 读取实际舵机角度并正解位姿
                        try:
                            actual_angles = self.read_servo_angles(LEFT_ARM_PORT, self.servo_ids[:6])
                            print(f"📖 读取左臂实际舵机角度: {[f'{a:.2f}' for a in actual_angles]}")
                            self.left_current_pose, self.left_current_rot = self.reset_arm_pose_from_current_angles(
                                self.left_arm, actual_angles, LEFT_ARM_REVERSE_JOINTS)
                            print(f"📍 重置左臂位姿: {[f'{p:.3f}' for p in self.left_current_pose]}")
                        except Exception as e:
                            print(f"❌ 左臂位姿重置失败: {e}")
                        send_left_cmd = False  # 检测到突变，不发送手臂命令，但继续处理夹爪等
                        break  # 检测到一个突变就停止检查，避免重复重置
            # 添加角度平滑：限制每帧最大角度变化，减少抖动但保持响应性
            if not left_secondary and self.last_left_angles_sent:
                smoothed_angles = []
                max_angle_change_per_frame = 5.0  # 度每帧，调整此值控制平滑程度
                for i, target in enumerate(final_angles[:6]):
                    last = self.last_left_angles_sent[i]
                    delta = target - last
                    clamped_delta = np.clip(delta, -max_angle_change_per_frame, max_angle_change_per_frame)
                    smoothed = last + clamped_delta
                    smoothed_angles.append(smoothed)
                smoothed_angles.append(final_angles[6])  # 夹爪不平滑
                final_angles = smoothed_angles
                print(f"  [平滑] 左臂平滑后角度(deg): {[f'{a:.2f}' for a in final_angles]}")
            print(f"  [DEBUG] 左臂最终下发角度(deg): {[f'{a:.2f}' for a in final_angles]}")
            print(f"  [执行] 左臂当前位姿: Pos{[f'{p:.3f}' for p in self.left_current_pose]} Rot{[f'{r:.1f}' for r in np.rad2deg(self.left_current_rot.as_euler('zyx'))]}")
            exec_time = RESET_MOVE_TIME_MS if left_secondary else NORMAL_MOVE_TIME_MS
            self.last_left_angles_sent = final_angles.copy()
            #下发moveit测试使用
            #self.send_angles_to_moveit(self.left_traj_pub, self.left_arm_joint_names, final_angles, exec_time, "左臂")
            self.control_left_arm(final_angles, time_ms=exec_time)
            if left_secondary:
                print("左臂初始化中，请等待5秒钟...")
                time.sleep(5)  # 初始化后强制等待5秒
                print("左臂初始化完成。")

        if send_right_cmd:
            final_angles = [max(0, min(240, angle)) for angle in new_right_angles]
            # 仅在非初始化流程时做关节跳变检测
            if not right_secondary and self.last_right_angles_sent:
                for i, (last_a, new_a) in enumerate(zip(self.last_right_angles_sent[:6], final_angles[:6])):
                    if abs(new_a - last_a) > ANGLE_JUMP_THRESHOLD:
                        print(f"⚠️ 右臂关节{i+1}角度突变: {last_a:.2f} -> {new_a:.2f}，从实际舵机角度正解位姿！")
                        # 读取实际舵机角度并正解位姿
                        try:
                            actual_angles = self.read_servo_angles(RIGHT_ARM_PORT, self.servo_ids[:6])
                            print(f"📖 读取右臂实际舵机角度: {[f'{a:.2f}' for a in actual_angles]}")
                            self.right_current_pose, self.right_current_rot = self.reset_arm_pose_from_current_angles(
                                self.right_arm, actual_angles, RIGHT_ARM_REVERSE_JOINTS)
                            print(f"📍 重置右臂位姿: {[f'{p:.3f}' for p in self.right_current_pose]}")
                        except Exception as e:
                            print(f"❌ 右臂位姿重置失败: {e}")
                        send_right_cmd = False  # 检测到突变，不发送手臂命令，但继续处理夹爪等
                        break  # 检测到一个突变就停止检查，避免重复重置
            # 添加角度平滑：限制每帧最大角度变化，减少抖动但保持响应性
            if not right_secondary and self.last_right_angles_sent:
                smoothed_angles = []
                max_angle_change_per_frame = 5.0  # 度每帧，调整此值控制平滑程度
                for i, target in enumerate(final_angles[:6]):
                    last = self.last_right_angles_sent[i]
                    delta = target - last
                    clamped_delta = np.clip(delta, -max_angle_change_per_frame, max_angle_change_per_frame)
                    smoothed = last + clamped_delta
                    smoothed_angles.append(smoothed)
                smoothed_angles.append(final_angles[6])  # 夹爪不平滑
                final_angles = smoothed_angles
                print(f"  [平滑] 右臂平滑后角度(deg): {[f'{a:.2f}' for a in final_angles]}")
            print(f"  [DEBUG] 右臂最终下发角度(deg): {[f'{a:.2f}' for a in final_angles]}")
            print(f"  [执行] 右臂当前位姿: Pos{[f'{p:.3f}' for p in self.right_current_pose]} Rot{[f'{r:.1f}' for r in np.rad2deg(self.right_current_rot.as_euler('xyz'))]}")
            exec_time = RESET_MOVE_TIME_MS if right_secondary else NORMAL_MOVE_TIME_MS
            self.last_right_angles_sent = final_angles.copy()
            #下发moveit测试使用
            #self.send_angles_to_moveit(self.right_traj_pub, self.right_arm_joint_names, final_angles, exec_time, "右臂")
            self.control_right_arm(final_angles, time_ms=exec_time)
            if right_secondary:
                print("右臂初始化中，请等待5秒钟...")
                time.sleep(5)  # 初始化后强制等待5秒
                print("右臂初始化完成。")

# ==============================================================================

def start_web_server():
    global web_left_controller, web_right_controller
    try:
        web_left_controller = WebMultiServoController(LEFT_ARM_PORT, 115200, ROBOT_CONFIG["SERVO_IDS"], 'left')
        web_right_controller = WebMultiServoController(RIGHT_ARM_PORT, 115200, ROBOT_CONFIG["SERVO_IDS"], 'right')
        print("Starting Web control server on port 8082...")
        web_socketio.run(web_app, host='0.0.0.0', port=8082, debug=False)
    except Exception as e:
        print(f"Error starting Web server: {e}")

def get_vr_server_ip():
    config_file = 'vr_server_config.txt'
    saved_ip = None
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                saved_ip = f.read().strip()
            print(f"发现已保存的VR服务端IP地址: {saved_ip}")
        except Exception as e:
            print(f"读取配置文件失败: {e}")
    if saved_ip:
        print("\n请选择：\n1. 使用已保存的IP地址\n2. 重新输入新的IP地址")
        while True:
            choice = input("请输入选择 (1/2): ").strip()
            if choice == '1': return saved_ip
            elif choice == '2': break
            else: print("请输入有效选择 (1 或 2)")
    while True:
        try:
            new_ip = input("请输入VR服务端IP地址: ").strip()
            parts = new_ip.split('.')
            if len(parts) == 4 and all(0 <= int(part) <= 255 for part in parts):
                try:
                    with open(config_file, 'w') as f: f.write(new_ip)
                    print(f"IP地址已保存到 {config_file}")
                except Exception as e: print(f"保存IP地址失败: {e}")
                return new_ip
            else: print("IP地址格式无效，请重新输入 (例如: 192.168.1.100)")
        except (ValueError, KeyboardInterrupt): print("\n程序中断"); exit(0)

def main(args=None):
    rclpy.init(args=args)
    controller_node = VRArmController()
    
    SERVER_HOST = get_vr_server_ip()
    SERVER_PORT = 12345
    
    print(f"VR控制系统启动，目标服务端: {SERVER_HOST}:{SERVER_PORT}")
    print("操作说明：")
    print(f"- 控制频率: 底盘 {CHASSIS_CONTROL_FREQ}Hz / 机械臂 {ARM_CONTROL_FREQ}Hz (Socket线程直接处理)")
    print("- 安全保护: 双重限位系统 (IK级 + 舵机级) 确保机械臂安全")
    print("- 强制初始化: 连接后必须先按 B/Y 键初始化对应手臂，才能解锁功能。")
    print("- 遥操作: 初始化后，主按键移动手臂，扳机控制夹爪，摇杆控制底盘。")
    print("- 紧急停止: 同时按住右手A+B键，底盘停止且双臂卸力。")
    print("- 按 Ctrl+C 退出程序")
    print("="*50)
    
    retry_count = 0
    max_retry_display = 5
    
    def socket_thread_func():
        nonlocal retry_count
        while rclpy.ok():
            try:
                client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_socket.settimeout(5)
                if retry_count == 0: print(f"正在连接到VR服务端 {SERVER_HOST}:{SERVER_PORT}...")
                elif retry_count < max_retry_display: print(f"重试连接中... (第{retry_count + 1}次)")
                elif retry_count == max_retry_display: print(f"继续等待服务端启动... (后续重试将静默进行)")
                client_socket.connect((SERVER_HOST, SERVER_PORT))
                
                print("✅ VR服务端连接成功！机器人功能已锁定。")
                print("请按手柄 B 或 Y 键，手动初始化机械臂以解锁。")
                retry_count = 0
                
                client_socket.setblocking(False)
                buffer = ""
                # 采用V0.3的数据处理架构：Socket线程直接处理数据
                last_process_time = 0
                arm_control_period = 1.0 / ARM_CONTROL_FREQ

                while rclpy.ok():
                    latest_data_chunk = None
                    while True:
                        try:
                            chunk = client_socket.recv(4096)
                            if not chunk: latest_data_chunk = b''; break
                            latest_data_chunk = chunk
                        except BlockingIOError: break
                    
                    if latest_data_chunk is not None:
                        if not latest_data_chunk:
                            print("⚠️ VR服务端断开连接，等待重新连接...")
                            break
                        buffer += latest_data_chunk.decode('utf-8', errors='ignore')

                    # 采用V0.3架构：直接在Socket线程中处理数据
                    if '\n' in buffer:
                        messages = buffer.split('\n')
                        for msg in reversed(messages):
                            if msg:
                                try: 
                                    latest_vr_data = json.loads(msg)
                                    # V0.3架构：直接处理数据，控制频率
                                    current_time = time.time()
                                    if current_time - last_process_time >= arm_control_period:
                                        try:
                                            controller_node.process_arm_data(latest_vr_data)
                                            last_process_time = current_time
                                        except Exception as e:
                                            print(f"机械臂控制失败: {e}")
                                    break
                                except json.JSONDecodeError: continue
                        buffer = messages[-1] if not buffer.endswith('\n') else ""

                    time.sleep(0.001)
            
            except (socket.timeout, ConnectionRefusedError, ConnectionResetError, BrokenPipeError) as e:
                retry_count += 1
                if isinstance(e, socket.timeout):
                    if retry_count <= max_retry_display: print(f"连接超时，正在重试...")
                elif isinstance(e, ConnectionRefusedError):
                    if retry_count == 1: print(f"等待VR服务端启动...")
                else:
                    if retry_count <= max_retry_display: print(f"连接错误 ({type(e).__name__})，正在重试...")
            except Exception as e:
                retry_count += 1
                if retry_count <= max_retry_display:
                    print(f"连接失败: {e}")
            finally:
                try: client_socket.close()
                except: pass
            
            try: time.sleep(3)
            except KeyboardInterrupt: break
    
    socket_thread = threading.Thread(target=socket_thread_func, daemon=True)
    socket_thread.start()

    # 启动 Web 控制服务器
    web_process = subprocess.Popen(['python3', 'web_control.py'], cwd=os.getcwd())

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        print("\n👋 程序被用户中断，正在退出...")
    finally:
        if web_process:
            web_process.terminate()
            web_process.wait()
        controller_node.destroy_node()
        rclpy.shutdown()
        print("VR控制已停止")

if __name__ == "__main__":
    main()