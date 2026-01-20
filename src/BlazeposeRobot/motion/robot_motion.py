from geometry_msgs.msg import Twist
import time


class PID:
    def __init__(self, Kp, Ki, Kd, integral_limit=1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_limit = integral_limit
        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        self.integral = max(-self.integral_limit,
                             min(self.integral_limit, self.integral))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


class RobotMotion:
    """
    统一的机器人运动控制模块
    - 所有运动都通过 cmd_vel
    - 内部做 PID 平滑
    - 支持 forward / turn / stop
    """

    def __init__(
        self,
        node,
        cmd_vel_topic='/cmd_vel',
        max_linear=0.25,
        max_angular=0.6,
        control_dt=0.05
    ):
        self.node = node
        self.publisher = node.create_publisher(Twist, cmd_vel_topic, 10)

        self.max_linear = max_linear
        self.max_angular = max_angular
        self.dt = control_dt

        # 当前 & 目标速度
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0

        # PID 只用于线速度
        self.pid_linear = PID(Kp=0.25, Ki=0.05, Kd=0.005)

        self.last_cmd_time = time.time()

    # =====================
    # 对外接口（main 只调这些）
    # =====================

    def set_forward(self, speed_ratio=1.0):
        self.target_linear = self.max_linear * speed_ratio
        self.target_angular = 0.0
        self.last_cmd_time = time.time()

    def set_turn_left(self, speed_ratio=1.0):
        self.target_linear = 0.0
        self.target_angular = self.max_angular * speed_ratio
        self.last_cmd_time = time.time()

    def set_turn_right(self, speed_ratio=1.0):
        self.target_linear = 0.0
        self.target_angular = -self.max_angular * speed_ratio
        self.last_cmd_time = time.time()

    def stop(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.pid_linear.reset()
        self.publish_twist(0.0, 0.0)

    # =====================
    # 主更新函数（每帧调用）
    # =====================

    def update(self):
        """
        main.py 每一帧调用一次
        """
        error = self.target_linear - self.current_linear
        adjust = self.pid_linear.update(error, self.dt)
        self.current_linear += adjust

        self.current_linear = max(
            -self.max_linear,
            min(self.max_linear, self.current_linear)
        )

        # 角速度直接跟随目标
        self.current_angular = self.target_angular

        self.publish_twist(self.current_linear, self.current_angular)

    def publish_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)

    # =====================
    # 无人 / 超时保护
    # =====================

    def check_timeout(self, timeout=0.5):
        """
        超过 timeout 秒未收到指令，自动 STOP
        """
        if time.time() - self.last_cmd_time > timeout:
            self.stop()
