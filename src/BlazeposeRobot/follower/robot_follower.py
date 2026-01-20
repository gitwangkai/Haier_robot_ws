from motion.robot_motion import RobotMotion
from utils.geometry import calculate_angle

class RobotFollower:
    def __init__(self, motion, lost_threshold=10):
        self.motion = motion
        self.lost_counter = 0
        self.lost_threshold = lost_threshold  # 帧数
        self.current_direction = "stop"
        self.lost_counter = 0
        self.lost_threshold = 10  # 超过10帧丢失则停止
        self.min_follow_dist_ratio = 0.7  # 最小跟随距离阈值，当目标高度占屏幕70%时认为太近

    def update(self, bboxes, classes, pose_results, frame_width, frame_height):
        direction = "stop"
        failure = False
        target_found = False

        # ---------- 1. 目标检测与选择判断 ----------
        # 筛选出person的bbox
        person_bboxes = []
        if bboxes is not None and len(bboxes) > 0 and classes is not None:
            for i, cls in enumerate(classes):
                if cls == 0:  # person class
                    person_bboxes.append(bboxes[i])
        
        if len(person_bboxes) > 0:
            # 选择第一个person bbox作为目标
            bx, by, bx2, by2 = person_bboxes[0]
            cx = (bx + bx2) / 2
            norm_x = cx / frame_width
            
            # 距离判断，计算 BBox 高度占屏幕的比例
            bbox_height_ratio = (by2 - by) / frame_height
            
            # 如果距离太近或太高，强制设为 stop，避免碰撞
            if bbox_height_ratio > self.min_follow_dist_ratio:
                direction = "stop"
                is_too_close = True
            else:
                is_too_close = False
                
                # ---------- 2. 方向控制 (Deadzone) ----------
                # 使用死区控制 (范围 0.1) 控制方向
                deadzone = 0.08 
                if norm_x < 0.5 - deadzone:
                    direction = "left"
                elif norm_x > 0.5 + deadzone:
                    direction = "right"
                else:
                    direction = "forward"
            
            target_found = True
            self.lost_counter = 0  # 重置丢失计数
        
        # ---------- 3. Pose 检测逻辑 (用于 Failure 判断) ----------
        if pose_results is not None and pose_results.pose_landmarks:
            lm = pose_results.pose_landmarks.landmark
            # 膝关节角度逻辑，判断摔倒
            lh, lk, la = lm[23], lm[25], lm[27]
            angle_knee = calculate_angle([lh.x, lh.y], [lk.x, lk.y], [la.x, la.y])
            failure = angle_knee < 150

        # ---------- 4. 丢失处理 (超过 10 帧) ----------
        if not target_found:
            self.lost_counter += 1
            if self.lost_counter >= self.lost_threshold:
                direction = "stop"
            else:
                # 在丢失 10 帧以内，保持上一帧的状态前进，简单起见设为 stop
                direction = "stop" 

        # ---------- 5. 执行运动 ----------
        if direction == "forward":
            self.motion.set_forward()
        elif direction == "left":
            self.motion.set_turn_left()
        elif direction == "right":
            self.motion.set_turn_right()
        else:
            self.motion.stop()

        self.motion.update()
        return failure