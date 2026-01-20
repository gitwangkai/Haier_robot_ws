import cv2
import mediapipe as mp

class PoseEstimator:
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils

    def __init__(self):
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

    def process(self, frame):
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        return self.pose.process(image)

    @staticmethod
    def draw(frame, results):
        if results and results.pose_landmarks:
            PoseEstimator.mp_drawing.draw_landmarks(
                frame,
                results.pose_landmarks,
                PoseEstimator.mp_pose.POSE_CONNECTIONS,
                PoseEstimator.mp_drawing.DrawingSpec(
                    color=(245, 117, 66), thickness=2, circle_radius=2
                ),
                PoseEstimator.mp_drawing.DrawingSpec(
                    color=(245, 66, 230), thickness=2, circle_radius=2
                )
            )
