from vision.yolo_detector import YoloDetector
from vision.pose_estimator import PoseEstimator
from utils.geometry import check_element

class VisionManager:
    def __init__(self):
        self.yolo = YoloDetector()
        self.pose = PoseEstimator()
        self.current_bbox = None

    def process(self, frame, resolution):
        bboxes, classes = self.yolo.detect(frame)

        if check_element(classes, 0):
            for cls, box in zip(classes, bboxes):
                if cls == 0:
                    self.current_bbox = box
                    break
        else:
            self.current_bbox = None

        pose_data = self.pose.process(frame)

        if pose_data:
            return pose_data["mid_shoulder_x"]

        if self.current_bbox is not None:
            x1, _, x2, _ = self.current_bbox
            return ((x1 + x2) / 2.0) / resolution[0]

        return None
