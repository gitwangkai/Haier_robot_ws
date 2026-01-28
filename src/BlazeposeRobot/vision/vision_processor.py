# vision/vision_processor.py
from .yolo_detector import YoloDetector
from .pose_estimator import PoseEstimator

class VisionProcessor:
    def __init__(self):
        self.yolo = YoloDetector()
        self.pose = PoseEstimator()
    
    def process_frame(self, frame):
        """处理单帧图像，返回检测和姿态结果"""
        bboxes, classes = self.yolo.detect(frame)
        pose_results = self.pose.process(frame)
        return bboxes, classes, pose_results
    
    def draw_results(self, frame, bboxes, classes, pose_results):
        """在图像上绘制结果"""
        if bboxes is not None and classes is not None:
            self.yolo.draw(frame, bboxes, classes)
        if pose_results is not None:
            self.pose.draw(frame, pose_results)