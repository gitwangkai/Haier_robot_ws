import cv2
import numpy as np
from ultralytics import YOLO

class YoloDetector:
    def __init__(self, model_path="yolov8n.pt"):
        self.model = YOLO(model_path)

    def detect(self, frame):
        results = self.model(frame)[0]
        bboxes = np.array(results.boxes.xyxy.cpu(), dtype=int)
        classes = np.array(results.boxes.cls.cpu(), dtype=int)
        return bboxes, classes

    def draw(self, frame, bboxes, classes):
        for box, cls in zip(bboxes, classes):
            if cls != 0:
                continue
            x1, y1, x2, y2 = box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, "Person", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
