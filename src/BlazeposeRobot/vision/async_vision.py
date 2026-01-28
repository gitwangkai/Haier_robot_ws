# vision/async_vision.py
import threading
import time
from .vision_processor import VisionProcessor

class AsyncVision(VisionProcessor):
    def __init__(self):
        super().__init__()
        self.latest_frame = None
        self.latest_bboxes = []
        self.latest_classes = []
        self.latest_pose = None
        self.lock = threading.Lock()
        self.running = False

    def start(self):
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self.running = False

    def update_frame(self, frame):
        with self.lock:
            self.latest_frame = frame.copy()

    def get_results(self):
        with self.lock:
            return self.latest_bboxes, self.latest_classes, self.latest_pose

    def _loop(self):
        while self.running:
            if self.latest_frame is None:
                time.sleep(0.001)
                continue
            frame = None
            with self.lock:
                frame = self.latest_frame.copy()
            bboxes, classes, pose_results = self.process_frame(frame)
            with self.lock:
                self.latest_bboxes = bboxes
                self.latest_classes = classes
                self.latest_pose = pose_results