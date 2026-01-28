# vision/config.py
CONFIG = {
    "yolo": {
        "model_path": "yolov8n.pt",
        "confidence_threshold": 0.5
    },
    "pose": {
        "min_detection_confidence": 0.5,
        "min_tracking_confidence": 0.5
    }
}