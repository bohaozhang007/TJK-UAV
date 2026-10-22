import sys
from pathlib import Path

# Reserve the binary reply pipe before any third-party imports emit logs.
output = sys.stdout.buffer
sys.stdout = sys.stderr

import numpy as np
from ultralytics import YOLO

from detector_base import Detector, run_detector


CHECKPOINT = Path(__file__).resolve().parents[3] / "yolo/runs/bolt_yolo26l_10-22/weights/best.pt"
IMAGE_SIZE = 1920


class YoloDetector(Detector):
    def __init__(self):
        self.model = YOLO(str(CHECKPOINT))

    def detect(
        self,
        target_img,
        confidence_threshold=0.5,
    ):
        if not 0 <= confidence_threshold <= 1:
            raise ValueError("confidence_threshold must be in [0, 1]")
        result = self.model.predict(
            target_img,
            conf=confidence_threshold,
            imgsz=IMAGE_SIZE,
            device=0,
            verbose=False,
        )[0]
        if len(result.boxes) == 0:
            return []
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        detections = []
        for index in np.argsort(-scores, kind="stable"):
            box = boxes[index]
            # Use the box region for depth sampling; this is not a segmentation mask.
            mask = np.zeros(target_img.shape[:2], dtype=bool)
            x1, y1 = np.floor(box[:2]).astype(int)
            x2, y2 = np.ceil(box[2:]).astype(int)
            mask[y1:y2, x1:x2] = True
            detections.append({
                "box": box.tolist(),
                "confidence": float(scores[index]),
                "mask": mask,
            })
        return detections


run_detector(
    YoloDetector,
    "yolo",
    output,
)
