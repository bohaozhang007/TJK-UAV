import sys
from pathlib import Path

# Reserve the binary reply pipe before any third-party imports emit logs.
output = sys.stdout.buffer
sys.stdout = sys.stderr

import numpy as np
from ultralytics import YOLO

from detector_base import Detector, run_detector


CHECKPOINT = Path(__file__).resolve().parents[3] / "yolo/runs/bolt_yolo26l_seg_like_yolo11l/weights/best.pt"
IMAGE_SIZE = 1920


class YoloDetector(Detector):
    def __init__(self):
        self.model = YOLO(str(CHECKPOINT))
        if self.model.task != "segment":
            raise ValueError("YOLO checkpoint must support instance segmentation")

    def detect(
        self,
        target_img,
        confidence_threshold=0.5,
        iou_threshold=0.7,
    ):
        if (
            not 0 <= confidence_threshold <= 1
            or not 0 < iou_threshold <= 1
        ):
            raise ValueError("confidence_threshold must be in [0, 1] and iou_threshold in (0, 1]")
        result = self.model.predict(
            target_img,
            conf=confidence_threshold,
            iou=iou_threshold,
            imgsz=IMAGE_SIZE,
            device=0,
            retina_masks=True,
            verbose=False,
        )[0]
        if len(result.boxes) == 0:
            return []
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        masks = result.masks.data.cpu().numpy().astype(bool)
        detections = []
        for index in np.argsort(-scores, kind="stable"):
            box = boxes[index]
            detections.append({
                "box": box.tolist(),
                "confidence": float(scores[index]),
                "mask": masks[index],
            })
        return detections


run_detector(
    YoloDetector,
    "yolo",
    output,
)
