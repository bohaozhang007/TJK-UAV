import queue
import threading
from datetime import datetime
from pathlib import Path

import cv2


SAVE_DIR = Path(__file__).resolve().parents[2] / "logs"
JPEG_QUALITY = 95
SAVE_QUEUE_SIZE = 10


class ImageWriter:
    def __init__(
        self,
        kind="detector",
        timestamp=None,
    ):
        if timestamp is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.save_dir = SAVE_DIR / timestamp / kind
        self.original_dir = self.save_dir / "original"
        self.annotation_dir = self.save_dir / "annotation"
        self.original_dir.mkdir(parents=True, exist_ok=True)
        self.annotation_dir.mkdir(parents=True, exist_ok=True)
        # Block submission when full to bound buffered images without dropping them.
        self.queue = queue.Queue(maxsize=SAVE_QUEUE_SIZE)
        self.worker = threading.Thread(target=self.run)
        self.worker.start()

    def submit(
        self,
        original_img,
        img,
        detections,
        target_id=None,
    ):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"img_{timestamp}.jpg" if target_id is None else f"target_{target_id}_{timestamp}.jpg"
        self.queue.put((original_img, img, detections, filename))

    def run(self):
        while True:
            item = self.queue.get()
            if item is None:
                return
            original_img, img, detections, filename = item
            self.save_image(
                original_img,
                self.original_dir / filename,
            )
            try:
                img = img.copy()
                for detection in detections:
                    box = detection["box"]
                    mask = detection.get("mask")
                    if mask is not None:
                        img[mask] = img[mask] * 0.6 + (0, 100, 0)
                    confidence = detection.get("confidence")
                    label = "target" if confidence is None else f"{confidence:.3f}"
                    distance = detection.get("distance_m")
                    if distance is not None:
                        label += f"-{distance:.2f}m"
                    x1, y1, x2, y2 = [round(value) for value in box]
                    cv2.rectangle(
                        img,
                        (x1, y1),
                        (x2, y2),
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        img,
                        label,
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                self.save_image(
                    img,
                    self.annotation_dir / filename,
                )
            except Exception as exc:
                print(f"Image save failed: {self.annotation_dir / filename}: {exc}", flush=True)

    def save_image(
        self,
        img,
        path,
    ):
        try:
            ok, encoded = cv2.imencode(
                ".jpg",
                img,
                [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
            )
            if not ok:
                raise RuntimeError("Failed to encode saved image")
            encoded.tofile(path)
        except Exception as exc:
            print(f"Image save failed: {path}: {exc}", flush=True)

    def close(self):
        self.queue.put(None)
        self.worker.join()
