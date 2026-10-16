import os
import queue
import threading
from datetime import datetime, timedelta
from pathlib import Path

import cv2


SAVE_DIR = Path(__file__).resolve().parents[2] / "logs"
JPEG_QUALITY = 95
SAVE_QUEUE_SIZE = 10


class ImageWriter:
    def __init__(
        self,
        kind="all",
        timestamp=None,
    ):
        if timestamp is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.save_dir = SAVE_DIR / timestamp / kind
        self.original_dir = self.save_dir / "original"
        self.annotation_dir = self.save_dir / "annotation"
        self.tracker_dir = self.save_dir.parent / "tracker"
        self.last_timestamp = datetime.min
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
        frame_id=None,
    ):
        self.queue.put((original_img, img, detections, target_id, frame_id))

    def run(self):
        while True:
            item = self.queue.get()
            if item is None:
                return
            original_img, img, detections, target_id, frame_id = item
            # Generate strictly increasing names in queue order, even on coarse clocks.
            self.last_timestamp = max(datetime.now(), self.last_timestamp + timedelta(microseconds=1))
            timestamp = self.last_timestamp.strftime("%Y%m%d_%H%M%S_%f")
            filename = f"img_{timestamp}.jpg"
            link_name = None
            if target_id is not None:
                prefix = f"target_{target_id}" if frame_id is None else f"target_{target_id}_f_{frame_id}"
                link_name = f"{prefix}_{timestamp}.jpg"
            self.save_image(
                original_img,
                self.original_dir / filename,
                link_name=link_name,
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
                    link_name=link_name,
                )
            except Exception as exc:
                print(f"Image save failed: {self.annotation_dir / filename}: {exc}", flush=True)

    def save_image(
        self,
        img,
        path,
        link_name=None,
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
            return
        if link_name is not None:
            link = self.tracker_dir / path.parent.name / link_name
            try:
                link.parent.mkdir(parents=True, exist_ok=True)
                link.symlink_to(os.path.relpath(path, link.parent))
            except OSError as exc:
                print(f"WARNING: Tracker symlink failed: {link}: {exc}", flush=True)

    def close(self):
        self.queue.put(None)
        self.worker.join()
