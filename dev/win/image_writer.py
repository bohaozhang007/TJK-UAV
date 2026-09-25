import queue
import threading
from datetime import datetime
from pathlib import Path

import cv2


SAVE_DIR = Path(__file__).resolve().parents[2] / "logs"
JPEG_QUALITY = 95


class ImageWriter:
    def __init__(
        self,
        kind="detector",
        timestamp=None,
    ):
        if timestamp is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.save_dir = SAVE_DIR / timestamp / kind
        self.save_dir.mkdir(parents=True, exist_ok=True)
        # Keep every decoded image without blocking requests on disk writes.
        self.queue = queue.Queue()
        self.worker = threading.Thread(target=self.run)
        self.worker.start()

    def submit(
        self,
        img,
        detections,
    ):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.queue.put((img, detections, timestamp))

    def run(self):
        while True:
            item = self.queue.get()
            if item is None:
                return
            try:
                img, detections, timestamp = item
                img = img.copy()
                for detection in detections:
                    box = detection["box"]
                    mask = detection.get("mask")
                    if mask is not None:
                        img[mask] = img[mask] * 0.6 + (0, 100, 0)
                    confidence = detection.get("confidence")
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
                        "target" if confidence is None else f"{confidence:.3f}",
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                ok, encoded = cv2.imencode(
                    ".jpg",
                    img,
                    [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
                )
                if not ok:
                    raise RuntimeError("Failed to encode saved image")
                encoded.tofile(self.save_dir / f"img_{timestamp}.jpg")
            except Exception as exc:
                print(f"Image save failed: {exc}", flush=True)

    def close(self):
        self.queue.put(None)
        self.worker.join()
