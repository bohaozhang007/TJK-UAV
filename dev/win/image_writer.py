import queue
import threading
import time
from pathlib import Path

import cv2


SAVE_DIR = Path(__file__).resolve().parents[1] / "received_images"
JPEG_QUALITY = 95


class ImageWriter:
    def __init__(self):
        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        # Keep every decoded image without blocking requests on disk writes.
        self.queue = queue.Queue()
        self.worker = threading.Thread(target=self.run)
        self.worker.start()

    def submit(
        self,
        img,
        detections,
    ):
        self.queue.put((img, detections))

    def run(self):
        while True:
            item = self.queue.get()
            if item is None:
                return
            try:
                img, detections = item
                img = img.copy()
                for box, confidence in detections:
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
                        f"{confidence:.3f}",
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
                encoded.tofile(SAVE_DIR / f"{time.time_ns()}.jpg")
            except Exception as exc:
                print(f"Image save failed: {exc}", flush=True)

    def close(self):
        self.queue.put(None)
        self.worker.join()
