"""Save detection overlays outside the HTTP worker."""
import logging
from datetime import datetime, timedelta
from queue import SimpleQueue
from threading import Thread

import numpy as np
from PIL import Image, ImageDraw


COLORS = ((255, 80, 80), (50, 220, 100), (70, 150, 255))


def save_frame(path, image, detections):
    pixels = image.copy()
    for index, item in enumerate(detections):
        mask = item["mask"]
        color = np.array(COLORS[index % len(COLORS)], dtype=np.float32)
        pixels[mask] = (pixels[mask] * 0.65 + color * 0.35).astype(np.uint8)
    canvas = Image.fromarray(pixels)
    draw = ImageDraw.Draw(canvas)
    for index, item in enumerate(detections):
        color = COLORS[index % len(COLORS)]
        x1, y1, x2, y2 = item["box"]
        draw.rectangle((x1, y1, x2, y2), outline=color, width=3)
        label = item.get("label") or f"#{index + 1} {item['confidence']:.3f}"
        left, top, right, bottom = draw.textbbox((0, 0), label)
        width, height = right - left + 8, bottom - top + 8
        x = max(0, min(int(x1), canvas.width - width))
        y = max(0, min(int(y1) - height, canvas.height - height))
        draw.rectangle((x, y, x + width, y + height), fill=color)
        draw.text((x + 4 - left, y + 4 - top), label, fill=(0, 0, 0))
    canvas.save(path, format="JPEG", quality=95)


class ImageLog:
    def __init__(self, directory):
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=False)
        self._queue = SimpleQueue()
        self._last_timestamp = datetime.min
        self._worker = Thread(target=self._run, name="detector-image-log", daemon=True)
        self._worker.start()

    def submit(self, image, detections, prefix=""):
        # Transfer ownership; callers must not mutate these arrays afterwards.
        timestamp = max(datetime.now(), self._last_timestamp + timedelta(microseconds=1))
        self._last_timestamp = timestamp
        filename = prefix + timestamp.strftime("%Y%m%d_%H%M%S_%f") + ".jpg"
        self._queue.put((filename, image, detections))

    def close(self):
        self._queue.put(None)
        self._worker.join()

    def _run(self):
        while True:
            job = self._queue.get()
            if job is None:
                return
            filename, image, detections = job
            path = self.directory / filename
            try:
                save_frame(path, image, detections)
            except Exception:
                logging.exception("Failed to save visualization: %s", path)
