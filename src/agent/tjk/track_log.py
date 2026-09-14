"""Background TRACK artifacts, bound to the actual input exposure."""
import json
from pathlib import Path
import queue
import threading

import cv2
import numpy as np


class TrackImageLog:
    def __init__(self, event):
        self.event = event
        self.queue = queue.SimpleQueue()
        self.thread = None
        self.sequence = 0

    def submit(self, obs, bbox, mask, *, directory, source="track", depth=None):
        if self.thread is None:
            self.thread = threading.Thread(target=self._run, name="track-image-log", daemon=True)
            self.thread.start()
        self.sequence += 1
        name = f"track_{self.sequence:06d}_{round(obs.timestamp_s*1000)}"
        meta = dict(frame_id=obs.frame_id, timestamp_s=obs.timestamp_s,
                    pose=dict(obs.pose), localization_epoch=obs.localization_epoch,
                    source=source, box=np.asarray(bbox).tolist())
        # Snapshot arrays only. Encoding, rendering and all disk IO run off-thread.
        self.queue.put((Path(directory), name, obs.rgb.copy(),
                        np.asarray(mask, dtype=bool).copy(),
                        None if depth is None else np.asarray(depth).copy(), meta))

    def _run(self):
        while True:
            item = self.queue.get()
            if item is None:
                return
            directory, name, rgb, mask, depth, meta = item
            try:
                directory.mkdir(parents=True, exist_ok=True)
                mask = mask.squeeze()
                if mask.shape != rgb.shape[:2]:
                    raise ValueError("TRACK mask shape does not match input image")
                # Plain row-major RLE, alternating background/foreground runs,
                # beginning with a background run (possibly length zero).
                flat = mask.ravel().astype(np.uint8)
                edges = np.flatnonzero(np.diff(np.r_[0, flat, 0]))
                counts = np.diff(np.r_[0, edges, flat.size]).tolist()
                meta["mask"] = dict(size=list(mask.shape), order="C", encoding="rle", counts=counts)
                image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                image[mask] = (image[mask]*0.6 + np.array([0,255,0])*0.4).astype(np.uint8)
                x1,y1,x2,y2 = np.rint(meta["box"]).astype(int).reshape(-1)
                cv2.rectangle(image, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.rectangle(image, (0,0), (100,32), (0,0,0), -1)
                cv2.putText(image, "TRACK", (8,23), cv2.FONT_HERSHEY_SIMPLEX,
                            .7, (0,255,255), 2, cv2.LINE_AA)
                meta["image_file"] = name + ".png"
                if not cv2.imwrite(str(directory/meta["image_file"]), image):
                    raise OSError("TRACK image write failed")
                if depth is not None:
                    meta["depth_file"] = name + "_depth.npy"
                    meta["depth_unit"] = "cm"
                    np.save(directory/meta["depth_file"], depth)
                (directory/(name+".json")).write_text(json.dumps(meta, indent=2), encoding="utf-8")
            except Exception as exc:
                self.event("track_image_save_failed", name=name, error=str(exc))

    def close(self):
        if self.thread is not None:
            self.queue.put(None)
            self.thread.join()
            self.thread = None
