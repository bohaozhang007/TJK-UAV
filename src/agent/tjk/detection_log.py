"""Asynchronous per-inference image log; never changes model inputs."""
import json
from pathlib import Path
import queue
import threading

import cv2
import numpy as np


class DetectionImageLog:
    def __init__(self, directory, event):
        self.directory = Path(directory)
        self.event = event
        self.queue = queue.Queue(maxsize=64)
        self.thread = None
        self.sequence = 0

    def submit(self, rgb, detections, *, phase, frame_id, timestamp_s, pose, error=None, directory=None):
        if self.thread is None:
            self.thread = threading.Thread(target=self._run, name="detection-image-log", daemon=True)
            self.thread.start()
        self.sequence += 1
        name = f"{phase}_{self.sequence:06d}_{round(timestamp_s*1000)}"
        top = sorted(detections, key=lambda d: float(d["confidence"]), reverse=True)[:3]
        top = [dict(box=np.asarray(d["box"]).tolist(), confidence=float(d["confidence"])) for d in top]
        meta = dict(phase=phase, frame_id=frame_id, timestamp_s=timestamp_s,
                    pose=dict(pose), detection_count=len(detections), top3=top, error=error)
        # Bounded queue with backpressure, not silent dropping of detector inputs.
        self.queue.put((name, rgb.copy(), meta, Path(directory) if directory is not None else self.directory))
        return name

    def mark_trigger(self, name, directory=None):
        # The same FIFO worker handles initial save and later trigger annotation.
        self.queue.put((name, None, None, Path(directory) if directory is not None else self.directory))

    def _run(self):
        while True:
            item = self.queue.get()
            try:
                if item is None:
                    return
                name, rgb, meta, directory = item
                directory.mkdir(parents=True, exist_ok=True)
                if rgb is None:
                    path = directory / f"{name}_top3.png"
                    trigger_path = directory / f"{name}_top3_trigger.png"
                    if not path.exists():
                        path = trigger_path
                    annotated = cv2.imread(str(path))
                    if annotated is None:
                        raise OSError(f"Cannot mark missing detection image {path}")
                    cv2.rectangle(annotated, (0,0), (100,30), (0,0,0), -1)
                    cv2.putText(annotated, "trigger", (8,22), cv2.FONT_HERSHEY_SIMPLEX,
                                .7, (0,255,255), 2, cv2.LINE_AA)
                    if not cv2.imwrite(str(path), annotated):
                        raise OSError(f"Failed to save {path}")
                    if path != trigger_path:
                        path.replace(trigger_path)
                    metadata_path = directory / f"{name}.json"
                    meta = json.loads(metadata_path.read_text(encoding="utf-8"))
                    meta["trigger"] = True
                    meta["image_file"] = trigger_path.name
                    metadata_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
                    continue
                original = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                annotated = original.copy()
                for rank, result in enumerate(meta["top3"], 1):
                    x1,y1,x2,y2 = np.rint(result["box"]).astype(int)
                    cv2.rectangle(annotated, (x1,y1), (x2,y2), (0,255,0), 2)
                    cv2.putText(annotated, f"#{rank} conf={result['confidence']:.3f}",
                                (max(0,x1), max(15,y1-5)), cv2.FONT_HERSHEY_SIMPLEX,
                                .5, (0,255,0), 1, cv2.LINE_AA)
                if not meta["top3"]:
                    cv2.putText(annotated, "DETECTION ERROR" if meta["error"] else "NO DETECTIONS",
                                (8,22), cv2.FONT_HERSHEY_SIMPLEX, .6, (0,255,255), 2)
                path = directory / f"{name}_top3.png"
                if not cv2.imwrite(str(path), annotated):
                    raise OSError(f"Failed to save {path}")
                (directory / f"{name}.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
            except Exception as exc:
                self.event("detection_image_save_failed", name=item[0], error=str(exc))
            finally:
                self.queue.task_done()

    def close(self):
        if self.thread is not None:
            self.queue.put(None)
            self.thread.join()
            self.thread = None
