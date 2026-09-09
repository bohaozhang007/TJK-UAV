"""Model-independent v21 perception scheduling and spatial target memory."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, asdict
import threading
import time

import numpy as np


class TargetGeometryError(ValueError):
    pass


def target_position(observation, depth_cm, box, *, mask=None,
                    min_pixels=16, max_relative_mad=0.3, core_ratio=0.5):
    """Robust visible-surface position, returned in the existing public XYZ cm.

    DA3 depth is optical Z. Use foreground mask when available, otherwise the
    central fraction of the detection box (an approximation, logged by caller).
    world_from_camera is ROS ENU; public world reflects its Y axis.
    """
    depth = np.asarray(depth_cm)
    h, w = observation.rgb.shape[:2]
    if depth.shape != (h, w):
        raise TargetGeometryError("Depth and observation must have identical dimensions")
    b = np.asarray(box, dtype=float).reshape(-1)
    if b.size != 4 or not np.isfinite(b).all():
        raise TargetGeometryError("Invalid target box")
    x1, y1, x2, y2 = b
    if not (0 <= x1 < x2 <= w and 0 <= y1 < y2 <= h):
        raise TargetGeometryError("Target box is outside image")
    if mask is None:
        cx, cy = (x1+x2)/2, (y1+y2)/2
        rw, rh = (x2-x1)*core_ratio/2, (y2-y1)*core_ratio/2
        roi = np.zeros((h, w), dtype=bool)
        roi[int(np.ceil(cy-rh)):int(np.ceil(cy+rh)),
            int(np.ceil(cx-rw)):int(np.ceil(cx+rw))] = True
    else:
        roi = np.asarray(mask, dtype=bool).squeeze()
        if roi.shape != (h, w):
            raise TargetGeometryError("Invalid target mask dimensions")
    roi = roi & np.isfinite(depth) & (depth > 0)
    ys, xs = np.where(roi)
    if len(xs) < min_pixels:
        raise TargetGeometryError("Insufficient valid target depth pixels")
    depths = depth[ys, xs]
    median = float(np.median(depths))
    mad = float(np.median(np.abs(depths - median)))
    if mad / median > max_relative_mad:
        raise TargetGeometryError("Target depth is too dispersed")
    keep = np.abs(depths - median) <= max(3 * mad, median * 0.05)
    if np.count_nonzero(keep) < min_pixels:
        raise TargetGeometryError("Insufficient target depth inliers")
    rays = np.linalg.solve(observation.intrinsics,
                           np.stack([xs[keep], ys[keep], np.ones(np.count_nonzero(keep))]))
    camera = rays * depths[keep]
    world = observation.world_from_camera[:3, :3] @ camera
    world += observation.world_from_camera[:3, 3:4]
    xyz = np.median(world, axis=1)
    xyz[1] *= -1
    return xyz


@dataclass
class TargetRecord:
    target_id: int
    position_cm: list
    status: str = "pending"
    attempts: int = 1
    retry_after: float = 0.0


class TargetMemory:
    """One mission/localization epoch only; failures have bounded retries."""
    def __init__(self, distance_cm=100.0, retry_cooldown_s=30.0, max_attempts=2):
        self.distance_cm = distance_cm
        self.retry_cooldown_s = retry_cooldown_s
        self.max_attempts = max_attempts
        self.records = []

    def nearest(self, position):
        if not self.records:
            return None
        record = min(self.records, key=lambda r: np.linalg.norm(np.asarray(r.position_cm)-position))
        return record if np.linalg.norm(np.asarray(record.position_cm)-position) < self.distance_cm else None

    def claim(self, position, now=None):
        now = time.monotonic() if now is None else now
        existing = self.nearest(position)
        if existing:
            if (existing.status != "failed" or existing.attempts >= self.max_attempts
                    or now < existing.retry_after):
                return None
            existing.status = "pending"
            existing.attempts += 1
            return existing
        record = TargetRecord(len(self.records)+1, np.asarray(position).tolist())
        self.records.append(record)
        return record

    def finish(self, record, success, position=None, now=None):
        now = time.monotonic() if now is None else now
        record.status = "completed" if success else "failed"
        record.retry_after = now + self.retry_cooldown_s
        if position is not None:
            record.position_cm = np.asarray(position).tolist()

    def snapshot(self):
        return [asdict(r) for r in self.records]


class PerceptionPipeline:
    """One capture worker + one inference worker, with a latest-only input slot.

    pause(drain=False) invalidates in-flight results immediately. wait_idle must
    complete before main-thread detector/tracker use. drain=True processes the
    latest queued frame at route arrival so final views aren't silently lost.
    """
    def __init__(self, capture, infer, *, capture_fps=5.0, det_interval=1):
        self.capture, self.infer = capture, infer
        self.period = 1.0 / capture_fps
        self.det_interval = det_interval
        self.cv = threading.Condition()
        self.enabled = False
        self.closed = False
        self.generation = 0
        self.segment = None
        self.slot = None
        self.busy = False
        self.capture_busy = False
        self.error = None
        self.results = deque(maxlen=8)
        self.stats = {"captured": 0, "interval_skipped": 0, "overwritten": 0,
                      "inferred": 0, "stale_results": 0, "result_overflow": 0}
        self._counter = 0
        self._last_frame = None
        self.threads = [threading.Thread(target=self._capture_loop, name="v21-capture", daemon=True),
                        threading.Thread(target=self._infer_loop, name="v21-detect", daemon=True)]
        for thread in self.threads:
            thread.start()

    def resume(self, segment):
        with self.cv:
            if self.busy or self.capture_busy:
                raise RuntimeError("Perception must be idle before resume")
            self.generation += 1
            self.segment = segment
            self.slot = None
            self.results.clear()
            self._counter = 0
            self.enabled = True
            self.cv.notify_all()

    def pause(self, *, drain=False):
        with self.cv:
            self.enabled = False
            if not drain:
                self.generation += 1
                self.slot = None
                self.results.clear()
            self.cv.notify_all()

    def wait_idle(self, timeout_s):
        with self.cv:
            if not self.cv.wait_for(lambda: not self.busy and not self.capture_busy
                                    and self.slot is None, timeout=timeout_s):
                raise RuntimeError("Perception worker did not become idle")
            self.raise_error()

    def raise_error(self):
        if self.error:
            raise RuntimeError(f"Perception worker failed: {self.error}") from self.error

    def pop(self):
        with self.cv:
            self.raise_error()
            return self.results.popleft() if self.results else None

    def _fail(self, exc):
        with self.cv:
            self.error = exc
            self.enabled = False
            self.slot = None
            self.cv.notify_all()

    def _capture_loop(self):
        while True:
            with self.cv:
                self.cv.wait_for(lambda: self.closed or self.enabled)
                if self.closed:
                    return
                generation, segment = self.generation, self.segment
                self.capture_busy = True
            start = time.monotonic()
            try:
                obs = self.capture()
                with self.cv:
                    if generation == self.generation and self.enabled:
                        identity = (obs.localization_epoch, obs.frame_id)
                        if identity != self._last_frame:
                            self._last_frame = identity
                            self.stats["captured"] += 1
                            self._counter += 1
                            if (self._counter-1) % self.det_interval == 0:
                                if self.slot is not None:
                                    self.stats["overwritten"] += 1
                                self.slot = (generation, segment, obs)
                                self.cv.notify_all()
                            else:
                                self.stats["interval_skipped"] += 1
            except Exception as exc:
                self._fail(exc)
            finally:
                with self.cv:
                    self.capture_busy = False
                    self.cv.notify_all()
            with self.cv:
                self.cv.wait_for(lambda: self.closed or not self.enabled,
                                 timeout=max(0, self.period-(time.monotonic()-start)))

    def _infer_loop(self):
        while True:
            with self.cv:
                self.cv.wait_for(lambda: self.closed or self.slot is not None)
                if self.closed:
                    return
                generation, segment, obs = self.slot
                self.slot = None
                self.busy = True
            try:
                value = self.infer(obs)
                with self.cv:
                    self.stats["inferred"] += 1
                    if generation == self.generation:
                        if value:
                            if len(self.results) == self.results.maxlen:
                                self.stats["result_overflow"] += 1
                            self.results.append((segment, obs, value))
                    else:
                        self.stats["stale_results"] += 1
            except Exception as exc:
                self._fail(exc)
            finally:
                with self.cv:
                    self.busy = False
                    self.cv.notify_all()

    def close(self):
        with self.cv:
            self.closed = True
            self.enabled = False
            self.generation += 1
            self.slot = None
            self.cv.notify_all()
        for thread in self.threads:
            thread.join(timeout=3)
