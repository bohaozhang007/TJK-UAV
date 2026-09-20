"""Visual feedback framing for a textured box; this does not change lens focus."""

from __future__ import annotations

import math
import time

import cv2
import numpy as np


def validate_image_box(img, box):
    if not isinstance(img, np.ndarray) or img.dtype != np.uint8 or img.ndim != 3 or img.shape[2] != 3:
        raise ValueError("img must be a uint8 HxWx3 BGR image")
    height, width = img.shape[:2]
    bounds = np.asarray(box, dtype=float)
    if bounds.shape != (4,) or not np.isfinite(bounds).all():
        raise ValueError("box must contain four finite pixel coordinates: x1,y1,x2,y2")
    x1, y1, x2, y2 = bounds
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        raise ValueError("box must lie inside img")
    if min(x2 - x1, y2 - y1) < 24:
        raise ValueError("box must be at least 24 pixels wide and high for tracking")
    return bounds


class BoxTracker:
    """Match features inside the previous target; reject weak/ambiguous motion."""

    def __init__(self, img, box):
        self.box = validate_image_box(img, box)
        self.shape = img.shape[:2]
        self.orb = cv2.ORB_create(nfeatures=2000, edgeThreshold=8, fastThreshold=10)
        self._set_reference(img)

    def _gray(self, img):
        if img.shape[:2] != self.shape:
            raise RuntimeError("Tracking image dimensions changed")
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def _set_reference(self, img):
        gray = self._gray(img)
        mask = np.zeros(gray.shape, np.uint8)
        x1, y1, x2, y2 = np.round(self.box).astype(int)
        mask[y1:y2, x1:x2] = 255
        self.points, self.desc = self.orb.detectAndCompute(gray, mask)
        if self.desc is None or len(self.points) < 12:
            raise RuntimeError("Target has too few visual features; use a larger textured box")

    def update(self, img):
        points, desc = self.orb.detectAndCompute(self._gray(img), None)
        if desc is None or len(desc) < 12:
            raise RuntimeError("Target lost: insufficient features in new image")
        pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(self.desc, desc, k=2)
        matches = [pair[0] for pair in pairs if len(pair) == 2 and pair[0].distance < .7 * pair[1].distance]
        # One destination feature must not explain several reference features.
        unique = {m.trainIdx: m for m in sorted(matches, key=lambda m: -m.distance)}
        matches = list(unique.values())
        if len(matches) < 10:
            raise RuntimeError("Target lost: too few reliable feature matches")
        src = np.float32([self.points[m.queryIdx].pt for m in matches])
        dst = np.float32([points[m.trainIdx].pt for m in matches])
        affine, inliers = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC, ransacReprojThreshold=3)
        if affine is None or inliers is None or inliers.sum() < 8 or inliers.mean() < .55:
            raise RuntimeError("Target lost: inconsistent feature motion")
        scale = float(np.linalg.norm(affine[:, 0]))
        if not .6 <= scale <= 1.6:
            raise RuntimeError("Target lost: implausible scale change")
        support = np.ptp(src[inliers.ravel().astype(bool)], axis=0)
        if np.any(support < (self.box[2:] - self.box[:2]) * .15):
            raise RuntimeError("Target lost: matches cover too little of the box")
        x1, y1, x2, y2 = self.box
        corners = np.float32([[[x1, y1], [x2, y1], [x2, y2], [x1, y2]]])
        transformed = cv2.transform(corners, affine)[0]
        box = np.r_[transformed.min(axis=0), transformed.max(axis=0)]
        h, w = self.shape
        if box[0] < 0 or box[1] < 0 or box[2] > w or box[3] > h:
            raise RuntimeError("Target reaches image boundary; stop to avoid losing it")
        self.box = box
        self._set_reference(img)
        return box.copy()


def autofocus_box(controller, img, box, capture, *, target_ratio=.4,
                  center_tolerance=.04, size_tolerance=.05,
                  max_steps=30, timeout_s=120.0, settle_s=.5,
                  tracker_factory=BoxTracker):
    """Center and size a box, using fresh BGR frames and relative camera commands.

    capture(after_monotonic, timeout_s) must return a newly received frame.
    Coordinates in the result use the supplied img's pixel dimensions.
    Camera commands are never retried automatically after a failure.
    """
    bounds = validate_image_box(img, box)
    for name, value, low, high in (
        ('target_ratio', target_ratio, .1, .8),
        ('center_tolerance', center_tolerance, .005, .1),
        ('size_tolerance', size_tolerance, .005, .1),
        ('timeout_s', timeout_s, 1, 600), ('settle_s', settle_s, .1, 5),
    ):
        if isinstance(value, bool) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{name} must be within [{low}, {high}]')
    if isinstance(max_steps, bool) or not isinstance(max_steps, int) or not 1 <= max_steps <= 100:
        raise ValueError('max_steps must be an integer from 1 to 100')
    h, w = img.shape[:2]
    deadline = time.monotonic() + timeout_s
    history = []
    slopes = {}
    pending = None
    stable = 0
    ratio = None
    center = None
    zoom = None
    box_confirmed = False

    def result(ok, message):
        return dict(ok=ok, message=message, box=bounds.tolist(), box_confirmed=box_confirmed,
                    image_width=w, image_height=h, occupancy=ratio,
                    target_ratio=target_ratio, center_error=center, zoom=zoom,
                    actions=history)

    try:
        tracker = tracker_factory(img, bounds)
        zoom = float(controller.get_zoom()['zoom'])
        for _ in range(max_steps + 3):
            remaining = deadline - time.monotonic()
            if remaining <= settle_s:
                return result(False, 'Automatic framing timed out')
            box_confirmed = False
            frame = capture(time.monotonic() + settle_s, min(3.0, remaining))
            if frame.shape[:2] != (h, w):
                raise RuntimeError('Live image dimensions differ from img; pass a native-resolution image')
            bounds = tracker.update(frame)
            box_confirmed = True
            center = [float((bounds[0] + bounds[2]) / (2 * w) - .5),
                      float((bounds[1] + bounds[3]) / (2 * h) - .5)]
            ratio = float(max((bounds[2] - bounds[0]) / w, (bounds[3] - bounds[1]) / h))
            if pending is not None:
                axis, old_error, start_angle = pending
                pending = None
                key = 'yaw_deg' if axis == 0 else 'pitch_deg'
                actual_delta = controller.get_gimbal()[key] - start_angle
                if abs(actual_delta) >= .02:
                    slope = (center[axis] - old_error) / actual_delta
                    # Yaw right moves target left; pitch up moves target down.
                    if abs(slope) < .0001 or slope * (-1 if axis == 0 else 1) <= 0:
                        raise RuntimeError('Image motion disagrees with gimbal motion; target may have moved')
                    slopes[axis] = slope
                else:
                    raise RuntimeError('Gimbal feedback shows insufficient movement to estimate correction')
            centered = max(abs(v) for v in center) <= center_tolerance
            sized = abs(ratio - target_ratio) <= size_tolerance
            stable = stable + 1 if centered and sized else 0
            if stable >= 2:
                return result(True, 'Target centered and framed')
            if centered and sized:
                continue
            if len(history) >= max_steps or time.monotonic() >= deadline:
                break
            if not centered:
                axis = int(abs(center[1]) > abs(center[0]))
                if axis in slopes:
                    delta = -.65 * center[axis] / slopes[axis]
                else:
                    delta = math.copysign(.5 / zoom, center[axis] * (1 if axis == 0 else -1))
                limit = min(3.0 / zoom, 3.0)
                delta = round(float(np.clip(delta, -limit, limit)), 2)
                if abs(delta) < .02:
                    raise RuntimeError('Required correction is below reliable gimbal resolution')
                name = 'gimbal_yaw' if axis == 0 else 'gimbal_pitch'
                entry = dict(command=name, value=delta, confirmed=False)
                history.append(entry)
                box_confirmed = False
                feedback = getattr(controller, name)(delta)
                key = 'yaw_deg' if axis == 0 else 'pitch_deg'
                actual_delta = feedback[key] - feedback['start'][key]
                entry.update(confirmed=True, actual_delta=actual_delta)
                pending = (axis, center[axis], feedback['start'][key])
            else:
                desired = float(np.clip(zoom * target_ratio / ratio, zoom * .8, zoom * 1.25))
                desired = round(float(np.clip(desired, 1, 160)), 1)
                if abs(desired - zoom) < .05:
                    raise RuntimeError('Target size cannot be reached at the available zoom range/resolution')
                entry = dict(command='zoom', value=desired, confirmed=False)
                history.append(entry)
                box_confirmed = False
                zoom = float(controller.zoom(desired)['zoom'])
                entry['confirmed'] = True
                slopes.clear()
        return result(False, 'Automatic framing reached its step/time limit')
    except KeyboardInterrupt:
        return result(False, 'Automatic framing interrupted; no further camera commands will be sent')
    except Exception as exc:
        return result(False, str(exc))
