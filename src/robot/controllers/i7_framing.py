"""Frame a target with remote tracking and reference-resolution pixel gains."""

from __future__ import annotations

import math
import time

import numpy as np

from ..hardware.tracker_client import TrackerClient, validate_image_box


def autofocus_box(controller, img, box, capture, *,
                  tracker_url='http://192.168.31.66:8791', tracker_timeout_s=30.0,
                  reference_width_px=640, reference_height_px=360,
                  reference_zoom=1.0, yaw_deg_per_pixel=.1, pitch_deg_per_pixel=.1,
                  max_yaw_step_deg=3.0, max_pitch_step_deg=3.0, damping=.7,
                  zoom_step_up=1.25, zoom_step_down=.8,
                  target_ratio=.4, center_tolerance=.04, size_tolerance=.05,
                  max_steps=30, timeout_s=180.0, settle_s=.5, tracker=None):
    """Init with the supplied full-size BGR image/xyxy box, then track each frame.

    Gains are degrees/pixel at the reference resolution and zoom, following
    v21's reference-to-actual resolution conversion. Positive yaw turns right;
    positive pitch turns up. This does not command aircraft motion or lens focus.
    """
    bounds = validate_image_box(img, box)
    for name, value, low, high in (
        ('target_ratio', target_ratio, .1, .8),
        ('center_tolerance', center_tolerance, .005, .1),
        ('size_tolerance', size_tolerance, .005, .1),
        ('timeout_s', timeout_s, 1, 600), ('settle_s', settle_s, .1, 5),
        ('tracker_timeout_s', tracker_timeout_s, .1, 180),
        ('reference_width_px', reference_width_px, 1, 10000),
        ('reference_height_px', reference_height_px, 1, 10000),
        ('reference_zoom', reference_zoom, 1, 160),
        ('yaw_deg_per_pixel', yaw_deg_per_pixel, .0001, 1),
        ('pitch_deg_per_pixel', pitch_deg_per_pixel, .0001, 1),
        ('max_yaw_step_deg', max_yaw_step_deg, .02, 30),
        ('max_pitch_step_deg', max_pitch_step_deg, .02, 30),
        ('damping', damping, .05, 1),
        ('zoom_step_up', zoom_step_up, 1.01, 1.5),
        ('zoom_step_down', zoom_step_down, .5, .99),
    ):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{name} must be within [{low}, {high}]')
    if isinstance(max_steps, bool) or not isinstance(max_steps, int) or not 1 <= max_steps <= 200:
        raise ValueError('max_steps must be an integer from 1 to 200')
    h, w = img.shape[:2]
    deadline = time.monotonic() + timeout_s
    history = []
    stable = 0
    ratio = None
    center = None
    zoom = None
    box_confirmed = False
    track_count = 0

    def result(ok, message):
        return dict(ok=ok, message=message, box=bounds.tolist(), box_confirmed=box_confirmed,
                    image_width=w, image_height=h, occupancy=ratio,
                    target_ratio=target_ratio, center_error=center, zoom=zoom,
                    tracker_url=tracker_url, track_count=track_count, actions=history)

    def budget():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError('Automatic framing timed out')
        return remaining

    try:
        tracker = tracker if tracker is not None else TrackerClient(tracker_url, tracker_timeout_s)
        bounds = validate_image_box(img, tracker.init(img, bounds, timeout_s=min(tracker_timeout_s, budget())))
        zoom = float(controller.get_zoom()['zoom'])
        while True:
            remaining = budget()
            if remaining <= settle_s:
                return result(False, 'Automatic framing timed out')
            box_confirmed = False
            frame = capture(time.monotonic() + settle_s, min(3.0, remaining))
            if frame.shape[:2] != (h, w):
                raise RuntimeError('Live image dimensions differ from img; pass a native-resolution image')
            bounds = validate_image_box(frame, tracker.track(frame, timeout_s=min(tracker_timeout_s, budget())))
            track_count += 1
            budget()
            box_confirmed = True
            center = [float((bounds[0] + bounds[2]) / (2 * w) - .5),
                      float((bounds[1] + bounds[3]) / (2 * h) - .5)]
            ratio = float(max((bounds[2] - bounds[0]) / w, (bounds[3] - bounds[1]) / h))
            centered = max(abs(v) for v in center) <= center_tolerance
            sized = abs(ratio - target_ratio) <= size_tolerance
            stable = stable + 1 if centered and sized else 0
            if stable >= 2:
                return result(True, 'Target centered and framed')
            if centered and sized:
                continue
            if len(history) >= max_steps:
                return result(False, 'Automatic framing reached its step limit')
            zoom = float(controller.get_zoom()['zoom'])
            if not math.isfinite(zoom) or not 1 <= zoom <= 160:
                raise RuntimeError('Camera returned invalid zoom feedback')
            budget()
            if not centered:
                # v21: gain_actual = gain_reference * reference_size / image_size.
                # Reduce angular gain at increased zoom (an approximation).
                axis = int(abs(center[1]) > abs(center[0]))
                pixel_error = center[axis] * (w if axis == 0 else -h)
                gain = ((yaw_deg_per_pixel * reference_width_px / w) if axis == 0
                        else (pitch_deg_per_pixel * reference_height_px / h))
                delta = pixel_error * gain * damping * reference_zoom / zoom
                limit = max_yaw_step_deg if axis == 0 else max_pitch_step_deg
                delta = round(float(np.clip(delta, -limit, limit)), 2)
                if abs(delta) < .02:
                    raise RuntimeError('Required correction is below reliable gimbal resolution')
                name = 'gimbal_yaw' if axis == 0 else 'gimbal_pitch'
                entry = dict(command=name, value=delta, confirmed=False)
                history.append(entry)
                box_confirmed = False
                feedback = getattr(controller, name)(delta)
                key = 'yaw_deg' if axis == 0 else 'pitch_deg'
                entry.update(confirmed=True, actual_delta=feedback[key] - feedback['start'][key])
            else:
                desired = float(np.clip(zoom * target_ratio / ratio, zoom * zoom_step_down, zoom * zoom_step_up))
                desired = round(float(np.clip(desired, 1, 160)), 1)
                if abs(desired - zoom) < .05:
                    raise RuntimeError('Target size cannot be reached at the available zoom range/resolution')
                entry = dict(command='zoom', value=desired, confirmed=False)
                history.append(entry)
                box_confirmed = False
                zoom = float(controller.zoom(desired)['zoom'])
                entry['confirmed'] = True
    except KeyboardInterrupt:
        return result(False, 'Automatic framing interrupted; no further camera commands will be sent')
    except Exception as exc:
        return result(False, str(exc))
