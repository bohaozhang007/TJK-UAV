import numpy as np
import rospy

from hardware.k40t import get_img, track_img, get_gimbal, set_gimbal
from hardware.k40t import get_zoom, set_zoom, check_tracker


# Framing completion
TARGET_RATIO = 0.4
CENTER_TOLERANCE = 0.06
SIZE_TOLERANCE = 0.08
STABLE_FRAMES = 2

# Iteration limit
MAX_STEPS = 50

# Gimbal control
REFERENCE_WIDTH_PX = 640
REFERENCE_HEIGHT_PX = 360
ROTATE_DEG_PER_PIXEL = 0.1
MAX_ROTATE_DEG = 30.0

# Zoom control
BASELINE_ZOOM = 1.0
ZOOM_STEP_UP = 1.5
ZOOM_STEP_DOWN = 0.5
MAX_ZOOM = 160.0


def center_error(box, shape):
    box = np.asarray(box, dtype=float)
    height, width = shape[:2]
    if (
        box.shape != (4,)
        or not 0 <= box[0] < box[2] <= width
        or not 0 <= box[1] < box[3] <= height
    ):
        raise RuntimeError("Tracker lost the target or returned an invalid box")
    return (box[:2] + box[2:]) / (2 * np.array([width, height])) - 0.5


def run(flight, plan):
    # Recheck before any camera movement in case the tracker exited after startup.
    check_tracker()
    interrupted = False

    try:
        # Initialize with the exact frame and box used to detect this target.
        target = plan["target"]
        result = track_img(target["exposure_img"], target["box"])
        if result["box"] is None:
            raise RuntimeError("SAM2 could not initialize the autofocus target")
        set_zoom(BASELINE_ZOOM)
        set_gimbal(
            0.0,
            0.0,
        )
        stable = 0
        for _ in range(MAX_STEPS):
            img, _ = get_img()
            result = track_img(img)
            if result["box"] is None:
                raise RuntimeError("SAM2 lost the autofocus target")
            error = center_error(result["box"], img.shape)
            centered = np.max(np.abs(error)) <= CENTER_TOLERANCE
            x1, y1, x2, y2 = result["box"]
            ratio = max((x2 - x1) / img.shape[1], (y2 - y1) / img.shape[0])
            sized = abs(ratio - TARGET_RATIO) <= SIZE_TOLERANCE
            framed = (
                centered
                and sized
            )
            stable = stable + 1 if framed else 0
            if stable >= STABLE_FRAMES:
                rospy.loginfo("Autofocus complete: target centered and sized")
                return True
            if framed:
                continue
            zoom = get_zoom()
            if centered:
                desired = float(np.clip(
                    zoom * TARGET_RATIO / ratio,
                    zoom * ZOOM_STEP_DOWN,
                    zoom * ZOOM_STEP_UP,
                ))
                desired = round(min(MAX_ZOOM, max(BASELINE_ZOOM, desired)), 1)
                if abs(desired - zoom) < 0.05:
                    raise RuntimeError("Target size cannot be reached within the available zoom range")
                set_zoom(desired)
                continue
            # Scale normalized errors to v20 reference pixels; move one axis per frame.
            axis = int(abs(error[1]) > abs(error[0]))
            reference_pixels = -REFERENCE_HEIGHT_PX if axis else REFERENCE_WIDTH_PX
            delta = round(float(np.clip(
                error[axis] * reference_pixels * ROTATE_DEG_PER_PIXEL * BASELINE_ZOOM / zoom,
                -MAX_ROTATE_DEG,
                MAX_ROTATE_DEG,
            )), 2)
            status = get_gimbal()
            pitch_deg, yaw_deg = status["pitch_deg"], status["yaw_deg"]
            if axis:
                pitch_deg += delta
            else:
                yaw_deg += delta
            set_gimbal(pitch_deg, yaw_deg)
        raise RuntimeError("Autofocus reached its step limit")
    except KeyboardInterrupt:
        interrupted = True
        raise
    finally:
        # Do not move the gimbal after a keyboard interruption.
        if not interrupted:
            set_zoom(BASELINE_ZOOM)
            set_gimbal(
                0.0,
                0.0,
            )
