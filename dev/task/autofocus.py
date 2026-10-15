import numpy as np
import rospy

from hardware.k40t import get_img, track_img, get_gimbal, set_gimbal
from hardware.k40t import get_zoom, set_zoom, check_tracker


# Framing completion
TARGET_RATIO = 0.35
CENTER_TOLERANCE = 0.1
SIZE_TOLERANCE = 0.1
STABLE_FRAMES = 1

# Iteration limit
MAX_STEPS = 50

# Gimbal control
REFERENCE_WIDTH_PX = 640
REFERENCE_HEIGHT_PX = 360
ROTATE_DEG_PER_PIXEL = 0.1
MAX_ROTATE_DEG = 30.0

# Zoom control
BASELINE_ZOOM = 1.0
ZOOM_STEP_UP = 2
ZOOM_STEP_DOWN = 0.5
MAX_ZOOM = 160.0


def skip_autofocus(reason):
    message = f"Autofocus skipped: {reason}"
    rospy.logwarn(message)
    print(f"WARNING: {message}", flush=True)
    return False


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


def run(
    exposure_img,
    box,
    capture=None,
):
    # Recheck before any camera movement in case the tracker exited after startup.
    rospy.loginfo("Autofocus: checking tracker readiness")
    check_tracker()

    try:
        # Initialize with the exact frame and box used to detect this target.
        rospy.loginfo(f"Autofocus: initializing tracker with box={box}")
        result = track_img(exposure_img, box)
        if result["box"] is None:
            return skip_autofocus("SAM2 could not initialize the target")
        rospy.loginfo(f"Autofocus: setting baseline zoom to {BASELINE_ZOOM:g}x")
        set_zoom(BASELINE_ZOOM)
        rospy.loginfo("Autofocus: setting gimbal to zero")
        set_gimbal(
            0.0,
            0.0,
        )
        stable = 0
        for step in range(1, MAX_STEPS + 1):
            rospy.loginfo(f"Autofocus step {step}/{MAX_STEPS}: reading image")
            img, _ = get_img()
            rospy.loginfo(f"Autofocus step {step}/{MAX_STEPS}: tracking")
            result = track_img(img)
            if result["box"] is None:
                return skip_autofocus(f"SAM2 lost the target at step {step}/{MAX_STEPS}")
            try:
                error = center_error(result["box"], img.shape)
            except (RuntimeError, ValueError) as exc:
                return skip_autofocus(f"Invalid target box at step {step}/{MAX_STEPS}: {exc}")
            centered = np.max(np.abs(error)) <= CENTER_TOLERANCE
            x1, y1, x2, y2 = result["box"]
            ratio = max((x2 - x1) / img.shape[1], (y2 - y1) / img.shape[0])
            sized = abs(ratio - TARGET_RATIO) <= SIZE_TOLERANCE
            rospy.loginfo(
                f"Autofocus step {step}/{MAX_STEPS}: box={result['box']}, "
                f"error={error.tolist()}, ratio={ratio:.3f}, centered={centered}, sized={sized}"
            )
            framed = (
                centered
                and sized
            )
            stable = stable + 1 if framed else 0
            if stable >= STABLE_FRAMES:
                rospy.loginfo("Autofocus complete: target centered and sized")
                if capture is not None:
                    rospy.loginfo("Autofocus: starting burst capture")
                    capture()
                    rospy.loginfo("Autofocus: burst capture complete")
                return True
            if framed:
                continue
            rospy.loginfo("Autofocus: reading zoom")
            zoom = get_zoom()
            if centered:
                desired = float(np.clip(
                    zoom * TARGET_RATIO / ratio,
                    zoom * ZOOM_STEP_DOWN,
                    zoom * ZOOM_STEP_UP,
                ))
                desired = round(min(MAX_ZOOM, max(BASELINE_ZOOM, desired)), 1)
                if abs(desired - zoom) < 0.05:
                    rospy.loginfo(f"Skipping zoom adjustment: current={zoom:.1f}x, desired={desired:.1f}x")
                    continue
                rospy.loginfo(f"Autofocus: setting zoom from {zoom:.1f}x to {desired:.1f}x")
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
            rospy.loginfo("Autofocus: reading gimbal angles")
            status = get_gimbal()
            pitch_deg, yaw_deg = status["pitch_deg"], status["yaw_deg"]
            if axis:
                pitch_deg += delta
            else:
                yaw_deg += delta
            rospy.loginfo(f"Autofocus: setting gimbal pitch={pitch_deg:.2f}, yaw={yaw_deg:.2f}")
            set_gimbal(pitch_deg, yaw_deg)
        return skip_autofocus(f"Reached the step limit ({MAX_STEPS}) without satisfying framing conditions")
    except Exception:
        rospy.logerr("Autofocus failed before camera restoration", exc_info=True)
        raise
    finally:
        # Restore the camera even after a keyboard interruption.
        try:
            rospy.loginfo(f"Autofocus: restoring zoom to {BASELINE_ZOOM:g}x")
            set_zoom(BASELINE_ZOOM)
            rospy.loginfo("Autofocus: zoom restored")
        finally:
            rospy.loginfo("Autofocus: restoring gimbal to zero")
            set_gimbal(
                0.0,
                0.0,
            )
            rospy.loginfo("Autofocus: gimbal restored")
