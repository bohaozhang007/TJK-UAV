import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rospy

from hardware.k40t import close_camera, get_img, detect_img, track_img, get_gimbal, set_gimbal
from hardware.k40t import get_zoom, set_zoom, check_tracker
from hardware.pose import close_pose, get_pose
from task.detection import sample_is_fresh


# Target matching
MATCH_DISTANCE_M = 1.0

# Framing completion
CENTER_TOLERANCE = 0.06
TARGET_RATIO = 0.4
SIZE_TOLERANCE = 0.08
STABLE_FRAMES = 2

# Timing and iteration limits
MAX_STEPS = 30
SETTLE_S = 0.5
FRAME_MAX_AGE_S = 0.5

# Gimbal control
YAW_GAIN_DEG = 64.0
PITCH_GAIN_DEG = 36.0
DAMPING = 0.7
MAX_STEP_DEG = 3.0

# Zoom control
BASELINE_ZOOM = 1.0
ZOOM_STEP_UP = 1.25
ZOOM_STEP_DOWN = 0.8
MAX_ZOOM = 160.0


def match_target(detections, target):
    matches = []
    for item in detections:
        if (
            item["position_world_m"] is None
            or item["world_frame"] != target["world_frame"]
        ):
            continue
        distance = np.linalg.norm(np.asarray(item["position_world_m"]) - target["position_world_m"])
        if distance <= MATCH_DISTANCE_M:
            matches.append(item)
    if len(matches) != 1:
        raise RuntimeError("Autofocus target is missing or ambiguous in the current image")
    return matches[0]["box"]


def center_error(box, shape):
    box = np.asarray(box, dtype=float)
    height, width = shape[:2]
    if (
        box.shape != (4,)
        or not np.isfinite(box).all()
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
        set_zoom(BASELINE_ZOOM)
        # DA3 assumes the forward camera mounting; reacquire before moving the gimbal.
        set_gimbal(
            0.0,
            0.0,
        )
        time.sleep(SETTLE_S)
        with ThreadPoolExecutor(max_workers=2) as readers:
            img_future = readers.submit(get_img)
            pose_future = readers.submit(get_pose)
            img_sample, pose_sample = img_future.result(), pose_future.result()
        if not sample_is_fresh(img_sample, pose_sample):
            raise RuntimeError("Autofocus image or pose is stale")
        img, _ = img_sample
        pose, _ = pose_sample
        detections = detect_img(img, pose)
        box = match_target(detections, plan["target"])
        result = track_img(img, box)
        if result["box"] is None:
            raise RuntimeError("SAM2 could not initialize the autofocus target")
        stable = 0
        for _ in range(MAX_STEPS):
            time.sleep(SETTLE_S)
            img, received = get_img()
            if time.monotonic() - received > FRAME_MAX_AGE_S:
                raise RuntimeError("Autofocus image is stale")
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
            # Match v22's reference-resolution gains; move one axis per fresh frame.
            axis = int(abs(error[1]) > abs(error[0]))
            gain = -PITCH_GAIN_DEG if axis else YAW_GAIN_DEG
            delta = round(float(np.clip(
                error[axis] * gain * DAMPING * BASELINE_ZOOM / zoom,
                -MAX_STEP_DEG,
                MAX_STEP_DEG,
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
        try:
            # Do not move the gimbal after a keyboard interruption.
            if not interrupted:
                set_zoom(BASELINE_ZOOM)
                set_gimbal(
                    0.0,
                    0.0,
                )
                time.sleep(SETTLE_S)
        finally:
            try:
                close_camera()
            finally:
                close_pose()
