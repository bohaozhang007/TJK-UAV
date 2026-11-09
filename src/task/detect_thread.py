import json
import queue
import threading
import time
from concurrent.futures import ThreadPoolExecutor, wait

import numpy as np
import rospy

from util.timing import timed, log_timing
from util.target_log import log_target, target_snapshot

from util.dedup import (
    MAX_CAPTURE_ATTEMPTS,
    select_new_targets,
    match_rechecked_targets,
    select_current_targets,
)
from hardware.k40t import get_img, detect_img
from hardware.pose import get_pose
from interface.next_waypoint import NextWaypoint


# Sampling and buffering.
SAMPLE_INTERVAL_S = 0.2
QUEUE_SIZE = 10

# Sample freshness.
IMG_MAX_AGE_S = 0.5
POSE_MAX_AGE_S = 0.5

# Capture retries.
CAPTURE_RETRY_S = 0.1

# Inference request retries.
SEND_ATTEMPTS = 2
SEND_RETRY_S = 0.1

# Target selection radius from the paired image pose, including height difference.
MAX_TARGET_DISTANCE_M = 8.0
RECHECK_CAPTURE_ATTEMPTS = 3


class DetectThread:
    def __init__(self):
        self.frames = queue.Queue(maxsize=QUEUE_SIZE)
        self.stop = threading.Event()
        self.worker = None
        self.target_records = {}
        self.waypoint = NextWaypoint()
        self.recheck_frame_id = 0
        self.last_recheck = None

    def start(self):
        self.stop.clear()
        self.worker = threading.Thread(
            target=capture_loop,
            args=(self.frames, self.stop, self.waypoint),
            daemon=True,
        )
        self.worker.start()

    @timed("uav.detection_pause")
    def pause(self):
        self.stop.set()
        if self.worker is not None:
            self.worker.join()
            self.worker = None
        self.frames = queue.Queue(maxsize=QUEUE_SIZE)

    def close(self):
        try:
            self.pause()
        finally:
            self.waypoint.close()

    def start_attempt(self, target):
        record = target["_record"]
        record["attempts"] += 1
        rospy.loginfo(f"Target {record['target_id']} attempt {record['attempts']}/{MAX_CAPTURE_ATTEMPTS}")

    def mark_observed(self, target):
        target["_record"]["observed"] = True

    def expand_current_targets(self, matched):
        """Admit B-only pending targets after B has been chosen and checked."""
        if self.last_recheck is None:
            raise RuntimeError("Cannot choose a current batch without a successful recheck")
        snapshot = self.last_recheck
        if snapshot["missing_ids"] or {target["target_id"] for target in matched} != snapshot["matched_ids"]:
            raise RuntimeError("Current batch requires full coverage of the original targets")
        return select_current_targets(
            matched,
            snapshot["detections"],
            snapshot["img"],
            self.target_records,
            log_context=snapshot["context"],
        )

    @timed("uav.recheck_targets")
    def recheck_targets(self, targets):
        """Detect a fresh held frame, associating only existing batch IDs.

        The patrol worker must be paused. No target histories or IDs are changed.
        Every HTTP retry samples a new frame and its own paired vehicle pose.
        """
        if self.worker is not None:
            raise RuntimeError("Pause patrol detection before rechecking held targets")
        self.last_recheck = None
        for attempt in range(SEND_ATTEMPTS):
            if rospy.is_shutdown():
                raise rospy.ROSInterruptException("Shutdown during held re-detection")
            try:
                img, pose = capture_current_frame()
                self.recheck_frame_id += 1
                # Separate these IDs from the positive patrol/capture frame IDs.
                frame_id = -self.recheck_frame_id
                detections = detect_img(img, pose, calculate_depth=True, frame_id=frame_id)
                context = {"phase": "held_recheck", "frame_id": frame_id, "pose": pose}
                detections = filter_localized_detections(detections, pose, log_context=context)
                matched, missing = match_rechecked_targets(targets, detections, img, log_context=context)
                for target in matched:
                    target["_localization_context"] = context
                self.last_recheck = {
                    "img": img, "detections": detections, "context": context,
                    "matched_ids": {target["target_id"] for target in matched},
                    "missing_ids": [target["target_id"] for target in missing],
                }
                print(json.dumps({
                    "phase": "held_recheck", "frame_id": frame_id, "pose": pose,
                    "detections": detections,
                    "matched_target_ids": [target["target_id"] for target in matched],
                    "missing_target_ids": [target["target_id"] for target in missing],
                }), flush=True)
                return matched, missing
            except rospy.ROSInterruptException:
                raise
            except Exception as exc:
                rospy.logwarn(f"Held re-detection {attempt + 1}/{SEND_ATTEMPTS} failed: {exc}")
                if attempt + 1 < SEND_ATTEMPTS:
                    rospy.sleep(SEND_RETRY_S)
        rospy.logwarn("Held re-detection failed; all batch targets remain unconfirmed")
        log_target("uav.recheck_unavailable", reason="request_or_sampling_failed",
                   target_ids=[target["target_id"] for target in targets], attempts=SEND_ATTEMPTS)
        return [], list(targets)

    @timed("uav.find_targets")
    def find_targets(self):
        self.start()
        while not rospy.is_shutdown():
            try:
                frame = self.frames.get(timeout=SAMPLE_INTERVAL_S)
            except queue.Empty:
                continue
            frame_id, img, pose, waypoint, queued_at, image_received = frame
            log_timing(
                "uav.frame_dequeued", frame_id=frame_id,
                queue_wait_ms=round((time.monotonic() - queued_at) * 1000, 3),
                image_age_ms=round((time.monotonic() - image_received) * 1000, 3),
                queue_size=self.frames.qsize(),
            )
            for attempt in range(SEND_ATTEMPTS):
                try:
                    detections = detect_img(img, pose, frame_id=frame_id)
                    break
                except Exception as exc:
                    rospy.logwarn(
                        f"Detection failed: frame={frame_id}, attempt={attempt + 1}/{SEND_ATTEMPTS}; "
                        f"{type(exc).__name__}: {exc}"
                    )
                    self.stop.wait(SEND_RETRY_S)
            else:
                rospy.logwarn(f"Dropping frame {frame_id} after {SEND_ATTEMPTS} failed attempts")
                continue
            if detections:
                print(json.dumps({"frame_id": frame_id, "pose": pose, "waypoint": waypoint, "detections": detections}), flush=True)
            selection_started = time.perf_counter()
            raw_count = len(detections)
            context = {"phase": "patrol", "frame_id": frame_id, "pose": pose}
            detections = filter_localized_detections(detections, pose, log_context=context)
            targets = select_new_targets(detections, self.target_records, log_context=context)
            for target in targets:
                target["_localization_context"] = context
            log_timing(
                "uav.target_selection", frame_id=frame_id, raw_count=raw_count,
                valid_count=len(detections), selected_count=len(targets),
                duration_ms=round((time.perf_counter() - selection_started) * 1000, 3),
            )
            if targets:
                # Keep the detection frame paired with its boxes for tracker initialization.
                for target in targets:
                    target["exposure_img"] = img
                self.pause()
                return pose, waypoint, targets
        return None


def filter_localized_detections(detections, pose, log_context=None):
    valid = []
    for index, detection in enumerate(detections):
        position = np.asarray(detection.get("position_world_m"), dtype=float)
        valid_position = position.shape == (3,) and np.isfinite(position).all()
        same_frame = detection.get("world_frame") == pose["frame_id"]
        distance = float(np.linalg.norm(position - np.asarray(pose["position_m"]))) if valid_position and same_frame else None
        reason = ("invalid_world_position" if not valid_position else
                  "world_frame_mismatch" if not same_frame else
                  "outside_distance_limit" if distance > MAX_TARGET_DISTANCE_M else "accepted")
        log_target("uav.target_localization", context=log_context, pose=pose,
                   detection_index=index, detection=target_snapshot(detection),
                   distance_from_uav_m=distance, max_distance_m=MAX_TARGET_DISTANCE_M,
                   decision=reason)
        if (
            position.shape == (3,)
            and np.isfinite(position).all()
            and detection.get("world_frame") == pose["frame_id"]
            and np.linalg.norm(position - np.asarray(pose["position_m"])) <= MAX_TARGET_DISTANCE_M
        ):
            valid.append(detection)
    return valid


def capture_current_frame():
    """Bypass the patrol queue; obtain a fresh image/pose pair while held."""
    with ThreadPoolExecutor(max_workers=2) as readers:
        for _ in range(RECHECK_CAPTURE_ATTEMPTS):
            if rospy.is_shutdown():
                raise rospy.ROSInterruptException("Shutdown while sampling held image")
            img_future = readers.submit(get_img)
            pose_future = readers.submit(get_pose)
            wait((img_future, pose_future))
            img_sample, pose_sample = img_future.result(), pose_future.result()
            if sample_is_fresh(img_sample, pose_sample):
                return img_sample[0], pose_sample[0]
    raise RuntimeError("Could not obtain a fresh image/pose pair for held re-detection")


def sample_is_fresh(img_sample, pose_sample):
    _, img_received = img_sample
    pose, pose_received = pose_sample
    now = time.monotonic()
    ros_now = rospy.Time.now().to_sec()
    # Image receipt time cannot verify camera exposure time or exact synchronization.
    return (
        0 <= now - img_received <= IMG_MAX_AGE_S
        and 0 <= now - pose_received <= POSE_MAX_AGE_S
        and 0 <= ros_now - pose["stamp_s"] <= POSE_MAX_AGE_S
    )


def capture_loop(
    frames,
    stop,
    waypoint,
):
    frame_id = 0
    with ThreadPoolExecutor(max_workers=2) as readers:
        while not stop.is_set():
            started = time.monotonic()
            stage = "starting image and pose reads"
            try:
                img_future = readers.submit(get_img)
                pose_future = readers.submit(get_pose)
                # Finish both reads before retrying; receipt times are not exposure times.
                wait((img_future, pose_future))
                stage = "reading camera image"
                img_sample = img_future.result()
                stage = "reading vehicle pose"
                pose_sample = pose_future.result()
                stage = "reading next waypoint"
                waypoint_sample = waypoint.get()
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0,
                    f"Capture failed while {stage}; retrying every {CAPTURE_RETRY_S:g}s: "
                    f"{type(exc).__name__}: {exc}",
                )
                stop.wait(CAPTURE_RETRY_S)
                continue
            fresh = sample_is_fresh(img_sample, pose_sample)
            log_timing("uav.sample", fresh=fresh,
                       duration_ms=round((time.monotonic() - started) * 1000, 3))
            if fresh:
                img, _ = img_sample
                pose, _ = pose_sample
                frame_id += 1
                # Pair at sampling time, not at the camera exposure time.
                if frames.full():
                    try:
                        frames.get_nowait()
                    except queue.Empty:
                        pass
                frames.put_nowait((frame_id, img, pose, waypoint_sample, time.monotonic(), img_sample[1]))
            stop.wait(max(0.0, SAMPLE_INTERVAL_S - (time.monotonic() - started)))
