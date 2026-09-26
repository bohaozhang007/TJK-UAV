import json
import queue
import threading
import time
from concurrent.futures import ThreadPoolExecutor, wait

import rospy

from util.dedup import select_new_targets
from hardware.k40t import get_img, detect_img
from hardware.pose import get_pose


# Sampling and buffering.
SAMPLE_INTERVAL_S = 0.1
QUEUE_SIZE = 10

# Sample freshness.
IMG_MAX_AGE_S = 0.5
POSE_MAX_AGE_S = 0.5

# Capture retries.
CAPTURE_RETRY_S = 0.1

# Inference request retries.
SEND_ATTEMPTS = 2
SEND_RETRY_S = 0.1


class DetectThread:
    def __init__(self):
        self.frames = queue.Queue(maxsize=QUEUE_SIZE)
        self.stop = threading.Event()
        self.worker = None
        self.seen_positions = {}

    def start(self):
        self.stop.clear()
        self.worker = threading.Thread(
            target=capture_loop,
            args=(self.frames, self.stop),
            daemon=True,
        )
        self.worker.start()

    def pause(self):
        self.stop.set()
        if self.worker is not None:
            self.worker.join()
            self.worker = None
        self.frames = queue.Queue(maxsize=QUEUE_SIZE)

    def find_targets(self):
        self.start()
        while not rospy.is_shutdown():
            try:
                frame = self.frames.get(timeout=SAMPLE_INTERVAL_S)
            except queue.Empty:
                continue
            frame_id, img, pose = frame
            for attempt in range(SEND_ATTEMPTS):
                try:
                    detections = detect_img(img, pose)
                    break
                except Exception as exc:
                    rospy.logwarn(f"Frame {frame_id}, attempt {attempt + 1}/{SEND_ATTEMPTS}: {exc}")
                    self.stop.wait(SEND_RETRY_S)
            else:
                rospy.logwarn(f"Dropping frame {frame_id} after {SEND_ATTEMPTS} failed attempts")
                continue
            print(json.dumps({"frame_id": frame_id, "pose": pose, "detections": detections}), flush=True)
            targets = select_new_targets(detections, self.seen_positions)
            if targets:
                # Keep the detection frame paired with its boxes for tracker initialization.
                for target in targets:
                    target["exposure_img"] = img
                self.pause()
                return pose, targets
        return None


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


def capture_loop(frames, stop):
    frame_id = 0
    with ThreadPoolExecutor(max_workers=2) as readers:
        while not stop.is_set():
            started = time.monotonic()
            try:
                img_future = readers.submit(get_img)
                pose_future = readers.submit(get_pose)
                # Finish both reads before retrying; receipt times are not exposure times.
                wait((img_future, pose_future))
                img_sample = img_future.result()
                pose_sample = pose_future.result()
            except Exception as exc:
                rospy.logwarn(f"Capture failed; retrying: {exc}")
                stop.wait(CAPTURE_RETRY_S)
                continue
            if sample_is_fresh(img_sample, pose_sample):
                img, _ = img_sample
                pose, _ = pose_sample
                frame_id += 1
                # Pair at sampling time, not at the camera exposure time.
                if frames.full():
                    try:
                        frames.get_nowait()
                    except queue.Empty:
                        pass
                frames.put_nowait((frame_id, img, pose))
            stop.wait(max(0.0, SAMPLE_INTERVAL_S - (time.monotonic() - started)))
