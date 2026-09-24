import json
import queue
import threading
import time
from concurrent.futures import ThreadPoolExecutor, wait

import rospy

from util.dedup import select_new_targets
from hardware.k40t import close_camera, get_img, detect_img
from hardware.pose import close_pose, get_pose


SAMPLE_INTERVAL_S = 0.1
QUEUE_SIZE = 50  # FIFO; discard the oldest queued frame when full.
IMG_MAX_AGE_S = 0.5
POSE_MAX_AGE_S = 0.5
CAPTURE_RETRY_S = 0.1
SEND_ATTEMPTS = 2
SEND_RETRY_S = 0.1


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
                while not stop.is_set():
                    try:
                        frames.put_nowait((frame_id, img, pose))
                        break
                    except queue.Full:
                        try:
                            frames.get_nowait()
                        except queue.Empty:
                            pass
            stop.wait(max(0.0, SAMPLE_INTERVAL_S - (time.monotonic() - started)))


class Detection:
    def __init__(self):
        self.frames = queue.Queue(maxsize=QUEUE_SIZE)
        self.stop = threading.Event()
        self.worker = None
        self.seen_positions = {}

    def start(self):
        if self.worker is not None:
            return
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
        try:
            close_camera()
        finally:
            close_pose()
            while True:
                try:
                    self.frames.get_nowait()
                except queue.Empty:
                    break

    def find_targets(self, flight):
        self.start()
        while flight.is_offboard():
            try:
                frame = self.frames.get(timeout=SAMPLE_INTERVAL_S)
            except queue.Empty:
                continue
            frame_id, img, pose = frame
            for attempt in range(SEND_ATTEMPTS):
                if not flight.is_offboard():
                    self.pause()
                    return None
                try:
                    detections = detect_img(img, pose)
                    break
                except Exception as exc:
                    rospy.logwarn(f"Frame {frame_id}, attempt {attempt + 1}/{SEND_ATTEMPTS}: {exc}")
                    if attempt + 1 < SEND_ATTEMPTS:
                        self.stop.wait(SEND_RETRY_S)
            else:
                rospy.logwarn(f"Dropping frame {frame_id} after {SEND_ATTEMPTS} failed attempts")
                continue
            if not flight.is_offboard():
                break
            print(json.dumps({"frame_id": frame_id, "pose": pose, "detections": detections}), flush=True)
            targets = select_new_targets(detections, self.seen_positions)
            if targets:
                self.pause()
                return pose, targets
        self.pause()
        return None
