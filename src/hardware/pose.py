import atexit
import math
import threading
import time

import rospy
from nav_msgs.msg import Odometry


# Read FAST-LIO directly in the SLAM map frame.
POSE_TOPIC = "/laserMapping/odometry"
POSE_WAIT_TIMEOUT_S = 0.5
POSE_MAX_AGE_S = 0.5


_pose = None
_subscriber = None
_lock = threading.Lock()
_ready = threading.Event()


def _update_pose(message):
    global _pose
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    _pose = {
        "stamp_s": message.header.stamp.to_sec(),
        "frame_id": message.header.frame_id,
        "position_m": [p.x, p.y, p.z],
        "yaw_deg": math.degrees(yaw),
        # Full measured attitude is still needed for DA3 camera-to-world localization.
        "quaternion_xyzw": [q.x, q.y, q.z, q.w],
    }, time.monotonic()
    _ready.set()


def get_pose():
    """Wait for a fresh pose; return it with its monotonic receipt time."""
    global _subscriber
    with _lock:
        _ready.clear()
        if _subscriber is None:
            _subscriber = rospy.Subscriber(
                POSE_TOPIC,
                Odometry,
                _update_pose,
                queue_size=1,
            )
        if not _ready.wait(timeout=POSE_WAIT_TIMEOUT_S):
            raise RuntimeError(f"Pose message timeout of {POSE_WAIT_TIMEOUT_S:g} s exceeded")
        pose, received = _pose
        if not 0 <= rospy.Time.now().to_sec() - pose["stamp_s"] <= POSE_MAX_AGE_S:
            raise RuntimeError("Pose is stale or has an invalid timestamp")
        return pose, received


def close_pose():
    global _subscriber, _pose
    with _lock:
        if _subscriber is not None:
            _subscriber.unregister()
            _subscriber = None
        _pose = None
        _ready.clear()


atexit.register(close_pose)
