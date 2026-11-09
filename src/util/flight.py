import time
import math
from datetime import datetime

import rospy

from util.timing import timed

from hardware.pose import get_pose
from interface.check_goal_reached import GoalReached
from interface.release_control import ControlRelease
from interface.to_target_pose import TargetPose


CONNECTION_TIMEOUT_S = 5.0
ARRIVAL_TIMEOUT_S = 120.0
POLL_INTERVAL_S = 0.1
TIME_FORMAT = "%H:%M:%S.%f"

# Feedback acknowledgement alone does not establish a settled camera platform.
HOLD_POSITION_TOLERANCE_M = 0.3
HOLD_YAW_TOLERANCE_DEG = 5.0
HOLD_POSITION_CHANGE_M = 0.05
HOLD_YAW_CHANGE_DEG = 2.0
HOLD_STABLE_READINGS = 3
HOLD_STABLE_TIMEOUT_S = 10.0


def poses_close(actual, goal, position_tolerance=HOLD_POSITION_TOLERANCE_M,
                yaw_tolerance=HOLD_YAW_TOLERANCE_DEG):
    if actual["frame_id"] != goal["frame_id"]:
        return False
    position = actual["position_m"]
    target = goal["position_m"]
    values = [*position, *target, actual["yaw_deg"], goal["yaw_deg"]]
    if len(position) != 3 or len(target) != 3 or not all(math.isfinite(v) for v in values):
        return False
    yaw_error = abs((actual["yaw_deg"] - goal["yaw_deg"] + 180) % 360 - 180)
    return math.dist(position, target) <= position_tolerance and yaw_error <= yaw_tolerance


class FlightControl:
    def __init__(self):
        self.release = ControlRelease()
        self.goal = TargetPose()
        self.feedback = GoalReached()

    @timed("uav.hold_current_pose")
    def hold_current_pose(self):
        """Acquire mission control at the current pose, then verify settling."""
        pose, _ = get_pose()
        rospy.loginfo(f"Holding current pose: position={pose['position_m']}, yaw={pose['yaw_deg']:.2f}")
        self.go_to_pose(pose)
        self.wait_for_stable_pose(pose)
        return pose

    @timed("uav.wait_for_stable_pose")
    def wait_for_stable_pose(self, goal):
        deadline = time.monotonic() + HOLD_STABLE_TIMEOUT_S
        previous = None
        stable = 0
        while not rospy.is_shutdown():
            if time.monotonic() >= deadline:
                raise RuntimeError("Vehicle did not settle at the held pose before re-detection")
            actual, _ = get_pose()
            settled = poses_close(actual, goal)
            if previous is not None:
                settled = settled and poses_close(
                    actual, previous, HOLD_POSITION_CHANGE_M, HOLD_YAW_CHANGE_DEG,
                )
            stable = stable + 1 if settled else 0
            if stable >= HOLD_STABLE_READINGS:
                return actual
            previous = actual
            rospy.sleep(POLL_INTERVAL_S)
        raise rospy.ROSInterruptException("Shutdown while waiting for a stable hold")

    def assert_at_pose(self, goal):
        """Reject tracking if the vehicle moved away during the HTTP request."""
        actual, _ = get_pose()
        if not poses_close(actual, goal):
            raise RuntimeError("Vehicle left the held pose during re-detection; refusing to track a stale image")
        return actual

    @timed("uav.flight_go_to_pose")
    def go_to_pose(self, pose):
        # Stage 1: Wait for the mission interfaces; never request a flight mode change.
        deadline = time.monotonic() + CONNECTION_TIMEOUT_S
        while not (
            self.release.connected()
            and self.goal.connected()
            and self.feedback.connected()
        ):
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Mission interface connection timeout of {CONNECTION_TIMEOUT_S:g} s exceeded")
            time.sleep(POLL_INTERVAL_S)

        # Stage 2: Send the requested SLAM pose, acquiring or retaining task control.
        # Clear before sending so even an immediate True response is captured.
        self.feedback.reset()
        try:
            sent_time = datetime.now().strftime(TIME_FORMAT)[:-3]
            sent_at = time.monotonic()
            self.goal.send(pose)
            rospy.loginfo(f"Goal sent: time={sent_time}")

            # Stage 3: Wait for position and yaw feedback.
            deadline = sent_at + ARRIVAL_TIMEOUT_S
            while True:
                if time.monotonic() >= deadline:
                    self.log_result(
                        "timeout",
                        time.monotonic() - sent_at,
                    )
                    raise RuntimeError(f"Goal arrival feedback timeout of {ARRIVAL_TIMEOUT_S:g} s exceeded")
                if self.feedback.wait(POLL_INTERVAL_S):
                    self.log_result(
                        "reached",
                        self.feedback.received_at - sent_at,
                    )
                    return True
        finally:
            self.feedback.reset()

    def log_result(
        self,
        status,
        elapsed,
    ):
        actual, _ = get_pose()
        position = actual["position_m"]
        message = f"Goal {status}: elapsed={elapsed:.2f}s, actual={position}"
        rospy.loginfo(message)

    @timed("uav.flight_release")
    def release_control(self):
        # Stage 4: After all extra operations, return control without changing modes.
        if not self.release.connected():
            raise RuntimeError("Mission release topic has no subscriber")
        self.release.send()
        print("Control release sent; returning to patrol detection.", flush=True)
        return True

    def close(self):
        self.feedback.close()
        self.goal.close()
        self.release.close()
