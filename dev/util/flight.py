import threading
import time

import rospy
from mavros_msgs.msg import State

from to_B.check_goal_reached import GoalReached
from to_B.release_control import ControlRelease
from to_B.to_target_pose import TargetPose


STATE_TOPIC = "/mavros/state"
CONTROL_MODE = "OFFBOARD"
CONNECTION_TIMEOUT_S = 5.0
ARRIVAL_TIMEOUT_S = 120.0
POLL_INTERVAL_S = 0.1


class FlightControl:
    def __init__(self):
        self.mode = None
        self.interrupted = threading.Event()
        self.release = ControlRelease()
        self.goal = TargetPose()
        self.feedback = GoalReached()
        self.state = rospy.Subscriber(
            STATE_TOPIC,
            State,
            self.update_mode,
            queue_size=1,
        )

    def update_mode(self, message):
        if (
            self.mode == CONTROL_MODE
            and message.mode != CONTROL_MODE
        ):
            self.interrupted.set()
            rospy.logwarn(f"Left {CONTROL_MODE}; detection mission interrupted")
        self.mode = message.mode

    def is_offboard(self):
        return (
            self.mode == CONTROL_MODE
            and not self.interrupted.is_set()
            and not rospy.is_shutdown()
        )

    def go_to_pose(self, pose):
        # Stage 1: Wait for the mission interfaces; never request a flight mode change.
        deadline = time.monotonic() + CONNECTION_TIMEOUT_S
        while (
            not self.release.connected()
            or not self.goal.connected()
            or not self.feedback.connected()
        ):
            if not self.is_offboard():
                return False
            if time.monotonic() >= deadline:
                raise RuntimeError("Mission target, release or arrival interface is disconnected")
            rospy.sleep(POLL_INTERVAL_S)
        if not self.is_offboard():
            return False

        # Stage 2: Send the requested SLAM pose, acquiring or retaining task control.
        # Clear before sending so even an immediate True response is captured.
        self.feedback.reset()
        try:
            if not self.is_offboard():
                return False
            self.goal.send(pose)
            print("Target pose sent; waiting for arrival feedback...", flush=True)

            # Stage 3: Wait for position and yaw feedback; abort if OFFBOARD is lost.
            deadline = time.monotonic() + ARRIVAL_TIMEOUT_S
            while self.is_offboard():
                if time.monotonic() >= deadline:
                    raise RuntimeError("Timed out waiting for goal arrival feedback")
                if self.feedback.wait(POLL_INTERVAL_S):
                    if not self.is_offboard():
                        return False
                    print("Goal reached.", flush=True)
                    return True
            return False
        finally:
            self.feedback.reset(waiting=False)

    def release_control(self):
        # Stage 4: After all extra operations, return control without changing modes.
        if not self.is_offboard():
            return False
        if not self.release.connected():
            raise RuntimeError("Mission release topic has no subscriber")
        self.release.send()
        print("Control release sent; returning to patrol detection.", flush=True)
        return True

    def close(self):
        self.state.unregister()
        self.feedback.close()
        self.goal.close()
        self.release.close()
