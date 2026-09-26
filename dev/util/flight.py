import time

from interface.check_goal_reached import GoalReached
from interface.release_control import ControlRelease
from interface.to_target_pose import TargetPose


CONNECTION_TIMEOUT_S = 5.0
ARRIVAL_TIMEOUT_S = 120.0
POLL_INTERVAL_S = 0.1


class FlightControl:
    def __init__(self):
        self.release = ControlRelease()
        self.goal = TargetPose()
        self.feedback = GoalReached()

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
            self.goal.send(pose)
            print("Target pose sent; waiting for arrival feedback...", flush=True)

            # Stage 3: Wait for position and yaw feedback.
            deadline = time.monotonic() + ARRIVAL_TIMEOUT_S
            while True:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"Goal arrival feedback timeout of {ARRIVAL_TIMEOUT_S:g} s exceeded")
                if self.feedback.wait(POLL_INTERVAL_S):
                    print("Goal reached.", flush=True)
                    return True
        finally:
            self.feedback.reset()

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
