import sys
from pathlib import Path
from itertools import groupby

import rospy


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from util.flight import FlightControl
from task.approach_target import prepare as prepare_approaches
from task.approach_target import run as approach
from task.autofocus import run as autofocus
from task.config import TASKS
from task.detection import Detection
from task.return_and_resume import run as return_and_resume


POLL_INTERVAL_S = 0.1


def run_tasks(
    flight,
    exposure_pose,
    targets,
):
    released = False
    for per_target, group in groupby(TASKS, key=lambda task: task in (approach, autofocus)):
        tasks = tuple(group)
        if per_target:
            # Plan the entire batch from the current pose after any configured return.
            plans = prepare_approaches(targets)
            for plan in plans:
                for task in tasks:
                    if not task(flight, plan):
                        return False
            released = False
        else:
            for task in tasks:
                if not task(flight, exposure_pose):
                    return False
                released = task is return_and_resume
    # Resume detection only after a configured task releases mission control.
    return released


def main():
    rospy.init_node("i7_detection_main", disable_signals=True)
    flight = FlightControl()
    detection = Detection()
    try:
        while not rospy.is_shutdown():
            if flight.interrupted.is_set():
                break
            if not flight.is_offboard():
                rospy.sleep(POLL_INTERVAL_S)
                continue
            result = detection.find_targets(flight)
            if result is None:
                break
            exposure_pose, targets = result
            if not run_tasks(
                flight,
                exposure_pose,
                targets,
            ):
                break
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    finally:
        try:
            detection.pause()
        finally:
            flight.close()
            rospy.signal_shutdown("Detection task stopped")


if __name__ == "__main__":
    main()
