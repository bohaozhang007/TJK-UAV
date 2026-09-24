import sys
from pathlib import Path

import rospy


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from util.flight import FlightControl
from task.approach_target import prepare as prepare_approaches
from task.approach_target import run as approach_target
from task.detection import Detection
from task.return_and_resume import run as return_and_resume
from task.return_to_exposure import run as return_to_exposure


POLL_INTERVAL_S = 0.1


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
            if not return_to_exposure(flight, exposure_pose):
                break
            goals = prepare_approaches(targets)
            if not approach_target(flight, goals):
                break
            if not return_and_resume(flight, exposure_pose):
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
