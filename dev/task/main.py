import sys
import os
import threading
from pathlib import Path

import rospy
from mavros_msgs.msg import State


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from util.flight import FlightControl
from hardware.k40t import close_camera
from hardware.pose import close_pose
from task.approach_target import prepare as prepare_approaches
from task.detection import Detection


POLL_INTERVAL_S = 0.1
STATE_TOPIC = "/mavros/state"
CONTROL_MODE = "OFFBOARD"


def main():
    rospy.init_node("i7_detection_main", disable_signals=True)
    # ROS shutdown terminates this UAV process and all its threads immediately.
    rospy.on_shutdown(lambda: os._exit(0))
    flight = FlightControl()
    detection = Detection()
    state = None
    ready = threading.Event()

    def update_state(message):
        if message.mode != CONTROL_MODE:
            # Terminate this UAV process and every thread without further commands.
            os._exit(0)
        ready.set()

    try:
        state = rospy.Subscriber(STATE_TOPIC, State, update_state, queue_size=1)
        while not rospy.is_shutdown():
            if ready.wait(POLL_INTERVAL_S):
                break
        if rospy.is_shutdown():
            return
        while not rospy.is_shutdown():
            result = detection.find_targets()
            if result is None:
                break
            exposure_pose, targets = result
            flight.go_to_pose(exposure_pose)
            plans = prepare_approaches(targets)
            for plan in plans:
                flight.go_to_pose(plan["pose"])
            flight.go_to_pose(exposure_pose)
            # Release mission control after every successful task batch.
            flight.release_control()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    finally:
        if state is not None:
            state.unregister()
        try:
            detection.pause()
        finally:
            try:
                close_camera()
            finally:
                try:
                    close_pose()
                finally:
                    flight.close()
                    rospy.signal_shutdown("Detection task stopped")


if __name__ == "__main__":
    main()
