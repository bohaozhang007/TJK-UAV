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
from task.approach_target import prepare as prepare_observation_points
from task.detect_thread import DetectThread


POLL_INTERVAL_S = 0.1
STATE_TOPIC = "/mavros/state"
CONTROL_MODE = "OFFBOARD"


def main():
    rospy.init_node("buaa_vln", disable_signals=True)
    # ROS shutdown terminates this UAV process and all its threads immediately.
    rospy.on_shutdown(lambda: os._exit(0))
    flight = FlightControl()
    detect_thread = DetectThread()
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
        while not rospy.is_shutdown():
            # Stage 1: Detect new targets and pause detection when found.
            result = detect_thread.find_targets()
            if result is None:
                break
            exposure_pose, targets = result

            # Stage 2: Return to the pose recorded with the detection image.
            flight.go_to_pose(exposure_pose)

            # Stage 3: Plan observation poses and visit each one.
            plans = prepare_observation_points(targets)
            for plan in plans:
                flight.go_to_pose(plan["pose"])

            # Stage 4: Return to the recorded pose after all visits.
            flight.go_to_pose(exposure_pose)

            # Stage 5: Release mission control, then restart detection.
            flight.release_control()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    finally:
        if state is not None:
            state.unregister()
        detect_thread.pause()
        close_camera()
        close_pose()
        flight.close()
        rospy.signal_shutdown("Detection task stopped")


if __name__ == "__main__":
    main()
