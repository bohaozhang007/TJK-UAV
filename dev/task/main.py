import sys
import os
import threading
from pathlib import Path
from itertools import groupby

import rospy
from mavros_msgs.msg import State


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from util.flight import FlightControl
from hardware.k40t import check_tracker
from task.approach_target import prepare as prepare_approaches
from task.approach_target import run as approach
from task.autofocus import run as autofocus
from task.config import TASKS
from task.detection import Detection


POLL_INTERVAL_S = 0.1
STATE_TOPIC = "/mavros/state"
CONTROL_MODE = "OFFBOARD"


def run_tasks(
    flight,
    exposure_pose,
    targets,
):
    for per_target, group in groupby(TASKS, key=lambda task: task in (approach, autofocus)):
        tasks = tuple(group)
        if per_target:
            # Plan the entire batch from the current pose after any configured return.
            plans = prepare_approaches(targets)
            for plan in plans:
                for task in tasks:
                    if not task(flight, plan):
                        return False
        else:
            for task in tasks:
                if not task(flight, exposure_pose):
                    return False
    return True


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
        if autofocus in TASKS:
            check_tracker()
        while not rospy.is_shutdown():
            result = detection.find_targets()
            if result is None:
                break
            exposure_pose, targets = result
            if not run_tasks(
                flight,
                exposure_pose,
                targets,
            ):
                break
            # Release mission control after every successful task batch.
            if not flight.release_control():
                break
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    finally:
        if state is not None:
            state.unregister()
        try:
            detection.pause()
        finally:
            flight.close()
            rospy.signal_shutdown("Detection task stopped")


if __name__ == "__main__":
    main()
