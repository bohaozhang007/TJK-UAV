import argparse
import sys
import os
import threading
import traceback
from pathlib import Path

import rospy

from mavros_msgs.msg import State


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from util.timing import timed, timed_step
from util.flight import FlightControl
from util.terminal_log import save_terminal_output
from hardware.k40t import close_camera, get_img, detect_img, disable_osd
from hardware.pose import close_pose
from task.approach_target import prepare as prepare_observation_points
from task.detect_thread import DetectThread
from task.observe_targets import run as observe_current_or_return


POLL_INTERVAL_S = 0.1
STATE_TOPIC = "/mavros/state"
CONTROL_MODE = "OFFBOARD"
CAPTURE_COUNT = 1
RELEASE_COOLDOWN_S = 1.0


@timed("uav.capture")
def capture_images(target_id):
    for index in range(CAPTURE_COUNT):
        rospy.loginfo(f"Target {target_id} capture {index + 1}/{CAPTURE_COUNT}: reading image")
        img, _ = get_img()
        rospy.loginfo(f"Target {target_id} capture {index + 1}/{CAPTURE_COUNT}: sending image")
        detect_img(
            img,
            calculate_depth=False,
            target_id=target_id,
            frame_id=index + 1,
        )
        rospy.loginfo(f"Target {target_id} capture {index + 1}/{CAPTURE_COUNT}: complete")


@timed("uav.mission")
def main(skip_exposure_point=False, autofocus=False):
    rospy.init_node("buaa_vln", disable_signals=True)
    disable_osd()
    # ROS shutdown terminates this UAV process and all its threads immediately.
    rospy.on_shutdown(lambda: os._exit(0))
    flight = FlightControl()
    detect_thread = DetectThread()
    state = None
    ready = threading.Event()
    close_lock = threading.Lock()

    def close():
        # Serialize cleanup if both the callback and main thread enter here.
        with close_lock:
            try:
                if state is not None:
                    state.unregister()
            finally:
                try:
                    detect_thread.close()
                finally:
                    try:
                        close_camera()
                    finally:
                        try:
                            close_pose()
                        finally:
                            try:
                                flight.close()
                            finally:
                                rospy.signal_shutdown("Detection task stopped")

    def update_state(message):
        if message.mode != CONTROL_MODE:
            rospy.loginfo(f"Stopping detection task: flight mode changed to {message.mode}")
            close()
        else:
            ready.set()

    stage = "waiting for OFFBOARD mode"
    target_id = None
    try:
        state = rospy.Subscriber(STATE_TOPIC, State, update_state, queue_size=1)
        while not rospy.is_shutdown():
            if ready.wait(POLL_INTERVAL_S):
                break
        while not rospy.is_shutdown():
            # Stage 1: Detect new targets and pause detection when found.
            stage = "detecting targets"
            target_id = None
            result = detect_thread.find_targets()
            if result is None:
                break
            exposure_pose, next_waypoint, targets = result
            for target in targets:
                rospy.loginfo(f"Target {target['target_id']} detected: position={target['position_world_m']}")

            if autofocus:
                stage = "holding, rechecking and capturing targets"
                observe_current_or_return(
                    flight, detect_thread, exposure_pose, targets, capture_images,
                )
            else:
                if not skip_exposure_point:
                    stage = "returning to exposure pose"
                    flight.go_to_pose(exposure_pose)
                # Plan observation poses and visit each one.
                stage = "planning observation poses"
                plans = prepare_observation_points(
                    exposure_pose,
                    next_waypoint,
                    targets,
                )
                for plan in plans:
                    target_id = plan["target"]["target_id"]
                    stage = "moving to observation pose"
                    detect_thread.start_attempt(plan["target"])
                    flight.go_to_pose(plan["pose"])
                    target_id = plan["target"]["target_id"]
                    stage = "capturing target images"
                    capture_images(target_id)
                    detect_thread.mark_observed(plan["target"])

                # Return to the recorded pose after all visits.
                stage = "returning to exposure pose"
                flight.go_to_pose(exposure_pose)

            # Stage 5: Release mission control, then restart detection.
            stage = "releasing flight control"
            flight.release_control()
            with timed_step("uav.patrol_cooldown", configured_s=RELEASE_COOLDOWN_S):
                rospy.sleep(RELEASE_COOLDOWN_S)
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    except Exception as exc:
        rospy.logerr(
            f"Detection task failed: stage={stage}, target={target_id}, autofocus={autofocus}; "
            f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}"
        )
        raise
    finally:
        close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--autofocus",
        action="store_true",
        help="Capture all pending targets at the current pose only on full coverage; otherwise return the entire original batch",
    )
    parser.add_argument(
        "--skip-exposure-point",
        action="store_true",
        help="Skip the initial return to the exposure pose; always return after all visits",
    )
    args = parser.parse_args(rospy.myargv()[1:])
    with save_terminal_output():
        main(
            skip_exposure_point=args.skip_exposure_point,
            autofocus=args.autofocus,
        )
