import math

import numpy as np
import rospy

from hardware.pose import close_pose, get_pose


TARGET_DISTANCE_M = 2.0
MIN_HEIGHT_M = 0.5


def make_pose(current_pose, target):
    if current_pose["frame_id"] != target["world_frame"]:
        raise ValueError("Drone and target positions must use the same SLAM frame")
    position = np.asarray(current_pose["position_m"], dtype=float)
    target_position = np.asarray(target["position_world_m"], dtype=float)
    if (
        position.shape != (3,)
        or target_position.shape != (3,)
        or not np.isfinite(position).all()
        or not np.isfinite(target_position).all()
    ):
        raise ValueError("Drone and target positions must contain three finite coordinates")
    direction = target_position - position
    distance = np.linalg.norm(direction)
    if distance <= TARGET_DISTANCE_M:
        rospy.logwarn("Target is within 2 m; skipping approach")
        return None
    if np.hypot(direction[0], direction[1]) == 0:
        # A vertical target has no horizontal bearing; preserve the current yaw.
        yaw_deg = current_pose["yaw_deg"]
    else:
        yaw_deg = math.degrees(math.atan2(direction[1], direction[0]))
    goal_position = target_position - direction * (TARGET_DISTANCE_M / distance)
    goal_position[2] = max(MIN_HEIGHT_M, goal_position[2])
    return {
        "frame_id": current_pose["frame_id"],
        "position_m": goal_position.tolist(),
        "yaw_deg": yaw_deg,
    }


def prepare(targets):
    # Compute every goal from the same measured pose before visiting any target.
    try:
        current_pose, _ = get_pose()
    finally:
        close_pose()
    goals = []
    for target in targets:
        goal = make_pose(current_pose, target)
        goals.append({"target": target, "pose": goal, "hold_pose": current_pose})
    # Visit fixed approach points from nearest to farthest relative to the planning pose.
    position = np.asarray(current_pose["position_m"])
    return sorted(goals, key=lambda item: np.linalg.norm(
        np.asarray(item["pose"]["position_m"]) - position) if item["pose"] is not None else 0.0)


def run(flight, plan):
    if not flight.is_offboard():
        return False
    if plan["pose"] is None:
        # Acquire control without approaching when the target is already close.
        return flight.go_to_pose(plan["hold_pose"])
    return flight.go_to_pose(plan["pose"])
