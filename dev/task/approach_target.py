import math

import numpy as np
import rospy

from hardware.pose import get_pose


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
    yaw_deg = math.degrees(math.atan2(direction[1], direction[0]))
    if distance <= TARGET_DISTANCE_M:
        rospy.logwarn(f"Target is within {TARGET_DISTANCE_M:g} m; keeping position and aligning yaw")
        goal_position = position
    else:
        goal_position = target_position - direction * (TARGET_DISTANCE_M / distance)
        goal_position[2] = max(MIN_HEIGHT_M, goal_position[2])
    return {
        "frame_id": current_pose["frame_id"],
        "position_m": goal_position.tolist(),
        "yaw_deg": yaw_deg,
    }


def prepare(targets):
    # Compute every goal from the same measured pose before visiting any target.
    current_pose, _ = get_pose()
    goals = []
    for target in targets:
        goal = make_pose(current_pose, target)
        goals.append({"target": target, "pose": goal})
    # Visit fixed approach points from nearest to farthest relative to the planning pose.
    position = np.asarray(current_pose["position_m"])
    return sorted(goals, key=lambda item: np.linalg.norm(
        np.asarray(item["pose"]["position_m"]) - position))
