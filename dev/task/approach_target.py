import math

import numpy as np
import rospy

MIN_HEIGHT_M = 0.5
MIN_RADIUS_M = 1.5
SIDE_ANGLE_DEG = 30


def make_pose(
    exposure_pose,
    next_waypoint,
    target,
):
    if exposure_pose["frame_id"] != target["world_frame"]:
        raise ValueError("Drone and target positions must use the same SLAM frame")
    position = np.asarray(exposure_pose["position_m"], dtype=float)
    waypoint_position = np.asarray(next_waypoint["position_m"], dtype=float)
    target_position = np.asarray(target["position_world_m"], dtype=float)
    if (
        position.shape != (3,)
        or waypoint_position.shape != (3,)
        or target_position.shape != (3,)
        or not np.isfinite(position).all()
        or not np.isfinite(waypoint_position).all()
        or not np.isfinite(target_position).all()
    ):
        raise ValueError("Drone, waypoint and target positions must contain three finite coordinates")
    pairs = [(position, waypoint_position)]
    previous = next_waypoint.get("previous_position_m")
    if previous is not None:
        previous_position = np.asarray(previous, dtype=float)
        pairs.extend([(previous_position, position), (previous_position, waypoint_position)])
    start, end = max(pairs, key=lambda pair: np.linalg.norm(pair[1][:2] - pair[0][:2]))
    direction = end - start
    ratio = np.dot(target_position - start, direction) / np.dot(direction, direction)
    goal_position = start + ratio * direction
    goal_position[2] = max(MIN_HEIGHT_M, target_position[2])
    toward_target = target_position - goal_position
    yaw_deg = math.degrees(math.atan2(toward_target[1], toward_target[0]))
    return {
        "frame_id": exposure_pose["frame_id"],
        "position_m": goal_position.tolist(),
        "yaw_deg": yaw_deg,
    }


def prepare(
    exposure_pose,
    next_waypoint,
    targets,
):
    # Use the exposure pose and route waypoint paired with the detection image.
    goals = []
    for index, target in enumerate(targets, start=1):
        goal = make_pose(
            exposure_pose,
            next_waypoint,
            target,
        )
        goals.append({"target": target, "pose": goal, "target_id": target.get("target_id", index)})
    # Sort targets by their foot points, keeping each target's three poses together.
    position = np.asarray(exposure_pose["position_m"])
    goals = sorted(goals, key=lambda item: np.linalg.norm(
        np.asarray(item["pose"]["position_m"]) - position))
    plans = []
    for item in goals:
        target = item["target"]
        goal = item["pose"]
        target_position = np.asarray(target["position_world_m"], dtype=float)
        offset = np.asarray(goal["position_m"])[:2] - target_position[:2]
        radius = np.linalg.norm(offset)
        if 0 < radius < MIN_RADIUS_M:
            offset = offset * (MIN_RADIUS_M / radius)
        for angle in (0, SIDE_ANGLE_DEG, -SIDE_ANGLE_DEG):
            radians = math.radians(angle)
            cosine, sine = math.cos(radians), math.sin(radians)
            rotated = np.array([
                cosine * offset[0] - sine * offset[1],
                sine * offset[0] + cosine * offset[1],
            ])
            point = target_position[:2] + rotated
            pose = {
                "frame_id": goal["frame_id"],
                "position_m": [float(point[0]), float(point[1]), goal["position_m"][2]],
                "yaw_deg": math.degrees(math.atan2(-rotated[1], -rotated[0])),
            }
            rospy.loginfo(
                f"Target {item['target_id']} planned: angle={angle:+g}, "
                f"position={pose['position_m']}, yaw={pose['yaw_deg']:.2f}"
            )
            plans.append({"target": target, "pose": pose})
    return plans
