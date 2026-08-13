# v19：所有 Agent 运动统一使用 XYZ+Yaw 组合命令，纯旋转补偿前进 1 cm。

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import datetime as dt
import importlib
import logging
import math
import os
import sys
import time
from enum import Enum
from pathlib import Path
from typing import Optional
from PIL import Image

import cv2
import numpy as np
import yaml

SRC_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
sys.path.insert(0, SRC_ROOT)

from robot_client.base import BaseClient
from utils import show_fig


def _load_config(config_path: str | Path) -> dict:
    path = Path(config_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Agent config file not found: {path}")
    try:
        with path.open("r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"Failed to load required agent config: {path}") from exc
    if not isinstance(config, dict):
        raise ValueError(f"Agent config must contain a mapping: {path}")
    return config


def _config_section(config: dict, section_name: str) -> dict:
    section = config.get(section_name)
    if not isinstance(section, dict):
        raise ValueError(f"Missing or invalid config section: {section_name}")
    return section


def _config_number(
    section: dict,
    key: str,
    *,
    integer: bool = False,
    minimum: float | None = None,
    maximum: float | None = None,
):
    if key not in section:
        raise ValueError(f"Missing config value: {key}")
    value = section[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Config value {key} must be numeric, got {value!r}")
    if integer and float(value) != int(value):
        raise ValueError(f"Config value {key} must be an integer, got {value!r}")
    result = int(value) if integer else float(value)
    if minimum is not None and result < minimum:
        raise ValueError(f"Config value {key} must be >= {minimum}, got {result}")
    if maximum is not None and result > maximum:
        raise ValueError(f"Config value {key} must be <= {maximum}, got {result}")
    return result


class AgentAction(str, Enum):
    STABLE = "stable"
    ROTATE = "rotate"
    XYZ_YAW_HYBRID = "xyz_yaw_hybrid"


class MissionState(str, Enum):
    INITIALIZING = "INITIALIZING"
    READY = "READY"
    TRANSIT_TO_WAYPOINT = "TRANSIT_TO_WAYPOINT"
    EXECUTING_WAYPOINT = "EXECUTING_WAYPOINT"
    RETURNING_HOME = "RETURNING_HOME"
    LANDING = "LANDING"
    COMPLETED = "COMPLETED"
    EMERGENCY_LANDING = "EMERGENCY_LANDING"
    ABORTED = "ABORTED"


class WaypointState(str, Enum):
    PREPARING = "PREPARING"
    SEARCH = "SEARCH"
    SELECT = "SELECT"
    TRACK = "TRACK"
    SCAN = "SCAN"
    RETURN_WAYPOINT = "RETURN_WAYPOINT"
    SUCCEEDED = "SUCCEEDED"
    TASK_FAILED = "TASK_FAILED"


class TaskFailure(RuntimeError):
    """The task cannot continue, while controlled return is still possible."""


class FlightSafetyError(RuntimeError):
    """Positioning, control, health, or Robot Server capability is unavailable."""


def _timestamped_experiment_dir(exp_name: str, timestamp: str) -> Path:
    return Path(f"{Path(exp_name).expanduser()}_{timestamp}")


def _configure_logger(log_path: Path) -> logging.Logger:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("tjk_v19")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    logger.handlers.clear()

    formatter = logging.Formatter(
        "%(asctime)s.%(msecs)03d %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)
    logger.addHandler(file_handler)
    return logger

class TJKAgent:
    MIN_TARGET_WORLD_Z_CM = 50.0
    PURE_ROTATE_FORWARD_CM = 1.0
    ACTION_SLEEP_S = 2.0

    def __init__(
        self,
        client: BaseClient,
        detector,
        tracker,
        config: dict,
        detector_name: str,
        tracker_name: str,
        vis_dir: str = './tjk_vis',
        save_vis: bool = True,
        save_depth: bool = False,
        logger: logging.Logger | None = None,
    ):
        self.client = client
        self.logger = logger
        self.position_tolerance_cm = None
        self.yaw_tolerance_deg = None
        self.motion_tolerance_source = None
        self._command_idx = 0
        self.last_motion_error = None
        self._last_motion_timing = {}
        self._last_motion_command_id = None
        self._mission_vis_dir = Path(vis_dir).expanduser().resolve()
        self.vis_dir = str(self._mission_vis_dir)
        self.save_vis = save_vis
        self.save_depth = save_depth
        self.mission_state = MissionState.INITIALIZING
        self.state = WaypointState.PREPARING
        self.mission_origin_pose = None
        self.current_waypoint_index = None
        self.current_waypoint = None
        self.waypoint_results = []
        self.waypoint_pose = None
        self.search_candidates = []
        self.ranked_search_candidates = []
        self.selected_candidate = None
        self.last_track_observation = None
        self.detector = detector
        self.tracker = tracker
        self.detector_name = detector_name
        self.tracker_name = tracker_name
        mission_config = _config_section(config, "mission")
        self.mission_frame = str(mission_config.get("frame", ""))
        if self.mission_frame != "takeoff_world":
            raise ValueError(
                "mission.frame must be 'takeoff_world', got "
                f"{self.mission_frame!r}"
            )
        if mission_config.get("position_unit") != "cm":
            raise ValueError("mission.position_unit must be 'cm'")
        if mission_config.get("yaw_unit") != "deg":
            raise ValueError("mission.yaw_unit must be 'deg'")
        self.task_failure_policy = str(
            mission_config.get("on_task_failure", "")
        )
        if self.task_failure_policy not in {"continue", "return_home"}:
            raise ValueError(
                "mission.on_task_failure must be 'continue' or "
                f"'return_home', got {self.task_failure_policy!r}"
            )
        self.mission_waypoints = self._parse_mission_waypoints(
            mission_config.get("waypoints")
        )
        motion_config = _config_section(config, "motion")
        search_config = _config_section(config, "search")
        detection_config = _config_section(config, "detection")
        select_config = _config_section(config, "select")
        safety_config = _config_section(config, "safety")
        track_config = _config_section(config, "track")
        scan_config = _config_section(config, "scan")
        # Forward/backward action.
        self.max_fb_step_cm = _config_number(
            motion_config, "max_fb_step_cm", minimum=1e-6
        )
        self.fb_stop_step_cm = _config_number(
            motion_config, "fb_stop_step_cm", minimum=0.0
        )
        self.backward_ratio = _config_number(
            motion_config,
            "backward_ratio",
            minimum=0.0,
            maximum=1.0,
        )

        # Vertical action.
        self.z_cm_per_pixel = _config_number(
            motion_config, "z_cm_per_pixel", minimum=1e-6
        )
        self.max_z_step_cm = _config_number(
            motion_config, "max_z_step_cm", minimum=1e-6
        )
        # Yaw action.
        self.rotate_deg_per_pixel = _config_number(
            motion_config, "rotate_deg_per_pixel", minimum=1e-6
        )
        self.max_rotate_deg = _config_number(
            motion_config, "max_rotate_deg", minimum=1e-6
        )
        if self.fb_stop_step_cm > self.max_fb_step_cm:
            raise ValueError(
                "motion.fb_stop_step_cm cannot exceed max_fb_step_cm"
            )

        self.search_rotation_step_deg = _config_number(
            search_config,
            "rotation_step_deg",
            minimum=1.0,
            maximum=360.0,
        )
        search_rotation_count = 360.0 / self.search_rotation_step_deg
        if abs(search_rotation_count - round(search_rotation_count)) > 1e-6:
            raise ValueError(
                "search.rotation_step_deg must divide 360 exactly, got "
                f"{self.search_rotation_step_deg}"
            )
        self.search_rotation_count = int(round(search_rotation_count))
        self.search_height_offset_cm = _config_number(
            search_config,
            "height_offset_cm",
            integer=True,
            minimum=0.0,
        )
        self.max_candidates_per_frame = _config_number(
            detection_config,
            "max_candidates_per_frame",
            integer=True,
            minimum=1.0,
        )
        self.select_dedup_iou_threshold = _config_number(
            select_config,
            "dedup_iou_threshold",
            minimum=0.0,
            maximum=1.0,
        )
        self.select_min_edge_margin_ratio = _config_number(
            select_config,
            "min_edge_margin_ratio",
            minimum=0.0,
            maximum=0.5,
        )
        self.select_area_top_k = _config_number(
            select_config,
            "area_top_k",
            integer=True,
            minimum=1.0,
        )
        self.select_confidence_top_k = _config_number(
            select_config,
            "confidence_top_k",
            integer=True,
            minimum=1.0,
        )
        self.select_max_view_attempts = _config_number(
            select_config,
            "max_view_attempts",
            integer=True,
            minimum=1.0,
        )

        # safe thresh
        self.safe_z_cm = _config_number(
            safety_config, "safe_z_cm", minimum=0.0
        )
        if self.safe_z_cm < self.MIN_TARGET_WORLD_Z_CM:
            raise ValueError(
                "safety.safe_z_cm must be >= "
                f"{self.MIN_TARGET_WORLD_Z_CM:.2f}cm"
            )
        self.depth_safe_distance_cm = _config_number(
            safety_config,
            "depth_safe_distance_cm",
            minimum=0.0,
        )

        # img setting
        # connect() replaces these fallbacks with the native camera frame size.
        self.img_height = 360
        self.img_width = 640
        self.horizontal_center = self.img_width / 2
        self.vertical_center = self.img_height / 2

        # stop thresh
        self.target_stop_ratio = _config_number(
            track_config,
            "target_stop_ratio",
            minimum=1e-6,
            maximum=1.0,
        )
        self.yaw_only_threshold_deg = _config_number(
            track_config,
            "yaw_only_threshold_deg",
            minimum=0.0,
            maximum=self.max_rotate_deg,
        )

        # max exec iters
        self.max_exec_iters = _config_number(
            track_config,
            "max_exec_iters",
            integer=True,
            minimum=1.0,
        )

        # scan settings
        self.scan_num_points = _config_number(
            scan_config,
            "num_points",
            integer=True,
            minimum=12.0,
            maximum=12.0,
        )
        self.scan_radius_cm = _config_number(
            scan_config,
            "radius_cm",
            minimum=1e-6,
        )
        self.scan_yaw_unit_deg = _config_number(
            scan_config,
            "yaw_unit_deg",
            minimum=1e-6,
        )

    @staticmethod
    def _parse_mission_waypoints(raw_waypoints):
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise ValueError("mission.waypoints must be a non-empty list")

        waypoints = []
        names = set()
        required_keys = {"name", "x_cm", "y_cm", "z_cm", "yaw_deg"}
        for index, raw_waypoint in enumerate(raw_waypoints):
            if not isinstance(raw_waypoint, dict):
                raise ValueError(
                    f"mission.waypoints[{index}] must be a mapping"
                )
            missing = sorted(required_keys - raw_waypoint.keys())
            unknown = sorted(raw_waypoint.keys() - required_keys)
            if missing or unknown:
                details = []
                if missing:
                    details.append(f"missing={missing}")
                if unknown:
                    details.append(f"unknown={unknown}")
                raise ValueError(
                    f"Invalid mission.waypoints[{index}]: "
                    + " ".join(details)
                )

            name = str(raw_waypoint["name"]).strip()
            if not name:
                raise ValueError(
                    f"mission.waypoints[{index}].name cannot be empty"
                )
            if name in names:
                raise ValueError(f"Duplicate mission waypoint name: {name!r}")
            names.add(name)

            waypoint = {"name": name}
            for key in ("x_cm", "y_cm", "z_cm", "yaw_deg"):
                value = raw_waypoint[key]
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                ):
                    raise ValueError(
                        f"mission.waypoints[{index}].{key} must be a "
                        f"finite number, got {value!r}"
                    )
                waypoint[key] = float(value)
            waypoint["yaw_deg"] = TJKAgent._normalize_angle_deg(
                waypoint["yaw_deg"]
            )
            waypoints.append(waypoint)
        return waypoints

    def connect(self):
        self._set_mission_state(MissionState.INITIALIZING)
        self._call_flight_operation("start", self.client.start)
        self._configure_motion_tolerances(
            self._call_flight_operation(
                "motion tolerances",
                self.client.get_motion_tolerances,
            )
        )
        frame = self._capture_for_task(
            include_depth=False,
            context="connect RGB readiness",
        )
        height, width = frame.shape[:2]

        self.img_height = height
        self.img_width = width
        self.horizontal_center = width / 2
        self.vertical_center = height / 2

        self.tracker.set_img_size(height, width)
        print(f"[ROBOT-INFO] RGB resolution: {width}x{height}")

        pose = self._copy_pose(
            self._get_pose_for_flight("connect pose readiness")
        )
        self.mission_origin_pose = pose.copy()
        self._validate_mission_world_targets()

        x = float(pose["x"])
        y = float(pose["y"])
        z = float(pose["z"])
        yaw = float(pose["yaw"])
        pose_str = (
            f"x={x:.2f}cm, y={y:.2f}cm, z={z:.2f}cm, yaw={yaw:.2f}deg"
        )

        print(f"[ROBOT-INFO] Connected to {self.client.base_url}; pose: {pose_str}")

        self._set_mission_state(MissionState.READY)
        self._log(
            "[MISSION] Takeoff hover pose is mission origin "
            "(x=0.00cm, y=0.00cm, z=0.00cm, yaw=0.00deg)."
        )

    def _configure_motion_tolerances(self, tolerances):
        if not isinstance(tolerances, dict):
            raise FlightSafetyError(
                "motion tolerances response must be a mapping"
            )
        try:
            position_tolerance_cm = float(
                tolerances["position_tolerance_cm"]
            )
            yaw_tolerance_deg = float(tolerances["yaw_tolerance_deg"])
        except (KeyError, TypeError, ValueError) as exc:
            raise FlightSafetyError(
                "motion tolerances response is missing valid position/yaw values"
            ) from exc
        if (
            not math.isfinite(position_tolerance_cm)
            or position_tolerance_cm < 0.0
            or not math.isfinite(yaw_tolerance_deg)
            or yaw_tolerance_deg < 0.0
        ):
            raise FlightSafetyError(
                "motion tolerances must be finite and non-negative"
            )
        if self.yaw_only_threshold_deg < yaw_tolerance_deg:
            raise FlightSafetyError(
                "track.yaw_only_threshold_deg="
                f"{self.yaw_only_threshold_deg:.2f}deg must be >= Robot "
                f"Server yaw_tolerance_deg={yaw_tolerance_deg:.2f}deg"
            )

        self.position_tolerance_cm = position_tolerance_cm
        self.yaw_tolerance_deg = yaw_tolerance_deg
        self.motion_tolerance_source = str(
            tolerances.get("source", "unknown")
        )
        self._log(
            "[MOTION-TOLERANCE] "
            f"position={position_tolerance_cm:.2f}cm "
            f"yaw={yaw_tolerance_deg:.2f}deg "
            f"metric={tolerances.get('position_error_metric')!r} "
            f"source={self.motion_tolerance_source!r}"
        )

    def run_mission(self, detector_prompt):
        """Visit configured world waypoints, run each task, then return home."""
        if self.mission_origin_pose is None:
            raise FlightSafetyError("mission origin is unavailable; call connect first")

        self.waypoint_results = []
        for index, waypoint in enumerate(self.mission_waypoints):
            world_pose = self._mission_to_world_pose(waypoint)
            self._navigate_to_waypoint(index, waypoint, world_pose)
            self._set_mission_state(
                MissionState.EXECUTING_WAYPOINT,
                waypoint_index=index,
                waypoint_name=waypoint["name"],
            )
            succeeded = self.run(
                detector_prompt,
                waypoint_pose=world_pose,
                waypoint_index=index,
                waypoint_name=waypoint["name"],
            )
            final_pose = self._copy_pose(
                self._get_pose_for_flight(
                    f"waypoint {waypoint['name']} final pose"
                )
            )
            result = {
                "index": index,
                "name": waypoint["name"],
                "status": "succeeded" if succeeded else "task_failed",
                "mission_pose": self._world_to_mission_pose(final_pose),
            }
            self.waypoint_results.append(result)
            self._log(
                f"[MISSION] Waypoint {index + 1}/{len(self.mission_waypoints)} "
                f"name={waypoint['name']!r} status={result['status']}."
            )
            if not succeeded and self.task_failure_policy == "return_home":
                self._log(
                    "[MISSION] Task-failure policy requests an early "
                    "return home."
                )
                break

        self._return_home()
        return {
            "success": bool(
                len(self.waypoint_results) == len(self.mission_waypoints)
                and all(
                    result["status"] == "succeeded"
                    for result in self.waypoint_results
                )
            ),
            "returned_home": True,
            "waypoints": list(self.waypoint_results),
        }

    def _navigate_to_waypoint(self, index, waypoint, world_pose):
        self._set_mission_state(
            MissionState.TRANSIT_TO_WAYPOINT,
            waypoint_index=index,
            waypoint_name=waypoint["name"],
        )
        self._log(
            f"[MISSION] Navigate to waypoint "
            f"{index + 1}/{len(self.mission_waypoints)} "
            f"name={waypoint['name']!r} mission_pose="
            f"{self._format_pose(self._waypoint_mission_pose(waypoint))}."
        )
        return self._navigate_to_world_pose(
            world_pose,
            context=f"navigate waypoint={waypoint['name']}",
        )

    def _return_home(self):
        if self.mission_origin_pose is None:
            raise FlightSafetyError("mission origin is unavailable")
        self._set_mission_state(MissionState.RETURNING_HOME)
        self._log(
            "[MISSION] All scheduled waypoint work is finished; return "
            "to mission origin."
        )
        return self._navigate_to_world_pose(
            self.mission_origin_pose,
            context="return home",
        )

    def _navigate_to_world_pose(self, target_pose, context):
        """Blocking navigation to one fixed world pose."""
        self._ensure_flight_safety(f"before {context}")
        self._move_to_pose_hybrid(target_pose, context=context)
        self._ensure_flight_safety(f"after {context}")
        actual_pose = self._copy_pose(
            self._get_pose_for_flight(f"verify {context}")
        )
        pose_error = self._calculate_pose_error(target_pose, actual_pose)
        self._log(
            f"[MISSION] Navigation completed context={context!r} "
            f"target={self._format_pose(target_pose)} "
            f"actual={self._format_pose(actual_pose)} "
            f"error={self._format_motion_error(pose_error)}"
        )
        return actual_pose

    def _prepare_waypoint_context(self, index, name):
        self.current_waypoint_index = index
        self.current_waypoint = name
        self.search_candidates = []
        self.ranked_search_candidates = []
        self.selected_candidate = None
        self.last_track_observation = None
        self.last_motion_error = None
        self.state = WaypointState.PREPARING

        safe_name = "".join(
            char if char.isalnum() or char in {"-", "_"} else "_"
            for char in str(name)
        ).strip("_") or "waypoint"
        waypoint_dir = self._mission_vis_dir / (
            f"waypoint_{index + 1:02d}_{safe_name}"
        )
        waypoint_dir.mkdir(parents=True, exist_ok=True)
        self.vis_dir = str(waypoint_dir)
        reset = getattr(self.tracker, "reset", None)
        if callable(reset):
            reset()
        set_vis_dir = getattr(self.tracker, "set_vis_dir", None)
        if callable(set_vis_dir):
            set_vis_dir(self.vis_dir)
        detector_set_vis_dir = getattr(self.detector, "set_vis_dir", None)
        if callable(detector_set_vis_dir):
            detector_set_vis_dir(self.vis_dir)
        set_track_vis_naming = getattr(
            self.tracker,
            "set_track_vis_naming",
            None,
        )
        if callable(set_track_vis_naming):
            set_track_vis_naming("track", index_width=2)

    def _validate_mission_world_targets(self):
        origin = self.mission_origin_pose
        if origin is None:
            raise ValueError("mission origin is unavailable")
        targets = [
            ("mission origin", origin),
            *[
                (waypoint["name"], self._mission_to_world_pose(waypoint))
                for waypoint in self.mission_waypoints
            ],
        ]
        for name, target in targets:
            if float(target["z"]) < self.safe_z_cm:
                raise ValueError(
                    f"Mission target {name!r} world z={target['z']:.2f}cm "
                    f"is below the {self.safe_z_cm:.2f}cm safety limit"
                )

    def _mission_to_world_pose(self, mission_pose):
        if self.mission_origin_pose is None:
            raise ValueError("mission origin is unavailable")
        origin = self.mission_origin_pose
        x_cm = float(mission_pose["x_cm"])
        y_cm = float(mission_pose["y_cm"])
        origin_yaw_rad = math.radians(float(origin["yaw"]))
        return {
            "x": (
                float(origin["x"])
                + math.cos(origin_yaw_rad) * x_cm
                - math.sin(origin_yaw_rad) * y_cm
            ),
            "y": (
                float(origin["y"])
                + math.sin(origin_yaw_rad) * x_cm
                + math.cos(origin_yaw_rad) * y_cm
            ),
            "z": float(origin["z"]) + float(mission_pose["z_cm"]),
            "yaw": self._normalize_angle_deg(
                float(origin["yaw"]) + float(mission_pose["yaw_deg"])
            ),
        }

    def _world_to_mission_pose(self, world_pose):
        if self.mission_origin_pose is None:
            raise ValueError("mission origin is unavailable")
        origin = self.mission_origin_pose
        world_dx_cm = float(world_pose["x"]) - float(origin["x"])
        world_dy_cm = float(world_pose["y"]) - float(origin["y"])
        origin_yaw_rad = math.radians(float(origin["yaw"]))
        return {
            "x": (
                math.cos(origin_yaw_rad) * world_dx_cm
                + math.sin(origin_yaw_rad) * world_dy_cm
            ),
            "y": (
                -math.sin(origin_yaw_rad) * world_dx_cm
                + math.cos(origin_yaw_rad) * world_dy_cm
            ),
            "z": float(world_pose["z"]) - float(origin["z"]),
            "yaw": self._normalize_angle_deg(
                float(world_pose["yaw"]) - float(origin["yaw"])
            ),
        }

    @staticmethod
    def _waypoint_mission_pose(waypoint):
        return {
            "x": float(waypoint["x_cm"]),
            "y": float(waypoint["y_cm"]),
            "z": float(waypoint["z_cm"]),
            "yaw": float(waypoint["yaw_deg"]),
        }

    def _set_mission_state(
        self,
        state,
        waypoint_index=None,
        waypoint_name=None,
    ):
        self.mission_state = state
        details = ""
        if waypoint_index is not None:
            details += f" waypoint={waypoint_index + 1}"
        if waypoint_name is not None:
            details += f" name={waypoint_name!r}"
        self._log(f"[MISSION-STATE] Enter {state.value}{details}")

    def run(
        self,
        detector_prompt,
        waypoint_pose=None,
        waypoint_index=None,
        waypoint_name=None,
    ):
        """Run one waypoint task and distinguish task from safety failures."""
        if waypoint_pose is None:
            self.waypoint_pose = self._copy_pose(
                self._get_pose_for_flight("save waypoint")
            )
        else:
            self.waypoint_pose = self._copy_pose(waypoint_pose)
        if waypoint_index is not None and waypoint_name is not None:
            self._prepare_waypoint_context(waypoint_index, waypoint_name)
        self._log(
            "[WAYPOINT] Task reference pose: "
            f"x={self.waypoint_pose['x']:.2f}cm, "
            f"y={self.waypoint_pose['y']:.2f}cm, "
            f"z={self.waypoint_pose['z']:.2f}cm, "
            f"yaw={self.waypoint_pose['yaw']:.2f}deg"
        )

        try:
            prompt_updated = self.detector.set_prompt(detector_prompt)
        except Exception as exc:
            return self._recover_from_task_failure(
                TaskFailure(f"detector prompt setup failed: {exc}")
            )
        cache_builds = getattr(self.detector, "prompt_cache_builds", None)
        cache_text = (
            f" cache_builds={cache_builds}"
            if cache_builds is not None
            else ""
        )
        self._log(
            f"[DETECTOR] backend={self.detector_name} "
            f"prompt={detector_prompt!r} prompt_updated={prompt_updated}"
            f"{cache_text}"
        )
        search_succeeded = False
        select_succeeded = False
        track_succeeded = False
        scan_succeeded = False
        self.state = WaypointState.SEARCH

        while True:
            self._log(f"[STATE] Enter {self.state.value}")
            try:
                if self.state == WaypointState.SEARCH:
                    search_succeeded = self._run_task_stage(
                        "search",
                        self.search,
                    )
                    self.state = (
                        WaypointState.SELECT
                        if search_succeeded
                        else WaypointState.RETURN_WAYPOINT
                    )
                elif self.state == WaypointState.SELECT:
                    select_succeeded = self._run_task_stage(
                        "select",
                        self.select,
                    )
                    self.state = (
                        WaypointState.TRACK
                        if select_succeeded
                        else WaypointState.RETURN_WAYPOINT
                    )
                elif self.state == WaypointState.TRACK:
                    track_succeeded = self._run_task_stage(
                        "track",
                        self.track,
                    )
                    self.state = (
                        WaypointState.SCAN
                        if track_succeeded
                        else WaypointState.RETURN_WAYPOINT
                    )
                elif self.state == WaypointState.SCAN:
                    scan_succeeded = self._run_task_stage(
                        "scan",
                        self.scan,
                    )
                    self.state = WaypointState.RETURN_WAYPOINT
                elif self.state == WaypointState.RETURN_WAYPOINT:
                    self.return_waypoint()
                    break
            except FlightSafetyError as exc:
                self._log(
                    f"[SAFETY] state={self.state.value} capability lost: {exc}; "
                    "skip waypoint return and request landing."
                )
                raise
            except TaskFailure as exc:
                return self._recover_from_task_failure(exc)
            except Exception as exc:
                safety_error = FlightSafetyError(
                    f"state={self.state.value} unclassified failure: {exc}"
                )
                self._log(
                    f"[SAFETY] {safety_error}; skip waypoint return and "
                    "request landing."
                )
                raise safety_error from exc

        succeeded = (
            search_succeeded
            and select_succeeded
            and track_succeeded
            and scan_succeeded
        )
        self._log(
            f"[RES] Waypoint task completed; candidates="
            f"{len(self.search_candidates)} target_selected={select_succeeded} "
            f"target_reached={track_succeeded} "
            f"scan_completed={scan_succeeded}."
        )
        self.state = (
            WaypointState.SUCCEEDED
            if succeeded
            else WaypointState.TASK_FAILED
        )
        return succeeded

    def _call_flight_operation(self, operation, callback):
        try:
            return callback()
        except FlightSafetyError:
            raise
        except Exception as exc:
            raise FlightSafetyError(f"{operation} failed: {exc}") from exc

    def _run_task_stage(self, stage, callback):
        try:
            return callback()
        except FlightSafetyError:
            raise
        except TaskFailure:
            raise
        except Exception as exc:
            raise TaskFailure(f"{stage} task processing failed: {exc}") from exc

    def _get_pose_for_flight(self, context):
        return self._call_flight_operation(
            context,
            self.client.get_pose,
        )

    def _flight_health(self, context):
        result = self._call_flight_operation(
            f"{context} health query",
            self.client.health,
        )
        if result.get("ok", True) is False:
            detail = result.get("error") or result.get("message") or result
            raise FlightSafetyError(f"{context} health failed: {detail}")
        health = result.get("health", result)
        if not isinstance(health, dict):
            raise FlightSafetyError(
                f"{context} health response does not contain a health object"
            )
        return health

    def _ensure_flight_safety(self, context):
        health = self._flight_health(context)
        missing = [
            name
            for name in ("initialized", "airborne")
            if health.get(name) is not True
        ]
        optional_capabilities = (
            "control_ready",
            "odom_ok",
            "rgb_ok",
            "control_link_ok",
            "goal_link_ok",
            "yaw_link_ok",
            "state_ok",
            "extended_state_ok",
        )
        missing.extend(
            name
            for name in optional_capabilities
            if name in health and health.get(name) is not True
        )
        if missing:
            raise FlightSafetyError(
                f"{context} flight capability unavailable: "
                + ", ".join(missing)
            )
        return health

    def _capture_for_task(self, include_depth, context):
        try:
            return self.client.capture(include_depth=include_depth)
        except FlightSafetyError:
            raise
        except Exception as exc:
            try:
                self._ensure_flight_safety(f"after {context} failure")
            except FlightSafetyError as safety_exc:
                raise FlightSafetyError(
                    f"{context} failed: {exc}; {safety_exc}"
                ) from exc
            raise TaskFailure(f"{context} failed: {exc}") from exc

    def _recover_from_task_failure(self, failure):
        self._log(
            f"[TASK-FAILURE] state={self.state.value}: {failure}; "
            "attempt waypoint return."
        )
        if self.waypoint_pose is None:
            self._log(
                "[TASK-FAILURE] Waypoint is unavailable; skip return and "
                "request landing."
            )
            return False

        self._ensure_flight_safety("before task-failure waypoint return")
        self.state = WaypointState.RETURN_WAYPOINT
        try:
            self.return_waypoint()
        except FlightSafetyError:
            raise
        except Exception as exc:
            raise FlightSafetyError(
                f"task-failure waypoint return failed: {exc}"
            ) from exc
        self.state = WaypointState.TASK_FAILED
        self._log(
            "[TASK-FAILURE] Waypoint return completed; mission policy "
            "will select the next action."
        )
        return False

    def search(self):
        """Capture views continuously while one detector worker consumes them."""
        self.search_candidates = []
        self.ranked_search_candidates = []
        self.selected_candidate = None
        self.last_track_observation = None
        search_started = time.perf_counter()
        pending_views = []
        executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="tjk-detector",
        )
        try:
            # Search at h, h+offset, and h-offset, one full turn at each height.
            for height_idx, height_offset_cm, target_z_cm in (
                self._search_height_targets()
            ):
                self._raise_search_worker_errors(pending_views)
                current_pose = self._get_pose_for_flight(
                    f"search height={height_idx} pose"
                )
                dz_cm = target_z_cm - float(current_pose["z"])
                if abs(dz_cm) > 1e-6:
                    self.exec_xyz_yaw_hybrid(
                        0.0,
                        0.0,
                        dz_cm,
                        0.0,
                        context=f"search_height={height_idx}",
                    )

                for i in range(self.search_rotation_count):
                    self._raise_search_worker_errors(pending_views)
                    view_idx = height_idx * self.search_rotation_count + i
                    capture_started = time.perf_counter()
                    cur_frame = self._capture_for_task(
                        include_depth=False,
                        context=f"search view={view_idx} RGB capture",
                    )
                    capture_rgb_s = time.perf_counter() - capture_started
                    capture_pose = self._copy_pose(
                        self._get_pose_for_flight(
                            f"search view={view_idx} capture pose"
                        )
                    )
                    submitted_at = time.perf_counter()
                    detection_future = executor.submit(
                        self._detect_search_view,
                        cur_frame.copy(),
                        view_idx,
                        submitted_at,
                    )
                    view_record = {
                        "view_idx": view_idx,
                        "rotation_idx": i,
                        "height_offset_cm": height_offset_cm,
                        "frame": cur_frame,
                        "pose": capture_pose,
                        "capture_s": capture_rgb_s,
                        "submitted_at": submitted_at,
                        "future": detection_future,
                    }
                    pending_views.append(view_record)
                    self._log(
                        f"[SEARCH-PIPELINE] Enqueued view={view_idx} "
                        f"height={height_idx} pending="
                        f"{sum(not item['future'].done() for item in pending_views)} "
                        f"pose={self._format_pose(capture_pose)}"
                    )

                    # The detector consumes this frame while the main thread
                    # commands and waits for the next stable camera view.
                    self.exec_rotate_action_deg(
                        self.search_rotation_step_deg,
                        context=f"search view={view_idx}",
                        log_timing=False,
                    )
                    view_record["motion_timing"] = dict(
                        self._last_motion_timing
                    )
                    view_record["command_id"] = self._last_motion_command_id
        except BaseException as exc:
            for view_record in pending_views:
                view_record["future"].cancel()
            executor.shutdown(
                wait=isinstance(exc, TaskFailure),
                cancel_futures=True,
            )
            raise

        drain_started = time.perf_counter()
        try:
            for view_record in pending_views:
                try:
                    detection_result = view_record["future"].result()
                except Exception as exc:
                    raise TaskFailure(
                        "search detector worker failed for "
                        f"view={view_record['view_idx']}: {exc}"
                    ) from exc
                self._consume_search_view(view_record, detection_result)
        except BaseException:
            for view_record in pending_views:
                view_record["future"].cancel()
            executor.shutdown(wait=True, cancel_futures=True)
            raise
        else:
            executor.shutdown(wait=True)

        self._log(
            f"[SEARCH-PIPELINE] Completed views={len(pending_views)} "
            f"drain_s={time.perf_counter() - drain_started:.4f} "
            f"wall_s={time.perf_counter() - search_started:.4f}"
        )

        if not self.search_candidates:
            self._log(
                "[SEARCH] Completed all height levels; candidate queue is "
                "empty, return to the waypoint."
            )
            return False

        self.ranked_search_candidates = self._rank_view_candidates(
            self.search_candidates,
            context="SEARCH",
        )
        if not self.ranked_search_candidates:
            self._log(
                "[SEARCH] Detector candidates exist, but none passed the "
                "best-view safety filters; return to the waypoint."
            )
            return False
        selected = self.ranked_search_candidates[0]
        self._log(
            f"[SEARCH] Completed all height levels; queued="
            f"{len(self.search_candidates)} selected_candidate="
            f"{selected['candidate_idx']} "
            f"selected_view={selected['view_idx']} "
            f"box_area={selected['box_area_ratio']:.3%} "
            f"confidence={selected['confidence']:.3f} "
            f"center_distance={selected['center_distance']:.3f} "
            f"ranked_views={len(self.ranked_search_candidates)}."
        )
        return True

    @staticmethod
    def _raise_search_worker_errors(pending_views):
        for view_record in pending_views:
            future = view_record["future"]
            if not future.done() or future.cancelled():
                continue
            error = future.exception()
            if error is not None:
                raise TaskFailure(
                    "search detector worker failed for "
                    f"view={view_record['view_idx']}: {error}"
                ) from error

    def _detect_search_view(self, frame_rgb, view_idx, submitted_at):
        worker_started = time.perf_counter()
        detection_started = time.perf_counter()
        all_detections = self.detect(
            frame_rgb,
            vis_name=f"search_view_{view_idx:02d}",
        )
        detector_s = time.perf_counter() - detection_started
        detector_inference_s = float(
            getattr(self.detector, "last_timing", {}).get(
                "inference_s",
                detector_s,
            )
        )
        return {
            "all_detections": all_detections,
            "detector_s": detector_s,
            "detector_inference_s": detector_inference_s,
            "queue_wait_s": worker_started - submitted_at,
        }

    def _consume_search_view(self, view_record, detection_result):
        view_idx = view_record["view_idx"]
        cur_frame = view_record["frame"]
        all_detections = detection_result["all_detections"]
        detections = all_detections[:self.max_candidates_per_frame]
        candidate_boxes = []
        candidate_box_labels = []

        if len(all_detections) > len(detections):
            self._log(
                f"[SEARCH] View={view_idx} detections={len(all_detections)} "
                f"processing={len(detections)}."
            )

        image_height, image_width = cur_frame.shape[:2]
        image_area = float(image_height * image_width)
        for detection_rank, detection in enumerate(detections):
            candidate_idx = (
                view_idx * self.max_candidates_per_frame + detection_rank
            )
            target_box = np.asarray(
                detection["box"],
                dtype=np.float32,
            ).reshape(-1)
            if target_box.size != 4:
                raise ValueError(
                    "Detector box must contain four xyxy values, "
                    f"got shape={target_box.shape}"
                )
            confidence = float(detection["confidence"])
            label = str(detection["label"])
            box_width = max(0.0, float(target_box[2] - target_box[0]))
            box_height = max(0.0, float(target_box[3] - target_box[1]))
            box_area_px = box_width * box_height
            if box_area_px <= 0.0:
                self._log(
                    f"[SEARCH] Skip candidate={candidate_idx}: "
                    "detector box has zero area."
                )
                continue
            box_area_ratio = box_area_px / image_area
            candidate_boxes.append(target_box)
            candidate_box_labels.append(
                f"id: {candidate_idx}, conf: {confidence:.2f}"
            )
            self.search_candidates.append(
                {
                    "candidate_idx": candidate_idx,
                    "view_idx": view_idx,
                    "detection_rank": detection_rank,
                    "height_offset_cm": view_record["height_offset_cm"],
                    "heading_deg": (
                        view_record["rotation_idx"]
                        * self.search_rotation_step_deg
                    ),
                    "pose": view_record["pose"].copy(),
                    "box": target_box.copy(),
                    "box_area_px": box_area_px,
                    "box_area_ratio": box_area_ratio,
                    "confidence": confidence,
                    "label": label,
                    "image_width": image_width,
                    "image_height": image_height,
                }
            )

        visualization_started = time.perf_counter()
        show_fig(
            cur_frame,
            f"{self.vis_dir}/search_view_{view_idx:02d}.png",
            box_coords=(
                np.asarray(candidate_boxes) if candidate_boxes else None
            ),
            box_labels=(candidate_box_labels if candidate_box_labels else None),
        )
        visualization_s = time.perf_counter() - visualization_started
        motion_timing = view_record["motion_timing"]
        detector_s = detection_result["detector_s"]
        total_s = (
            view_record["capture_s"]
            + detector_s
            + visualization_s
            + motion_timing["drone_execution_s"]
        )
        self._log_timing(
            total_s,
            command_id=view_record["command_id"],
            capture=view_record["capture_s"],
            detector={
                "total": detector_s + visualization_s,
                "infer": detection_result["detector_inference_s"],
                "vis": visualization_s,
                "queue_wait": detection_result["queue_wait_s"],
            },
            execution=motion_timing["drone_execution_s"],
        )

    def _search_height_targets(self):
        if self.waypoint_pose is None:
            raise TaskFailure("Waypoint pose is unavailable for search")
        offsets = (
            0.0,
            self.search_height_offset_cm,
            # -self.search_height_offset_cm,
        )
        targets = []
        seen_z = set()
        for height_idx, offset_cm in enumerate(offsets):
            target_z_cm = max(
                self.safe_z_cm,
                float(self.waypoint_pose["z"]) + float(offset_cm),
            )
            rounded_z = round(target_z_cm, 6)
            if rounded_z in seen_z:
                self._log(
                    f"[SEARCH] Skip duplicate height level={height_idx} "
                    f"z={target_z_cm:.2f}cm after safety clamping."
                )
                continue
            seen_z.add(rounded_z)
            targets.append((height_idx, float(offset_cm), target_z_cm))
        return targets

    def select(self):
        """Try ranked safe views until one can initialize SAM tracking."""
        if not self.ranked_search_candidates:
            self._log("[SELECT] Ranked candidate queue is empty.")
            return False

        attempt_candidates = self.ranked_search_candidates[
            : self.select_max_view_attempts
        ]
        total_attempts = len(attempt_candidates)
        for attempt_idx, selected in enumerate(
            attempt_candidates,
            start=1,
        ):
            if self._attempt_select_candidate(
                selected,
                attempt_idx,
                total_attempts,
            ):
                return True
            self._log(
                f"[SELECT] Attempt {attempt_idx}/{total_attempts} "
                f"view={selected['view_idx']} failed re-detection; "
                "try the next ranked view."
            )

        self._log(
            f"[SELECT] All {total_attempts} ranked view attempts failed."
        )
        return False

    def _attempt_select_candidate(
        self,
        selected,
        attempt_idx,
        total_attempts,
    ):
        """Return to one ranked view and initialize tracking if re-detected."""
        self.selected_candidate = selected
        selected_pose = selected["pose"]
        self._log(
            f"[SELECT] Attempt {attempt_idx}/{total_attempts} "
            f"candidate={selected['candidate_idx']} "
            f"view={selected['view_idx']} "
            f"box_area={selected['box_area_ratio']:.3%} "
            f"confidence={selected['confidence']:.3f} "
            f"center_distance={selected['center_distance']:.3f} "
            f"edge_margin={selected['edge_margin_ratio']:.3f} "
            f"pose=(x={selected_pose['x']:.2f}cm, "
            f"y={selected_pose['y']:.2f}cm, z={selected_pose['z']:.2f}cm, "
            f"yaw={selected_pose['yaw']:.2f}deg)"
        )
        self._move_to_pose_hybrid(
            selected_pose,
            context=f"select candidate={selected['candidate_idx']}",
        )

        actual_pose = self._copy_pose(
            self._get_pose_for_flight("select candidate pose")
        )
        pose_error = self._calculate_pose_error(
            selected_pose,
            actual_pose,
        )
        self._log(
            "[SELECT] Candidate pose reached: "
            f"target=(x={selected_pose['x']:.2f}cm, "
            f"y={selected_pose['y']:.2f}cm, z={selected_pose['z']:.2f}cm, "
            f"yaw={selected_pose['yaw']:.2f}deg) "
            f"actual=(x={actual_pose['x']:.2f}cm, "
            f"y={actual_pose['y']:.2f}cm, z={actual_pose['z']:.2f}cm, "
            f"yaw={actual_pose['yaw']:.2f}deg) "
            f"error={self._format_motion_error(pose_error)}"
        )

        # Re-detect at the actual reached pose. The prompt embeddings are
        # already cached, and the stale search box is not reused.
        capture_started = time.perf_counter()
        cur_frame = self._capture_for_task(
            include_depth=False,
            context=f"select attempt={attempt_idx} RGB capture",
        )
        capture_s = time.perf_counter() - capture_started
        detector_started = time.perf_counter()
        all_detections = self.detect(
            cur_frame,
            vis_name=f"select_attempt_{attempt_idx:02d}",
        )
        detector_result_count = len(all_detections)
        detector_s = time.perf_counter() - detector_started
        detector_inference_s = float(
            getattr(self.detector, "last_timing", {}).get(
                "inference_s",
                detector_s,
            )
        )
        detections = all_detections[
            :self.max_candidates_per_frame
        ]
        considered_detection_count = len(detections)
        image_height, image_width = cur_frame.shape[:2]
        image_area = float(image_height * image_width)
        redetected_candidates = []
        for detection_rank, detection in enumerate(detections):
            target_box = np.asarray(
                detection["box"],
                dtype=np.float32,
            ).reshape(-1)
            if target_box.size != 4:
                raise ValueError(
                    "Detector box must contain four xyxy values, "
                    f"got shape={target_box.shape}"
                )
            box_width = max(
                0.0,
                float(target_box[2] - target_box[0]),
            )
            box_height = max(
                0.0,
                float(target_box[3] - target_box[1]),
            )
            box_area_px = box_width * box_height
            if box_area_px <= 0.0:
                raise ValueError(
                    "Detector returned a non-positive-area xyxy box: "
                    f"{target_box.tolist()}"
                )
            redetected_candidates.append(
                {
                    "detection_rank": detection_rank,
                    "box": target_box.copy(),
                    "box_area_px": box_area_px,
                    "box_area_ratio": box_area_px / image_area,
                    "confidence": float(detection["confidence"]),
                    "label": str(detection["label"]),
                    "image_width": image_width,
                    "image_height": image_height,
                    "view_idx": selected["view_idx"],
                }
            )

        redetected_ranked = self._rank_view_candidates(
            redetected_candidates,
            context=(
                f"REDETECT attempt={attempt_idx} "
                f"view={selected['view_idx']}"
            ),
        )
        visualization_started = time.perf_counter()
        if not redetected_ranked:
            if detector_result_count == 0:
                failure_reason = "no_detections"
            else:
                # Deduplication and one-per-view selection always retain at
                # least one candidate. An empty ranking with valid boxes thus
                # means every remaining box failed the edge-margin check.
                failure_reason = "all_valid_boxes_rejected_by_edge_filter"
            attempt_view_path = os.path.join(
                self.vis_dir,
                f"best_view_attempt_{attempt_idx:02d}.png",
            )
            show_fig(cur_frame, attempt_view_path)
            visualization_s = (
                time.perf_counter() - visualization_started
            )
            self._log(
                f"[SELECT-REDETECT] Attempt {attempt_idx}/{total_attempts} "
                f"failed reason={failure_reason} "
                f"detector_results={detector_result_count} "
                f"considered={considered_detection_count} "
                f"valid_boxes={len(redetected_candidates)} "
                f"edge_margin_threshold="
                f"{self.select_min_edge_margin_ratio:.3f} "
                f"image={attempt_view_path}"
            )
            self._log_timing(
                capture_s + detector_s + visualization_s,
                command_id=self._last_motion_command_id,
                capture=capture_s,
                detector={
                    "total": detector_s + visualization_s,
                    "infer": detector_inference_s,
                    "vis": visualization_s,
                },
            )
            return False

        redetected = redetected_ranked[0]
        best_view_path = os.path.join(
            self.vis_dir,
            "best_view_obj.png",
        )
        show_fig(
            cur_frame,
            best_view_path,
            box_coords=redetected["box"],
            box_labels=[
                f"id: {selected['candidate_idx']}, "
                f"conf: {redetected['confidence']:.2f}"
            ],
        )
        visualization_s = time.perf_counter() - visualization_started
        selected = {
            **selected,
            "box": redetected["box"].copy(),
            "box_area_px": redetected["box_area_px"],
            "box_area_ratio": redetected["box_area_ratio"],
            "confidence": redetected["confidence"],
            "label": redetected["label"],
            "redetection_rank": redetected["detection_rank"],
            "image_width": redetected["image_width"],
            "image_height": redetected["image_height"],
            "center_x": redetected["center_x"],
            "center_y": redetected["center_y"],
            "center_distance": redetected["center_distance"],
            "edge_margin_ratio": redetected["edge_margin_ratio"],
        }
        self.selected_candidate = selected
        self._log(
            f"[SELECT] Re-detected candidates="
            f"{len(redetected_candidates)} selected_rank="
            f"{redetected['detection_rank']} "
            f"box_area={redetected['box_area_ratio']:.3%} "
            f"confidence={redetected['confidence']:.3f} "
            f"center_distance={redetected['center_distance']:.3f} "
            f"edge_margin={redetected['edge_margin_ratio']:.3f} "
            f"image={best_view_path}"
        )

        self.tracker.reset()
        track_init_frame_idx = self.tracker.frame_idx
        tracker_started = time.perf_counter()
        bbox, _mask = self.tracker.track_with_mask(
            cur_frame,
            box=redetected["box"],
        )
        track_init_id = self._track_frame_id(track_init_frame_idx)
        tracker_fields = self._tracker_timing_fields(
            self.tracker,
            time.perf_counter() - tracker_started,
        )

        decision_started = time.perf_counter()
        state = self.get_bbox_state(bbox)
        horizontal_offset = state["horizontal_offset"]
        angle_to_rotate = self.prepare_rotate_action_deg(horizontal_offset)
        decision_s = time.perf_counter() - decision_started
        self.exec_rotate_action_deg(
            angle_to_rotate,
            context=f"track_init candidate={selected['candidate_idx']}",
            track_id=track_init_id,
            log_timing=False,
        )
        motion_timing = self._last_motion_timing
        self._log_timing(
            capture_s
            + detector_s
            + visualization_s
            + tracker_fields["total"]
            + decision_s
            + motion_timing["drone_execution_s"],
            command_id=self._last_motion_command_id,
            capture=capture_s,
            detector={
                "total": detector_s + visualization_s,
                "infer": detector_inference_s,
                "vis": visualization_s,
            },
            tracker=tracker_fields,
            decision=decision_s,
            execution=motion_timing["drone_execution_s"],
        )

        return True

    def track(self):
        """Track the initialized target and adjust the drone pose toward it."""
        success = False
        iterations = 0
        while True:
            iterations += 1
            if iterations > self.max_exec_iters:
                print(f"[WARN] Stop target loop after {self.max_exec_iters} iterations to avoid infinite motion.")
                break

            capture_started = time.perf_counter()
            frame_rgb, depth_raw = self._capture_for_task(
                include_depth=True,
                context=f"track iteration={iterations} RGB/depth capture",
            )
            capture_s = time.perf_counter() - capture_started
            track_frame_idx = self.tracker.frame_idx
            tracker_started = time.perf_counter()
            bbox, mask = self.tracker.track_with_mask(frame_rgb)
            track_id = self._track_frame_id(track_frame_idx)
            tracker_fields = self._tracker_timing_fields(
                self.tracker,
                time.perf_counter() - tracker_started,
            )
            save_depth_started = time.perf_counter()
            if self.save_depth:
                self.save_track_depth(depth_raw, track_frame_idx)
            save_depth_s = time.perf_counter() - save_depth_started

            # get box state
            decision_started = time.perf_counter()
            state = self.get_bbox_state(bbox)
            box_ratio = state["box_ratio"]
            horizontal_offset = state["horizontal_offset"]
            vertical_offset = state["vertical_offset"]
            self.last_track_observation = {
                "frame_rgb": frame_rgb,
                "depth_raw": depth_raw,
                "bbox": np.asarray(bbox).copy(),
                "mask": np.asarray(mask).copy(),
                "pose": self._get_pose_for_flight(
                    f"track iteration={iterations} observation pose"
                ),
            }

            angle_to_rotate = self.prepare_rotate_action_deg(horizontal_offset)
            quantized_yaw = self.client.quantize_motion(
                dyaw=angle_to_rotate
            )["yaw"]

            # Large yaw errors are corrected without translation. Once yaw is
            # approximately aligned, all remaining axes share one command.
            if abs(quantized_yaw) > self.yaw_only_threshold_deg:
                decision_s = time.perf_counter() - decision_started
                self.exec_rotate_action_deg(
                    angle_to_rotate,
                    context=f"track iteration={iterations} yaw_only",
                    track_id=track_id,
                    log_timing=False,
                )
                self._log_track_iteration_timing(
                    capture_s,
                    save_depth_s,
                    tracker_fields,
                    decision_s,
                    self._last_motion_timing,
                )
                continue
            dyaw_deg = angle_to_rotate

            dz_cm = self.prepare_z_action_cm(vertical_offset)
            if dz_cm < 0.0:
                cur_z_cm = self.get_current_z_cm()
                if cur_z_cm <= self.safe_z_cm:
                    dz_cm = 0.0
                    print(
                        f"[STALL] Downward motion is blocked at safe_z="
                        f"{self.safe_z_cm:.2f} cm."
                    )
                else:
                    dz_cm = max(dz_cm, self.safe_z_cm - cur_z_cm)

            dx_cm = self.prepare_fb_action_cm(box_ratio)
            if abs(dx_cm) <= self.fb_stop_step_cm:
                print(f"[STALL] dx is {dx_cm} cm and Stalled.")
                dx_cm = 0.0
            elif dx_cm > 0.0:
                requested_dx_cm = dx_cm
                path_depth_cm = self.get_nearest_path_depth_cm(depth_raw, mask)
                available_dx_cm = max(
                    0.0,
                    path_depth_cm - self.depth_safe_distance_cm,
                )
                dx_cm = min(requested_dx_cm, available_dx_cm)
                print(
                    f"[DEPTH] path_p01={path_depth_cm:.2f} cm, "
                    f"safe={self.depth_safe_distance_cm:.2f} cm, "
                    f"requested={requested_dx_cm:.2f} cm, "
                    f"available={available_dx_cm:.2f} cm, "
                    f"forward={dx_cm:.2f} cm."
                )
                if dx_cm <= self.fb_stop_step_cm:
                    print(
                        f"[STALL] Depth-limited forward step {dx_cm:.2f} cm "
                        f"is not executable (minimum "
                        f"{self.fb_stop_step_cm:.2f} cm); stop at the safe "
                        "distance."
                    )
                    dx_cm = 0.0

            dx_cm, _dy_cm, dz_cm, dyaw_deg = (
                self._filter_motion_by_server_tolerance(
                    dx_cm,
                    0.0,
                    dz_cm,
                    dyaw_deg,
                    context=f"track iteration={iterations}",
                    track_id=track_id,
                )
            )

            if any(value != 0.0 for value in (dx_cm, dz_cm, dyaw_deg)):
                decision_s = time.perf_counter() - decision_started
                self.exec_xyz_yaw_hybrid(
                    dx_cm,
                    0.0,
                    dz_cm,
                    dyaw_deg,
                    context=f"track iteration={iterations}",
                    track_id=track_id,
                    log_timing=False,
                )
                self._log_track_iteration_timing(
                    capture_s,
                    save_depth_s,
                    tracker_fields,
                    decision_s,
                    self._last_motion_timing,
                )
                continue

            # All axes are inside their stop thresholds.
            decision_s = time.perf_counter() - decision_started
            self._log_track_iteration_timing(
                capture_s,
                save_depth_s,
                tracker_fields,
                decision_s,
            )
            success = True
            break

        if success:
            print("[RES] Successfully moved to target.")
        else:
            print("[RES] Stopped before confirmed target because motion appears stalled or loop limit was reached.")

        return success

    def _track_frame_id(self, frame_idx):
        """Return the identifier of one saved SAM2 tracking frame."""
        prefix = str(getattr(self.tracker, "track_vis_prefix", "track"))
        index_width = int(
            getattr(self.tracker, "track_vis_index_width", 2)
        )
        return f"{prefix}_{int(frame_idx):0{index_width}d}"

    def scan(self):
        """Follow a small local circle and capture one RGB image at each point."""
        trajectory = self._estimate_scan_circle()
        for target_pose in trajectory:
            clock_hour = target_pose["clock_hour"]
            self._move_to_pose_hybrid(
                target_pose,
                context=f"scan_clock_hour={clock_hour}",
                log_timing=False,
            )
            self._capture_scan_point(clock_hour, self._last_motion_timing)

        # Close the 30-degree arc from 11 o'clock to 12 o'clock without
        # taking a duplicate image.
        self._move_to_pose_hybrid(
            trajectory[0],
            context="scan_close_circle clock_hour=12",
        )

        self._log(
            f"[SCAN] Completed local circular scan: points={len(trajectory)}."
        )
        return True

    def return_waypoint(self):
        """Return to the saved pose with one relative XYZ/yaw command."""
        if self.waypoint_pose is None:
            raise TaskFailure("Waypoint pose is unavailable; call run() first")

        self._ensure_flight_safety("return waypoint")
        target = self.waypoint_pose
        self._move_to_pose_hybrid(
            target,
            context="return_waypoint",
        )

        final_pose = self._get_pose_for_flight("verify waypoint return pose")
        position_error_cm = float(
            np.linalg.norm(
                [
                    float(target["x"]) - float(final_pose["x"]),
                    float(target["y"]) - float(final_pose["y"]),
                    float(target["z"]) - float(final_pose["z"]),
                ]
            )
        )
        yaw_error_deg = self._normalize_angle_deg(
            float(target["yaw"]) - float(final_pose["yaw"])
        )
        self._log(
            "[WAYPOINT] Return completed: "
            f"x={final_pose['x']:.2f}cm, y={final_pose['y']:.2f}cm, "
            f"z={final_pose['z']:.2f}cm, yaw={final_pose['yaw']:.2f}deg, "
            f"position_error={position_error_cm:.2f}cm, "
            f"yaw_error={yaw_error_deg:.2f}deg"
        )
        return final_pose

    def detect(self, image_rgb, vis_name=None):
        """Run the selected backend through the common detector interface."""
        if vis_name is not None:
            set_next_vis_name = getattr(
                self.detector,
                "set_next_vis_name",
                None,
            )
            if callable(set_next_vis_name):
                set_next_vis_name(vis_name)
        detections = self.detector.detect(image_rgb)
        composite_path = getattr(
            self.detector,
            "last_composite_path",
            None,
        )
        if composite_path:
            self._log(f"[SAM3-INPUT] saved={composite_path}")
        return detections

    @staticmethod
    def _candidate_number(candidate):
        return int(
            candidate.get(
                "candidate_idx",
                candidate.get("detection_rank", -1),
            )
        )

    @staticmethod
    def _box_iou(first_box, second_box):
        first = np.asarray(first_box, dtype=np.float32).reshape(4)
        second = np.asarray(second_box, dtype=np.float32).reshape(4)
        ix1 = max(float(first[0]), float(second[0]))
        iy1 = max(float(first[1]), float(second[1]))
        ix2 = min(float(first[2]), float(second[2]))
        iy2 = min(float(first[3]), float(second[3]))
        intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        first_area = max(0.0, float(first[2] - first[0])) * max(
            0.0,
            float(first[3] - first[1]),
        )
        second_area = max(0.0, float(second[2] - second[0])) * max(
            0.0,
            float(second[3] - second[1]),
        )
        union = first_area + second_area - intersection
        return 0.0 if union <= 0.0 else intersection / union

    @staticmethod
    def _enrich_candidate_geometry(candidate):
        enriched = dict(candidate)
        box = np.asarray(enriched["box"], dtype=np.float32).reshape(4)
        image_width = float(enriched["image_width"])
        image_height = float(enriched["image_height"])
        center_x = float((box[0] + box[2]) / 2.0)
        center_y = float((box[1] + box[3]) / 2.0)
        center_distance = float(
            np.hypot(
                (center_x - image_width / 2.0) / (image_width / 2.0),
                (center_y - image_height / 2.0) / (image_height / 2.0),
            )
        )
        edge_margin_ratio = float(
            min(
                float(box[0]) / image_width,
                float(box[1]) / image_height,
                (image_width - float(box[2])) / image_width,
                (image_height - float(box[3])) / image_height,
            )
        )
        enriched.update(
            {
                "center_x": center_x,
                "center_y": center_y,
                "center_distance": center_distance,
                "edge_margin_ratio": edge_margin_ratio,
            }
        )
        return enriched

    def _deduplicate_view_candidates(self, candidates, context):
        grouped = {}
        for candidate in candidates:
            grouped.setdefault(int(candidate["view_idx"]), []).append(
                candidate
            )

        deduplicated = []
        for view_idx in sorted(grouped):
            kept = []
            ordered = sorted(
                grouped[view_idx],
                key=lambda candidate: (
                    -float(candidate["confidence"]),
                    -float(candidate["box_area_ratio"]),
                    self._candidate_number(candidate),
                ),
            )
            for candidate in ordered:
                duplicate = None
                duplicate_iou = 0.0
                for retained in kept:
                    iou = self._box_iou(
                        candidate["box"],
                        retained["box"],
                    )
                    if iou >= self.select_dedup_iou_threshold:
                        duplicate = retained
                        duplicate_iou = iou
                        break
                if duplicate is None:
                    kept.append(candidate)
                    continue
                self._log(
                    f"[SELECT-FILTER][DEDUP] context={context} "
                    f"view={view_idx} drop_candidate="
                    f"{self._candidate_number(candidate)} "
                    f"keep_candidate={self._candidate_number(duplicate)} "
                    f"iou={duplicate_iou:.3f} "
                    f"drop_label={candidate['label']!r} "
                    f"keep_label={duplicate['label']!r}"
                )
            deduplicated.extend(kept)
        return deduplicated

    def _filter_edge_safe_candidates(self, candidates, context):
        safe_candidates = []
        for candidate in candidates:
            margin = float(candidate["edge_margin_ratio"])
            if margin < self.select_min_edge_margin_ratio:
                self._log(
                    f"[SELECT-FILTER][EDGE] context={context} "
                    f"view={candidate['view_idx']} candidate="
                    f"{self._candidate_number(candidate)} "
                    f"margin={margin:.3f} threshold="
                    f"{self.select_min_edge_margin_ratio:.3f}"
                )
                continue
            safe_candidates.append(candidate)
        return safe_candidates

    def _choose_one_candidate_per_view(self, candidates, context):
        selected_by_view = {}
        for candidate in candidates:
            view_idx = int(candidate["view_idx"])
            current = selected_by_view.get(view_idx)
            if current is None or (
                float(candidate["box_area_ratio"]),
                float(candidate["confidence"]),
                -float(candidate["center_distance"]),
            ) > (
                float(current["box_area_ratio"]),
                float(current["confidence"]),
                -float(current["center_distance"]),
            ):
                selected_by_view[view_idx] = candidate

        for candidate in candidates:
            retained = selected_by_view[int(candidate["view_idx"])]
            if candidate is retained:
                continue
            self._log(
                f"[SELECT-FILTER][VIEW] context={context} "
                f"view={candidate['view_idx']} drop_candidate="
                f"{self._candidate_number(candidate)} "
                f"keep_candidate={self._candidate_number(retained)}"
            )
        return list(selected_by_view.values())

    def _log_candidate_ranking(self, stage, context, candidates):
        for rank, candidate in enumerate(candidates, start=1):
            self._log(
                f"[SELECT-RANK][{stage}] context={context} rank={rank} "
                f"view={candidate['view_idx']} candidate="
                f"{self._candidate_number(candidate)} "
                f"label={candidate['label']!r} "
                f"area={candidate['box_area_ratio']:.3%} "
                f"confidence={candidate['confidence']:.3f} "
                f"center=({candidate['center_x']:.1f},"
                f"{candidate['center_y']:.1f}) "
                f"center_distance={candidate['center_distance']:.3f} "
                f"edge_margin={candidate['edge_margin_ratio']:.3f}"
            )

    def _rank_view_candidates(self, candidates, context):
        if not candidates:
            return []
        enriched = [
            self._enrich_candidate_geometry(candidate)
            for candidate in candidates
        ]
        deduplicated = self._deduplicate_view_candidates(
            enriched,
            context,
        )
        edge_safe = self._filter_edge_safe_candidates(
            deduplicated,
            context,
        )
        unique_views = self._choose_one_candidate_per_view(
            edge_safe,
            context,
        )

        area_ranked = sorted(
            unique_views,
            key=lambda candidate: (
                -float(candidate["box_area_ratio"]),
                -float(candidate["confidence"]),
                int(candidate["view_idx"]),
            ),
        )[: self.select_area_top_k]
        self._log_candidate_ranking("AREA", context, area_ranked)

        confidence_ranked = sorted(
            area_ranked,
            key=lambda candidate: (
                -float(candidate["confidence"]),
                -float(candidate["box_area_ratio"]),
                int(candidate["view_idx"]),
            ),
        )[: self.select_confidence_top_k]
        self._log_candidate_ranking(
            "CONFIDENCE",
            context,
            confidence_ranked,
        )

        center_ranked = sorted(
            confidence_ranked,
            key=lambda candidate: (
                float(candidate["center_distance"]),
                -float(candidate["confidence"]),
                -float(candidate["box_area_ratio"]),
                int(candidate["view_idx"]),
            ),
        )
        self._log_candidate_ranking("CENTER", context, center_ranked)
        return center_ranked

    def exec_rotate_action_deg(
        self,
        angle_to_rotate,
        context="",
        log_timing=True,
        track_id=None,
    ):
        _dx, _dy, _dz, dyaw = self._filter_motion_by_server_tolerance(
            0.0,
            0.0,
            0.0,
            float(angle_to_rotate),
            context=context,
            track_id=track_id,
        )
        if dyaw == 0.0:
            return self._motion_skipped_result(
                AgentAction.ROTATE,
                0.0,
                0.0,
                0.0,
                dyaw,
            )
        dx_cm = self.PURE_ROTATE_FORWARD_CM
        return self._execute_motion(
            AgentAction.ROTATE,
            dx_cm,
            0.0,
            0.0,
            dyaw,
            lambda: self.client.move_rel_xyz_yaw(
                x=dx_cm,
                y=0.0,
                z=0.0,
                yaw=dyaw,
            ),
            context=context,
            track_id=track_id,
            log_timing=log_timing,
        )

    def exec_xyz_yaw_hybrid(
        self,
        dx_cm,
        dy_cm,
        dz_cm,
        dyaw_deg,
        context="",
        log_timing=True,
        track_id=None,
    ):
        """Send XYZ translation and yaw through one combined command."""
        dx_cm, dy_cm, dz_cm, dyaw_deg = (
            self._filter_motion_by_server_tolerance(
                dx_cm,
                dy_cm,
                dz_cm,
                dyaw_deg,
                context=context,
                track_id=track_id,
            )
        )
        if not any((dx_cm, dy_cm, dz_cm)) and dyaw_deg != 0.0:
            dx_cm = self.PURE_ROTATE_FORWARD_CM
            self._log(
                f"[PURE-ROTATE] context={context!r} "
                f"using x={dx_cm:.2f}cm with yaw={dyaw_deg:.2f}deg"
            )
        if not any((dx_cm, dy_cm, dz_cm, dyaw_deg)):
            return self._motion_skipped_result(
                AgentAction.XYZ_YAW_HYBRID,
                dx_cm,
                dy_cm,
                dz_cm,
                dyaw_deg,
            )
        return self._execute_motion(
            AgentAction.XYZ_YAW_HYBRID,
            dx_cm,
            dy_cm,
            dz_cm,
            dyaw_deg,
            lambda: self.client.move_rel_xyz_yaw(
                x=dx_cm,
                y=dy_cm,
                z=dz_cm,
                yaw=dyaw_deg,
            ),
            context=context,
            track_id=track_id,
            log_timing=log_timing,
        )

    def _filter_motion_by_server_tolerance(
        self,
        dx_cm,
        dy_cm,
        dz_cm,
        dyaw_deg,
        *,
        context="",
        track_id=None,
    ):
        if self.position_tolerance_cm is None or self.yaw_tolerance_deg is None:
            raise FlightSafetyError(
                "Robot Server motion tolerances are unavailable"
            )

        raw = tuple(float(value) for value in (dx_cm, dy_cm, dz_cm, dyaw_deg))
        command = self.client.quantize_motion(*raw)
        dx_cm = float(command["x"])
        dy_cm = float(command["y"])
        dz_cm = float(command["z"])
        dyaw_deg = float(command["yaw"])
        subject = (
            f" track_id={track_id}"
            if track_id is not None
            else (f" context={context!r}" if context else "")
        )

        raw_translation_requested = any(value != 0.0 for value in raw[:3])
        translation_requested = any(
            value != 0.0 for value in (dx_cm, dy_cm, dz_cm)
        )
        if raw_translation_requested and not translation_requested:
            self._log(
                f"[MOTION-FILTER]{subject} xyz rounded to zero by Client "
                "integer command precision"
            )
        elif translation_requested:
            translation_norm_cm = math.sqrt(
                dx_cm * dx_cm + dy_cm * dy_cm + dz_cm * dz_cm
            )
            if translation_norm_cm <= self.position_tolerance_cm:
                self._log(
                    f"[MOTION-FILTER]{subject} "
                    f"translation_norm={translation_norm_cm:.2f}cm <= "
                    f"position_tolerance={self.position_tolerance_cm:.2f}cm; "
                    "xyz skipped"
                )
                dx_cm = dy_cm = dz_cm = 0.0

        raw_yaw_requested = raw[3] != 0.0
        if raw_yaw_requested and dyaw_deg == 0.0:
            self._log(
                f"[MOTION-FILTER]{subject} yaw rounded to zero by Client "
                "integer command precision"
            )
        elif dyaw_deg != 0.0 and abs(dyaw_deg) <= self.yaw_tolerance_deg:
            self._log(
                f"[MOTION-FILTER]{subject} abs_yaw={abs(dyaw_deg):.2f}deg "
                f"<= yaw_tolerance={self.yaw_tolerance_deg:.2f}deg; "
                "yaw skipped"
            )
            dyaw_deg = 0.0

        return dx_cm, dy_cm, dz_cm, dyaw_deg

    def _motion_skipped_result(
        self,
        action,
        dx_cm,
        dy_cm,
        dz_cm,
        dyaw_deg,
    ):
        self._last_motion_command_id = None
        self._last_motion_timing = {"drone_execution_s": 0.0}
        return {
            "ok": True,
            "skipped": True,
            "message": "motion skipped by Robot Server tolerances",
            "action": action.value,
            "command": {
                "x": dx_cm,
                "y": dy_cm,
                "z": dz_cm,
                "yaw": dyaw_deg,
            },
        }

    def _move_to_pose_hybrid(self, target_pose, context, log_timing=True):
        delta = self._relative_pose_delta(target_pose)
        return self.exec_xyz_yaw_hybrid(
            *delta,
            context=context,
            log_timing=log_timing,
        )

    def _execute_motion(
        self,
        action: AgentAction,
        dx_cm,
        dy_cm,
        dz_cm,
        dyaw_deg,
        operation,
        context="",
        log_timing=True,
        track_id=None,
    ):
        self._command_idx += 1
        command_idx = self._command_idx
        self._last_motion_command_id = command_idx
        pose = self._get_pose_for_flight(f"{context or action.value} start pose")
        command = self._format_motion_command(dx_cm, dy_cm, dz_cm, dyaw_deg)
        last_error = self._format_motion_error(
            getattr(self, "last_motion_error", None)
        )
        track_text = f" track_id={track_id}" if track_id else ""
        context_text = (
            f" context={context}" if context and track_id is None else ""
        )
        self._log(
            f"[COMMAND] id={command_idx}{track_text} action={action.value} "
            f"command={command} "
            f"pose=(x={float(pose['x']):.2f}cm, y={float(pose['y']):.2f}cm, "
            f"z={float(pose['z']):.2f}cm, yaw={float(pose['yaw']):.2f}deg) "
            f"last_error={last_error}{context_text}"
        )
        execution_started = time.perf_counter()
        try:
            result = operation()
        except Exception as exc:
            drone_execution_s = time.perf_counter() - execution_started
            self._last_motion_timing = {
                "drone_execution_s": drone_execution_s,
            }
            self._log(
                f"[MOTION-FAILURE] command_id={command_idx} "
                f"action={action.value} context={context!r} "
                f"execution_s={drone_execution_s:.4f} error={exc}"
            )
            if log_timing:
                self._log_timing(
                    drone_execution_s,
                    command_id=command_idx,
                    execution=drone_execution_s,
                )
            raise FlightSafetyError(
                f"{context or action.value} motion command failed: {exc}"
            ) from exc
        drone_execution_s = time.perf_counter() - execution_started
        self._log(
            f"[ACTION-SLEEP] command_id={command_idx} "
            f"sleep_s={self.ACTION_SLEEP_S:.1f}"
        )
        time.sleep(self.ACTION_SLEEP_S)
        end_pose = self._get_pose_for_flight(
            f"{context or action.value} completion pose"
        )
        self.last_motion_error = self._calculate_motion_error(
            pose,
            end_pose,
            dx_cm,
            dy_cm,
            dz_cm,
            dyaw_deg,
        )
        self._last_motion_timing = {
            "drone_execution_s": drone_execution_s,
        }
        if log_timing:
            self._log_timing(
                drone_execution_s,
                command_id=command_idx,
                execution=drone_execution_s,
            )
        return result

    def prepare_rotate_action_deg(self, horizontal_offset):
        angle_to_rotate = horizontal_offset * self.rotate_deg_per_pixel
        angle_to_rotate = np.clip(
            angle_to_rotate,
            -self.max_rotate_deg,
            self.max_rotate_deg,
        )
        return float(angle_to_rotate)

    def prepare_z_action_cm(self, vertical_offset):
        dz_cm = vertical_offset * self.z_cm_per_pixel
        dz_cm = float(np.clip(dz_cm, -self.max_z_step_cm, self.max_z_step_cm))
        return dz_cm

    def prepare_fb_action_cm(self, box_ratio):
        target = self.target_stop_ratio
        deadband = self.fb_stop_step_cm / self.max_fb_step_cm * target

        if box_ratio > target + deadband:
            step_cm = self.get_backward_step_cm()
        elif box_ratio < target - deadband:
            progress = box_ratio / target
            step_cm = self.max_fb_step_cm * (1.0 - progress)
        else:
            step_cm = 0.0

        return float(step_cm)

    def get_backward_step_cm(self):
        return -self.backward_ratio * self.max_fb_step_cm

    def get_current_z_cm(self) -> Optional[float]:
        pose = self._get_pose_for_flight("current altitude")
        return float(pose["z"])

    def check_box_offset(self, bbox):
        # bbox: [[x1, y1], [x2, y2]]
        box_center = (bbox[0, :] + bbox[1, :]) / 2
        horizontal_offset = box_center[0] - self.horizontal_center
        # Image y grows downward, so positive vertical_offset means the object is above center.
        vertical_offset = self.vertical_center - box_center[1]
        return horizontal_offset, vertical_offset

    def get_bbox_state(self, bbox):
        w = bbox[1, 0] - bbox[0, 0]
        h = bbox[1, 1] - bbox[0, 1]
        box_ratio = max(w / self.img_width, h / self.img_height)
        horizontal_offset, vertical_offset = self.check_box_offset(bbox)
        return {
            "box_ratio": float(box_ratio),
            "horizontal_offset": float(horizontal_offset),
            "vertical_offset": float(vertical_offset),
        }

    def get_nearest_path_depth_cm(self, depth_raw, mask):
        mask = np.asarray(mask, dtype=bool).squeeze()
        if mask.ndim != 2:
            raise RuntimeError(f"Expected a 2D target mask, got shape={mask.shape}")
        if depth_raw.shape != mask.shape:
            depth_raw = cv2.resize(
                depth_raw,
                (mask.shape[1], mask.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

        ys, xs = np.where(mask)
        if xs.size == 0 or ys.size == 0:
            raise RuntimeError("Target mask is empty")

        height, width = mask.shape
        roi_width = max(int(xs.max() - xs.min() + 1), width // 2)
        roi_height = max(int(ys.max() - ys.min() + 1), height // 2)
        x1 = (width - roi_width) // 2
        y1 = (height - roi_height) // 2
        roi_depth = depth_raw[y1:y1 + roi_height, x1:x1 + roi_width]

        valid_path_depths = roi_depth[np.isfinite(roi_depth) & (roi_depth > 0)]
        if valid_path_depths.size == 0:
            raise RuntimeError("No valid positive depth values inside the forward-path ROI")
        return float(np.percentile(valid_path_depths, 1))

    def _relative_pose_delta(self, target_pose, current_pose=None):
        """Convert a world-frame target pose to one body-relative command."""
        pose = (
            self._get_pose_for_flight("relative pose calculation")
            if current_pose is None
            else current_pose
        )
        world_dx_cm = float(target_pose["x"]) - float(pose["x"])
        world_dy_cm = float(target_pose["y"]) - float(pose["y"])
        yaw_rad = np.radians(float(pose["yaw"]))
        body_dx_cm = float(
            np.cos(yaw_rad) * world_dx_cm + np.sin(yaw_rad) * world_dy_cm
        )
        body_dy_cm = float(
            -np.sin(yaw_rad) * world_dx_cm + np.cos(yaw_rad) * world_dy_cm
        )
        dz_cm = float(target_pose["z"]) - float(pose["z"])
        dyaw_deg = self._normalize_angle_deg(
            float(target_pose["yaw"]) - float(pose["yaw"])
        )
        return body_dx_cm, body_dy_cm, dz_cm, dyaw_deg

    def _estimate_scan_circle(self):
        """Plan a 12-point clock-face circle around the track-end pose."""
        observation = self.last_track_observation
        if observation is None:
            raise RuntimeError("Last track observation is unavailable for scan")

        pose = observation["pose"]
        origin_yaw_rad = np.radians(float(pose["yaw"]))
        # Positive dyaw is a right turn in the agent command convention.
        yaw_steps_by_hour = {
            1: -1,
            2: -2,
            3: -3,
            4: -2,
            5: -1,
            6: 0,
            7: 1,
            8: 2,
            9: 3,
            10: 2,
            11: 1,
            12: 0,
        }
        trajectory = []
        clock_hours = [12, *range(1, self.scan_num_points)]
        for clock_hour in clock_hours:
            clock_angle_rad = 2.0 * np.pi * clock_hour / self.scan_num_points
            right_offset_cm = float(
                np.sin(clock_angle_rad) * self.scan_radius_cm
            )
            up_offset_cm = float(
                np.cos(clock_angle_rad) * self.scan_radius_cm
            )
            if clock_hour in (6, 12):
                right_offset_cm = 0.0
            if clock_hour in (3, 9):
                up_offset_cm = 0.0
            point_x = float(pose["x"]) - float(
                np.sin(origin_yaw_rad) * right_offset_cm
            )
            point_y = float(pose["y"]) + float(
                np.cos(origin_yaw_rad) * right_offset_cm
            )
            point_z = max(
                self.safe_z_cm,
                float(pose["z"]) + up_offset_cm,
            )
            point_yaw = self._normalize_angle_deg(
                float(pose["yaw"])
                + yaw_steps_by_hour[clock_hour] * self.scan_yaw_unit_deg
            )
            trajectory.append(
                {
                    "x": point_x,
                    "y": point_y,
                    "z": point_z,
                    "yaw": point_yaw,
                    "clock_hour": clock_hour,
                }
            )

        yaw_offsets = ", ".join(
            f"{point['clock_hour']}="
            f"{self._normalize_angle_deg(point['yaw'] - float(pose['yaw'])):.2f}"
            for point in trajectory
        )
        self._log(
            "[SCAN] Planned simple clock trajectory: "
            f"points={self.scan_num_points} radius={self.scan_radius_cm:.2f}cm "
            f"yaw_unit={self.scan_yaw_unit_deg:.2f}deg "
            f"center=(x={float(pose['x']):.2f}cm, y={float(pose['y']):.2f}cm, "
            f"z={float(pose['z']):.2f}cm, yaw={float(pose['yaw']):.2f}deg) "
            f"yaw_offsets_deg=({yaw_offsets})"
        )
        return trajectory

    def _capture_scan_point(self, clock_hour, motion_timing):
        capture_started = time.perf_counter()
        frame_rgb = self._capture_for_task(
            include_depth=False,
            context=f"scan clock_hour={clock_hour} RGB capture",
        )
        capture_rgb_s = time.perf_counter() - capture_started
        image_path = Path(self.vis_dir) / f"scan_{clock_hour:02d}.png"
        save_image_started = time.perf_counter()
        Image.fromarray(frame_rgb).save(image_path)
        save_image_s = time.perf_counter() - save_image_started
        pose = self._get_pose_for_flight(
            f"scan clock_hour={clock_hour} pose"
        )
        self._log(
            f"[SCAN] Captured clock_hour={clock_hour} image={image_path} "
            f"pose=(x={pose['x']:.2f}cm, y={pose['y']:.2f}cm, "
            f"z={pose['z']:.2f}cm, yaw={pose['yaw']:.2f}deg)"
        )
        total_s = (
            motion_timing["drone_execution_s"]
            + capture_rgb_s
            + save_image_s
        )
        self._log_timing(
            total_s,
            command_id=self._last_motion_command_id,
            capture=capture_rgb_s,
            save_image=save_image_s,
            execution=motion_timing["drone_execution_s"],
        )
        return pose

    @staticmethod
    def _normalize_angle_deg(angle_deg: float) -> float:
        return (float(angle_deg) + 180.0) % 360.0 - 180.0

    @staticmethod
    def _copy_pose(pose):
        return {
            axis: float(pose[axis])
            for axis in ("x", "y", "z", "yaw")
        }

    def save_track_depth(self, depth_raw, frame_idx):
        depth_path = os.path.join(self.vis_dir, f"track_{frame_idx:02d}.npy")
        np.save(depth_path, depth_raw)
        print(f"[DEPTH] Saved raw centimeter depth in {depth_path}")

    @staticmethod
    def _format_motion_command(dx_cm, dy_cm, dz_cm, dyaw_deg) -> str:
        return (
            f"(dx={float(dx_cm):.2f}cm, dy={float(dy_cm):.2f}cm, "
            f"dz={float(dz_cm):.2f}cm, dyaw={float(dyaw_deg):.2f}deg)"
        )

    @staticmethod
    def _format_pose(pose) -> str:
        return (
            f"(x={float(pose['x']):.2f}cm, "
            f"y={float(pose['y']):.2f}cm, "
            f"z={float(pose['z']):.2f}cm, "
            f"yaw={float(pose['yaw']):.2f}deg)"
        )

    @staticmethod
    def _format_motion_error(error) -> str:
        if error is None:
            return "(ex=N/A, ey=N/A, ez=N/A, eyaw=N/A, epos=N/A)"
        return (
            f"(ex={error['ex']:.2f}cm, ey={error['ey']:.2f}cm, "
            f"ez={error['ez']:.2f}cm, eyaw={error['eyaw']:.2f}deg, "
            f"epos={error['epos']:.2f}cm)"
        )

    def _calculate_pose_error(self, target_pose, actual_pose) -> dict:
        ex_cm = float(target_pose["x"]) - float(actual_pose["x"])
        ey_cm = float(target_pose["y"]) - float(actual_pose["y"])
        ez_cm = float(target_pose["z"]) - float(actual_pose["z"])
        eyaw_deg = self._normalize_angle_deg(
            float(target_pose["yaw"]) - float(actual_pose["yaw"])
        )
        return {
            "ex": ex_cm,
            "ey": ey_cm,
            "ez": ez_cm,
            "eyaw": eyaw_deg,
            "epos": float(np.linalg.norm([ex_cm, ey_cm, ez_cm])),
        }

    def _calculate_motion_error(
        self,
        start_pose,
        end_pose,
        dx_cm,
        dy_cm,
        dz_cm,
        dyaw_deg,
    ) -> dict:
        """Return requested-minus-actual error in the command's body frame."""
        world_dx_cm = float(end_pose["x"]) - float(start_pose["x"])
        world_dy_cm = float(end_pose["y"]) - float(start_pose["y"])
        start_yaw_rad = np.radians(float(start_pose["yaw"]))
        actual_dx_cm = float(
            np.cos(start_yaw_rad) * world_dx_cm
            + np.sin(start_yaw_rad) * world_dy_cm
        )
        actual_dy_cm = float(
            -np.sin(start_yaw_rad) * world_dx_cm
            + np.cos(start_yaw_rad) * world_dy_cm
        )
        actual_dz_cm = float(end_pose["z"]) - float(start_pose["z"])
        actual_dyaw_deg = self._normalize_angle_deg(
            float(end_pose["yaw"]) - float(start_pose["yaw"])
        )
        ex_cm = float(dx_cm) - actual_dx_cm
        ey_cm = float(dy_cm) - actual_dy_cm
        ez_cm = float(dz_cm) - actual_dz_cm
        eyaw_deg = self._normalize_angle_deg(
            float(dyaw_deg) - actual_dyaw_deg
        )
        return {
            "ex": ex_cm,
            "ey": ey_cm,
            "ez": ez_cm,
            "eyaw": eyaw_deg,
            "epos": float(np.linalg.norm([ex_cm, ey_cm, ez_cm])),
        }

    @staticmethod
    def _tracker_timing_fields(tracker, measured_total_s: float) -> dict:
        timing = getattr(tracker, "last_timing", {})
        return {
            "total": float(measured_total_s),
            "infer": float(
                timing.get("inference_s", measured_total_s)
            ),
            "vis": float(
                timing.get("visualization_s", 0.0)
            ),
        }

    def _log_track_iteration_timing(
        self,
        capture_s,
        save_depth_s,
        tracker_fields,
        decision_s,
        motion_timing=None,
    ) -> None:
        motion = motion_timing or {
            "drone_execution_s": 0.0,
        }
        total_s = (
            capture_s
            + save_depth_s
            + tracker_fields["total"]
            + decision_s
            + motion["drone_execution_s"]
        )
        self._log_timing(
            total_s,
            command_id=(
                self._last_motion_command_id
                if motion_timing is not None
                else None
            ),
            capture=capture_s,
            save_depth=save_depth_s,
            tracker=tracker_fields,
            decision=decision_s,
            execution=motion["drone_execution_s"],
        )

    def _log_timing(self, total_s: float, command_id=None, **fields) -> None:
        command_id_text = "none" if command_id is None else str(command_id)
        parts = [
            f"[TIMING] command_id={command_id_text}",
            f"total_s={float(total_s):.4f}",
        ]
        for name, value in fields.items():
            time_name = name if name.endswith("_s") else f"{name}_s"
            if isinstance(value, dict):
                details = ", ".join(
                    f"{detail_name if detail_name.endswith('_s') else f'{detail_name}_s'}="
                    f"{float(detail_value):.4f}"
                    for detail_name, detail_value in value.items()
                )
                parts.append(f"{name}:{{{details}}}")
            elif isinstance(value, (int, float)):
                parts.append(f"{time_name}={float(value):.4f}")
            else:
                parts.append(f"{time_name}={value}")
        self._log(" ".join(parts))

    def _log(self, message: str) -> None:
        if self.logger is None:
            print(message)
        else:
            self.logger.info(message)


def build_client(
    client_name: str,
    server_host: str,
    server_port: int,
    http_timeout_s: float,
) -> BaseClient:
    if client_name == "tello":
        from robot_client.tello import TelloClient

        return TelloClient(server_host, server_port, http_timeout_s)
    if client_name == "ue":
        try:
            from robot_client.ue import UEClient
        except ModuleNotFoundError as exc:
            if exc.name == "robot_client.ue":
                raise NotImplementedError("UEClient is not implemented yet") from exc
            raise
        return UEClient(server_host, server_port, http_timeout_s)
    if client_name == "owl":
        from robot_client.owl import OwlClient

        return OwlClient(server_host, server_port, http_timeout_s)
    if client_name == "i7":
        from robot_client.i7 import I7Client

        return I7Client(server_host, server_port, http_timeout_s)
    raise ValueError(f"unsupported client: {client_name}")


_DETECTOR_BACKENDS = {
    "yolo": {
        "module": "third_party.yolo_world.detector",
        "detector_class": "YOLOWorldX640Detector",
        "config_class": "YOLOWorldX640Config",
        "config_keys": {
            "input_size",
            "score_threshold",
            "nms_iou_threshold",
            "fp16",
        },
    },
    "grounding_dino": {
        "module": "third_party.grounding_dino.detector",
        "detector_class": "GroundingDINODetector",
        "config_class": "GroundingDINOConfig",
        "config_keys": {
            "box_threshold",
            "text_threshold",
        },
    },
    "sam3": {
        "module": "third_party.sam3.detector",
        "detector_class": "Sam3Detector",
        "config_class": "Sam3DetectorConfig",
        "config_keys": {
            "confidence_threshold",
        },
    },
}


def build_detector(detector_name: str, config: dict):
    try:
        backend = _DETECTOR_BACKENDS[detector_name]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported detector backend: {detector_name!r}"
        ) from exc

    detectors_config = _config_section(config, "detectors")
    detector_config = _config_section(detectors_config, detector_name)
    expected_keys = backend["config_keys"]
    missing_keys = sorted(expected_keys - detector_config.keys())
    unknown_keys = sorted(detector_config.keys() - expected_keys)
    if missing_keys or unknown_keys:
        details = []
        if missing_keys:
            details.append(f"missing={missing_keys}")
        if unknown_keys:
            details.append(f"unknown={unknown_keys}")
        raise ValueError(
            f"Invalid config for detector backend {detector_name!r}: "
            + " ".join(details)
        )

    module_name = backend["module"]
    detector_class_name = backend["detector_class"]
    config_class_name = backend["config_class"]
    module = importlib.import_module(module_name)
    try:
        detector_class = getattr(module, detector_class_name)
        config_class = getattr(module, config_class_name)
    except AttributeError as exc:
        raise ImportError(
            f"Invalid detector backend {detector_name!r}: "
            f"{module_name}.{exc.name} does not exist"
        ) from exc

    try:
        backend_config = config_class(**detector_config)
    except TypeError as exc:
        raise ValueError(
            f"Invalid config for detector backend {detector_name!r}"
        ) from exc
    return detector_class(backend_config)


_TRACKER_BACKENDS = {
    "sam2": {
        "module": "third_party.sam2.stream",
        "tracker_class": "Sam2VideoPredictor",
        "config_class": "SAM2Config",
        "config_keys": set(),
    },
}


def build_tracker(
    tracker_name: str,
    config: dict,
    vis_dir: str,
    save_vis: bool = True,
    image_height: int = 360,
    image_width: int = 640,
):
    try:
        backend = _TRACKER_BACKENDS[tracker_name]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported tracker backend: {tracker_name!r}"
        ) from exc

    trackers_config = _config_section(config, "trackers")
    tracker_config = _config_section(trackers_config, tracker_name)
    expected_keys = backend["config_keys"]
    missing_keys = sorted(expected_keys - tracker_config.keys())
    unknown_keys = sorted(tracker_config.keys() - expected_keys)
    if missing_keys or unknown_keys:
        details = []
        if missing_keys:
            details.append(f"missing={missing_keys}")
        if unknown_keys:
            details.append(f"unknown={unknown_keys}")
        raise ValueError(
            f"Invalid config for tracker backend {tracker_name!r}: "
            + " ".join(details)
        )

    module_name = backend["module"]
    tracker_class_name = backend["tracker_class"]
    config_class_name = backend["config_class"]
    module = importlib.import_module(module_name)
    try:
        tracker_class = getattr(module, tracker_class_name)
        config_class = getattr(module, config_class_name)
    except AttributeError as exc:
        raise ImportError(
            f"Invalid tracker backend {tracker_name!r}: "
            f"{module_name}.{exc.name} does not exist"
        ) from exc

    try:
        backend_config = config_class(**tracker_config)
    except TypeError as exc:
        raise ValueError(
            f"Invalid config for tracker backend {tracker_name!r}"
        ) from exc
    backend_config.VIDEO_HEIGHT = int(image_height)
    backend_config.VIDEO_WIDTH = int(image_width)
    tracker = tracker_class(backend_config)

    required_methods = (
        "reset",
        "track_with_mask",
        "set_img_size",
        "set_vis_mode",
        "set_vis_dir",
        "set_track_vis_naming",
    )
    missing_methods = [
        method_name
        for method_name in required_methods
        if not callable(getattr(tracker, method_name, None))
    ]
    if missing_methods:
        raise TypeError(
            f"Tracker backend {tracker_name!r} does not implement "
            f"required methods: {missing_methods}"
        )

    tracker.set_vis_mode(save_vis)
    tracker.set_vis_dir(vis_dir)
    tracker.set_track_vis_naming("track", index_width=2)
    return tracker


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help=(
            "YAML path relative to src/agent/config, such as i7/v19.yaml; "
            "absolute paths are also supported. Defaults to i7/v19.yaml "
            "for the I7 v19 agent"
        ),
    )
    parser.add_argument(
        "--exp_name",
        type=str,
        default=str(Path(__file__).resolve().parents[3] / "logs" / "test"),
        help=(
            "Experiment directory prefix; a timestamp suffix is added "
            "automatically (default: repository logs/test)"
        ),
    )
    prompt_group = parser.add_mutually_exclusive_group(required=True)
    prompt_group.add_argument(
        "--text",
        type=str,
        help="Text prompt used by YOLO, GroundingDINO, or SAM3",
    )
    prompt_group.add_argument(
        "--img",
        type=str,
        help="Reference image used as a SAM3 visual exemplar",
    )
    parser.add_argument(
        "--box",
        type=str,
        metavar="BOX_TXT",
        help=(
            "Path to a text file containing exactly four space-separated "
            "xyxy pixel coordinates; required with --img"
        ),
    )
    parser.add_argument(
        "--det",
        choices=("yolo", "grounding_dino", "sam3"),
        default="yolo",
        help="Detector backend loaded at runtime (default: yolo)",
    )
    parser.add_argument(
        "--trk",
        choices=("sam2",),
        default="sam2",
        help="Tracker backend loaded at runtime (default: sam2)",
    )
    parser.add_argument(
        "--client",
        "--robot",
        dest="client",
        choices=("tello", "ue", "owl", "i7"),
        default="i7",
        help=(
            "Robot to use: tello, ue, owl, or i7; --robot is an alias for "
            "--client (default: i7)"
        ),
    )
    parser.add_argument(
        "--server-host",
        default="127.0.0.1",
        help="Robot Server IP or hostname (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--server-port",
        type=int,
        default=8765,
        help="Robot Server HTTP port (default: 8765)",
    )
    parser.add_argument(
        "--http-timeout-s",
        type=float,
        default=180.0,
        help="Robot Server request timeout in seconds (default: 180)",
    )
    return parser


def _resolve_config_path(config_path: str | None, client_name: str) -> Path:
    config_root = Path(__file__).resolve().parents[1] / "config"
    if config_path is None:
        if client_name == "i7":
            return (config_root / "i7" / "v19.yaml").resolve()
        return (config_root / "owl" / "v17.yaml").resolve()

    requested_path = Path(config_path).expanduser()
    if requested_path.is_absolute():
        return requested_path.resolve()
    return (config_root / requested_path).resolve()


def _land_after_task(client: BaseClient, logger: logging.Logger) -> dict:
    result = client.land()
    if not result.get("ok", False):
        raise RuntimeError(
            "Robot land failed: "
            + str(result.get("error") or result.get("message") or result)
        )
    logger.info(f"[LAND] {result.get('message', 'landed')}")
    return result


def _build_detector_prompt(args) -> str | dict:
    if args.text is not None:
        text = args.text.strip()
        if not text:
            raise ValueError("--text must not be empty")
        if args.box is not None:
            raise ValueError("--box can only be used together with --img")
        return text

    if args.det != "sam3":
        raise ValueError(
            f"detector {args.det!r} only supports --text; --img requires --det sam3"
        )
    if args.box is None:
        raise ValueError("--box BOX_TXT is required with --img")
    image_path = Path(args.img).expanduser().resolve()
    if not image_path.is_file():
        raise ValueError(f"reference image not found: {image_path}")
    box_path = Path(args.box).expanduser().resolve()
    if not box_path.is_file():
        raise ValueError(f"reference box file not found: {box_path}")
    try:
        box_values = box_path.read_text(encoding="utf-8").split()
    except OSError as exc:
        raise ValueError(f"failed to read reference box file: {box_path}") from exc
    if len(box_values) != 4:
        raise ValueError(
            f"reference box file must contain exactly four space-separated "
            f"xyxy values, got {len(box_values)}: {box_path}"
        )
    try:
        x1, y1, x2, y2 = (float(value) for value in box_values)
    except ValueError as exc:
        raise ValueError(
            f"reference box file contains a non-numeric value: {box_path}"
        ) from exc
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
        raise ValueError(f"reference box values must be finite: {box_path}")
    if x1 < 0.0 or y1 < 0.0 or x2 <= x1 or y2 <= y1:
        raise ValueError(
            f"reference box must satisfy 0 <= X1 < X2 and "
            f"0 <= Y1 < Y2: {box_path}"
        )
    try:
        with Image.open(image_path) as reference_image:
            image_width, image_height = reference_image.size
            reference_image.verify()
    except Exception as exc:
        raise ValueError(
            f"failed to read reference image: {image_path}: {exc}"
        ) from exc
    if x2 > image_width or y2 > image_height:
        raise ValueError(
            f"reference box {(x1, y1, x2, y2)} from {box_path} exceeds "
            f"reference image size {(image_width, image_height)}"
        )
    return {
        "type": "visual",
        "image_path": str(image_path),
        "box_path": str(box_path),
        "box_xyxy": (x1, y1, x2, y2),
    }


def main():
    parser = _build_arg_parser()
    args = parser.parse_args()
    try:
        detector_prompt = _build_detector_prompt(args)
    except ValueError as exc:
        parser.error(str(exc))
    args.config = str(_resolve_config_path(args.config, args.client))
    # Validate the required config before creating logs, clients, or models.
    config = _load_config(args.config)
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    exp_dir = _timestamped_experiment_dir(args.exp_name, timestamp)
    vis_dir = exp_dir / "vis"
    log_path = exp_dir / "log.txt"
    vis_dir.mkdir(parents=True, exist_ok=True)
    logger = _configure_logger(log_path)
    logger.info(
        f"[RUN] client={args.client} detector={args.det} "
        f"tracker={args.trk} "
        f"prompt={detector_prompt!r} "
        f"depth_source={'ue_native' if args.client == 'ue' else 'client_da3'} "
        f"config={Path(args.config).expanduser().resolve()} "
        f"exp_dir={exp_dir} vis={vis_dir} log={log_path}"
    )

    client = build_client(
        args.client,
        server_host=args.server_host,
        server_port=args.server_port,
        http_timeout_s=args.http_timeout_s,
    )
    detector = build_detector(args.det, config)
    tracker = build_tracker(
        args.trk,
        config,
        vis_dir=str(vis_dir),
    )
    tjkAgent = TJKAgent(
        client=client,
        detector=detector,
        tracker=tracker,
        config=config,
        vis_dir=str(vis_dir),
        logger=logger,
        detector_name=args.det,
        tracker_name=args.trk,
    )
    try:
        tjkAgent.connect()
        mission_result = tjkAgent.run_mission(detector_prompt)
        logger.info(
            f"[MISSION-RESULT] success={mission_result['success']} "
            f"returned_home={mission_result['returned_home']} "
            f"waypoints={mission_result['waypoints']}; request landing."
        )
    except BaseException as exc:
        logger.exception(f"[FATAL] Agent mission failed: {exc}")
        tjkAgent._set_mission_state(MissionState.EMERGENCY_LANDING)
        try:
            _land_after_task(client, logger)
        except Exception:
            logger.exception("[LAND] failed while handling Agent exception")
        finally:
            tjkAgent._set_mission_state(MissionState.ABORTED)
        raise
    else:
        tjkAgent._set_mission_state(MissionState.LANDING)
        try:
            _land_after_task(client, logger)
        except Exception:
            tjkAgent._set_mission_state(MissionState.ABORTED)
            raise
        tjkAgent._set_mission_state(MissionState.COMPLETED)

if __name__ == "__main__":
    main()
