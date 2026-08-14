"""Robot Server controller for the I7 UAV."""

from __future__ import annotations

import datetime as dt
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np
import yaml

from ..hardware.i7 import I7Hardware


def _load_i7_nav_config(config_path: Optional[str | Path] = None) -> dict:
    path = (
        Path(config_path).expanduser().resolve()
        if config_path is not None
        else (
            Path(__file__).resolve().parents[3]
            / "ros"
            / "i7_nav"
            / "config"
            / "i7_nav.yaml"
        )
    )
    try:
        with path.open("r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"Failed to load I7 navigation config: {path}") from exc
    if not isinstance(config, dict):
        raise ValueError(f"I7 navigation config must be a mapping: {path}")
    return config


def _required_number(config: dict, key: str, *, integer: bool = False):
    value = config.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"I7 config value {key} must be numeric, got {value!r}")
    result = int(value) if integer else float(value)
    if integer and float(value) != result:
        raise ValueError(f"I7 config value {key} must be an integer, got {value!r}")
    if not math.isfinite(float(result)) or result <= 0:
        raise ValueError(f"I7 config value {key} must be positive, got {result!r}")
    return result


def _required_string(config: dict, key: str) -> str:
    value = config.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"I7 config value {key} must be a non-empty string")
    return value.strip()


def _required_section(config: dict, key: str) -> dict:
    value = config.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"Missing I7 config section: {key}")
    return value


class I7Controller:
    """Translate the common Robot API to I7 ROS goals and nav services.

    Public coordinates follow the existing Robot API: centimetres, x forward,
    y right, z up, and clockwise-positive yaw. ROS odometry is ENU, therefore
    public world y and yaw have the opposite sign.
    """

    def __init__(
        self,
        image_dir: Optional[str] = None,
        hardware: Optional[I7Hardware] = None,
        config_path: Optional[str | Path] = None,
    ) -> None:
        config = _load_i7_nav_config(config_path)
        completion_config = _required_section(config, "completion")
        timeouts_config = _required_section(config, "timeouts")
        control_config = _required_section(config, "control")
        controller_config = _required_section(config, "robot_controller")
        topics_config = _required_section(config, "topics")
        services_config = _required_section(config, "services")
        self._jpeg_quality = _required_number(
            controller_config, "jpeg_quality", integer=True
        )
        self._position_tolerance_cm = 100.0 * _required_number(
            completion_config, "goal_reached_distance_m"
        )
        self._yaw_tolerance_deg = _required_number(
            completion_config, "goal_yaw_tolerance_deg"
        )
        self._linear_speed_tolerance_cm_s = 100.0 * _required_number(
            completion_config, "stable_speed_m_s"
        )
        self._stable_samples = _required_number(
            completion_config, "stable_samples", integer=True
        )
        self._poll_hz = _required_number(
            controller_config, "completion_poll_hz"
        )
        self._goal_ack_timeout_s = _required_number(
            controller_config, "goal_ack_timeout_s"
        )
        self._goal_ack_position_tolerance_m = _required_number(
            controller_config, "goal_ack_position_tolerance_m"
        )
        self._assumed_speed_m_s = _required_number(
            controller_config, "assumed_speed_m_s"
        )
        self._translation_timeout_scale = _required_number(
            controller_config, "translation_timeout_scale"
        )
        self._rotation_timeout_deg_s = _required_number(
            controller_config, "rotation_timeout_deg_s"
        )
        self._motion_timeout_padding_s = _required_number(
            controller_config, "motion_timeout_padding_s"
        )
        self._default_motion_timeout_s = _required_number(
            controller_config, "default_motion_timeout_s"
        )
        self._max_motion_timeout_s = _required_number(
            controller_config, "max_motion_timeout_s"
        )
        if self._max_motion_timeout_s < self._default_motion_timeout_s:
            raise ValueError(
                "I7 config max_motion_timeout_s must be >= "
                "default_motion_timeout_s"
            )
        self._hardware = hardware or I7Hardware(
            node_name=_required_string(controller_config, "node_name"),
            rtsp_url=_required_string(controller_config, "rtsp_url"),
            ready_timeout_s=_required_number(
                controller_config, "ready_timeout_s"
            ),
            ros_services_ready_timeout_s=_required_number(
                controller_config, "ros_services_ready_timeout_s"
            ),
            state_max_age_s=_required_number(timeouts_config, "state_max_age_s"),
            odom_max_age_s=_required_number(timeouts_config, "odom_max_age_s"),
            rgb_max_age_s=_required_number(
                controller_config, "rgb_max_age_s"
            ),
            nav_max_age_s=_required_number(
                controller_config, "nav_max_age_s"
            ),
            planner_max_age_s=_required_number(
                timeouts_config, "planner_heartbeat_max_age_s"
            ),
            battery_max_age_s=_required_number(
                controller_config, "battery_max_age_s"
            ),
            condition_wait_timeout_s=_required_number(
                timeouts_config, "condition_wait_timeout_s"
            ),
            camera_open_retry_s=_required_number(
                controller_config, "camera_open_retry_s"
            ),
            camera_read_retry_s=_required_number(
                controller_config, "camera_read_retry_s"
            ),
            camera_join_timeout_s=_required_number(
                controller_config, "camera_join_timeout_s"
            ),
            rtsp_io_timeout_us=_required_number(
                controller_config, "rtsp_io_timeout_us", integer=True
            ),
            camera_buffer_size=_required_number(
                controller_config, "camera_buffer_size", integer=True
            ),
            control_queue_size=_required_number(
                control_config, "control_queue_size", integer=True
            ),
            status_queue_size=_required_number(
                control_config, "status_queue_size", integer=True
            ),
            odom_topic=_required_string(topics_config, "odom_topic"),
            state_topic=_required_string(topics_config, "state_topic"),
            extended_state_topic=_required_string(
                topics_config, "extended_state_topic"
            ),
            battery_topic=_required_string(topics_config, "battery_topic"),
            nav_state_topic=_required_string(topics_config, "nav_state_topic"),
            nav_active_goal_topic=_required_string(
                topics_config, "nav_active_goal_topic"
            ),
            planner_heartbeat_topic=_required_string(
                topics_config, "planner_heartbeat_topic"
            ),
            goal_topic=_required_string(topics_config, "goal_topic"),
            takeoff_service=_required_string(services_config, "takeoff_service"),
            landing_service=_required_string(services_config, "landing_service"),
            force_land_service=_required_string(
                services_config, "force_land_service"
            ),
            abort_service=_required_string(services_config, "abort_service"),
            reinitialize_service=_required_string(
                services_config, "reinitialize_service"
            ),
        )
        self._operation_lock = threading.RLock()
        self._image_lock = threading.RLock()
        self._image_dir = Path(image_dir or "captures").expanduser().resolve()
        self._image_dir.mkdir(parents=True, exist_ok=True)

    def init(self) -> Dict[str, Any]:
        """Initialize communication; remote init never clears a takeover latch."""
        with self._operation_lock:
            result = self._hardware.connect()
            health = self.health()
            if health.get("manual_takeover_latched") is True:
                raise RuntimeError(
                    "manual takeover is latched; use local console init before takeoff"
                )
            return {
                "ok": bool(result.get("ok", True)),
                "message": result.get("message", "I7 initialized"),
                "health": health,
            }

    def console_init(self) -> Dict[str, Any]:
        """Local-only recovery entry point used by the Robot Server console."""
        with self._operation_lock:
            self._hardware.connect()
            before = self.health()
            recovery = None
            if before.get("manual_takeover_latched") is True:
                recovery = self._hardware.call_reinitialize()
            return {
                "ok": True,
                "message": (
                    "I7 local navigation latch cleared; call takeoff, then select OFFBOARD"
                    if recovery is not None
                    else "I7 initialized"
                ),
                "recovery": recovery,
                "health": self.health(),
            }

    def takeoff(self) -> Dict[str, Any]:
        """Wait for manual OFFBOARD, then let i7_nav arm and take off."""
        with self._operation_lock:
            health = self.health()
            self._require_initialized(health)
            if health.get("manual_takeover_latched") is True:
                raise RuntimeError(
                    "manual takeover is latched; run local console init first"
                )
            result = self._hardware.call_takeoff()
            return {
                "ok": True,
                "message": result["message"],
                "pose": self.get_pose()["pose"],
                "health": self.health(),
            }

    def get_rgb_meta(self, save: bool = True) -> Dict[str, Any]:
        frame = self._get_native_bgr()
        result: Dict[str, Any] = {
            "ok": True,
            "height": int(frame.shape[0]),
            "width": int(frame.shape[1]),
            "source": self._hardware.rtsp_url,
        }
        if self._as_bool(save):
            path = self._image_dir / self._timestamped_name("image", ".jpg")
            if not cv2.imwrite(str(path), frame):
                raise RuntimeError(f"failed to save I7 RGB frame: {path}")
            result["saved_to"] = str(path)
        return result

    def get_rgb_byte(self) -> bytes:
        frame = self._get_native_bgr()
        success, encoded = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
        )
        if not success:
            raise RuntimeError("failed to encode I7 RGB frame as JPEG")
        return encoded.tobytes()

    def get_depth_meta(self, save: bool = True) -> Dict[str, Any]:
        del save
        return {"ok": False, "error": "I7 depth is estimated by DA3 in Agent"}

    def get_depth_np(self) -> Dict[str, Any]:
        return {"ok": False, "error": "I7 depth is estimated by DA3 in Agent"}

    def velocity(self, x: int, y: int, z: int, yaw: int) -> Dict[str, Any]:
        del x, y, z, yaw
        return {"ok": False, "error": "velocity control is not enabled for I7 v1"}

    def move_relative_xyz(self, x: int, y: int, z: int) -> Dict[str, Any]:
        result = self.move_relative_xyz_yaw(x=x, y=y, z=z, yaw=0)
        result["message"] = result["message"].replace(
            "move_relative_xyz_yaw", "move_relative_xyz"
        )
        result["command"].pop("yaw", None)
        return result

    def move_relative_xyz_yaw(
        self,
        x: int,
        y: int,
        z: int,
        yaw: int,
        timeout_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        with self._operation_lock:
            command = {"x": int(x), "y": int(y), "z": int(z), "yaw": int(yaw)}
            try:
                return self._move_relative_xyz_yaw(
                    **command,
                    timeout_s=timeout_s,
                )
            except Exception as exc:
                self._abort_after_command_error("move_relative_xyz_yaw", exc)

    def _move_relative_xyz_yaw(
        self,
        x: int,
        y: int,
        z: int,
        yaw: int,
        timeout_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        self._require_control_ready()
        start = self._hardware.get_pose_ros()
        command = {"x": x, "y": y, "z": z, "yaw": yaw}
        if not any(command.values()):
            return {
                "ok": True,
                "message": "move_relative_xyz_yaw skipped",
                "command": command,
                "pose": self._public_pose(start),
            }

        start_yaw = float(start["yaw_rad"])
        forward_m = x / 100.0
        right_m = y / 100.0
        up_m = z / 100.0
        target_x = (
            float(start["x_m"])
            + math.cos(start_yaw) * forward_m
            + math.sin(start_yaw) * right_m
        )
        target_y = (
            float(start["y_m"])
            + math.sin(start_yaw) * forward_m
            - math.cos(start_yaw) * right_m
        )
        target_z = float(start["z_m"]) + up_m
        target_yaw = self._normalize_angle(start_yaw - math.radians(yaw))
        self._validate_target_height(target_z)
        uses_planner = any(value != 0 for value in (x, y, z))

        sent_at = time.monotonic()
        self._hardware.publish_goal(
            target_x,
            target_y,
            target_z,
            math.degrees(target_yaw),
            str(start["frame_id"]),
            use_planner=uses_planner,
        )
        ack = self._hardware.wait_for_goal_ack(
            target_x,
            target_y,
            target_z,
            uses_planner=uses_planner,
            after_monotonic=sent_at,
            timeout_s=self._goal_ack_timeout_s,
            tolerance_m=self._goal_ack_position_tolerance_m,
        )

        distance_m = math.sqrt(forward_m**2 + right_m**2 + up_m**2)
        translation_timeout = (
            distance_m
            / self._assumed_speed_m_s
            * self._translation_timeout_scale
            + self._motion_timeout_padding_s
        )
        rotation_timeout = (
            abs(float(yaw)) / self._rotation_timeout_deg_s
            + self._motion_timeout_padding_s
        )
        configured_timeout_s = (
            self._default_motion_timeout_s
            if timeout_s is None
            else float(timeout_s)
        )
        if not math.isfinite(configured_timeout_s) or configured_timeout_s <= 0.0:
            raise ValueError(
                "motion timeout must be finite and positive, got "
                f"{configured_timeout_s!r}"
            )
        effective_timeout_s = min(
            self._max_motion_timeout_s,
            max(configured_timeout_s, translation_timeout, rotation_timeout),
        )
        final_pose = self._wait_for_pose(
            target_x,
            target_y,
            target_z,
            target_yaw,
            timeout_s=effective_timeout_s,
        )
        return {
            "ok": True,
            "message": "move_relative_xyz_yaw done",
            "command": command,
            "pose": self._public_pose(final_pose),
            "target_ros": {
                "x_cm": target_x * 100.0,
                "y_cm": target_y * 100.0,
                "z_cm": target_z * 100.0,
                "yaw_deg": math.degrees(target_yaw),
                "frame_id": str(start["frame_id"]),
            },
            "control_mode": "ego_planner" if uses_planner else "direct_yaw",
            "configured_motion_timeout_s": configured_timeout_s,
            "effective_motion_timeout_s": effective_timeout_s,
            "goal_ack_source": self._hardware.nav_active_goal_topic,
            "goal_ack": ack,
        }

    def rotate(self, angle_deg: int) -> Dict[str, Any]:
        angle = int(angle_deg)
        result = self.move_relative_xyz_yaw(0, 0, 0, angle)
        result["message"] = result["message"].replace(
            "move_relative_xyz_yaw", "rotate"
        )
        result["command"] = {"angle_deg": angle}
        return result

    def get_pose(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "pose": self._public_pose(self._hardware.get_pose_ros()),
            "units": {"position": "cm", "yaw": "degree"},
            "estimated": False,
            "frame": "agent_world_x_forward_y_right_z_up",
            "sources": {
                "position": self._hardware.odom_topic,
                "orientation": self._hardware.odom_topic,
            },
        }

    def get_motion_tolerances(self) -> Dict[str, Any]:
        return {
            "position_tolerance_cm": self._position_tolerance_cm,
            "yaw_tolerance_deg": self._yaw_tolerance_deg,
            "linear_speed_tolerance_cm_s": self._linear_speed_tolerance_cm_s,
            "stable_samples": self._stable_samples,
            "position_error_metric": "euclidean_3d",
            "source": "robot_pose_completion",
        }

    def land(self) -> Dict[str, Any]:
        with self._operation_lock:
            health = self.health()
            if health.get("ros_initialized") is not True:
                return {"ok": True, "message": "I7 is not initialized", "health": health}
            if health.get("manual_takeover_latched") is True:
                raise RuntimeError(
                    "manual takeover is latched; automatic /land is inhibited"
                )
            result = self._hardware.call_land()
            return {"ok": True, "message": result["message"], "health": self.health()}

    def abort(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._require_initialized(self.health())
            result = self._hardware.call_abort()
            return {"ok": True, "message": result["message"], "health": self.health()}

    def force_land(self) -> Dict[str, Any]:
        """Local-console emergency action; never exposed as an HTTP endpoint."""
        with self._operation_lock:
            self._require_initialized(self.health())
            result = self._hardware.call_force_land()
            return {"ok": True, "message": result["message"], "health": self.health()}

    def close(self) -> Dict[str, Any]:
        with self._operation_lock:
            health = self.health()
            landing_result = None
            landing_error = None
            should_land = bool(
                health.get("initialized") is True
                and health.get("airborne") is True
                and health.get("control_session_active") is True
                and health.get("manual_takeover_latched") is not True
            )
            if should_land:
                try:
                    landing_result = self.land()
                except Exception as exc:
                    landing_error = str(exc)
            self._hardware.close()
            if landing_error:
                return {
                    "ok": False,
                    "message": "I7 communication closed after landing error",
                    "error": landing_error,
                }
            return {
                "ok": True,
                "message": "I7 communication closed",
                "landing": landing_result,
            }

    def health(self) -> Dict[str, Any]:
        health = dict(self._hardware.health())
        health["velocity_control_supported"] = False
        health["image_output"] = {
            "width": health.get("camera_width"),
            "height": health.get("camera_height"),
            "encoding": "jpeg",
            "resolution_mode": "native",
        }
        return health

    def _wait_for_pose(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        target_yaw: float,
        *,
        timeout_s: float,
    ) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        stable_samples = 0
        last_pose = self._hardware.get_pose_ros()
        position_error_cm = yaw_error_deg = speed_cm_s = float("inf")
        while time.monotonic() < deadline:
            self._require_control_ready()
            last_pose = self._hardware.get_pose_ros()
            position_error_cm = 100.0 * math.sqrt(
                (target_x - float(last_pose["x_m"])) ** 2
                + (target_y - float(last_pose["y_m"])) ** 2
                + (target_z - float(last_pose["z_m"])) ** 2
            )
            yaw_error_deg = abs(
                math.degrees(
                    self._normalize_angle(target_yaw - float(last_pose["yaw_rad"]))
                )
            )
            speed_cm_s = 100.0 * math.sqrt(
                float(last_pose["vx_m_s"]) ** 2
                + float(last_pose["vy_m_s"]) ** 2
                + float(last_pose["vz_m_s"]) ** 2
            )
            if (
                position_error_cm <= self._position_tolerance_cm
                and yaw_error_deg <= self._yaw_tolerance_deg
                and speed_cm_s <= self._linear_speed_tolerance_cm_s
            ):
                stable_samples += 1
                if stable_samples >= self._stable_samples:
                    return last_pose
            else:
                stable_samples = 0
            time.sleep(1.0 / self._poll_hz)
        raise RuntimeError(
            "I7 pose move timeout: "
            f"position_error={position_error_cm:.1f}cm, "
            f"yaw_error={yaw_error_deg:.1f}deg, speed={speed_cm_s:.1f}cm/s"
        )

    def _validate_target_height(self, target_z: float) -> None:
        health = self.health()
        ground_z = health.get("ground_z_m")
        min_height = health.get("min_height_m")
        if ground_z is None or min_height is None:
            raise RuntimeError("I7 ground-height safety boundary is unavailable")
        min_z = float(ground_z) + float(min_height)
        if target_z < min_z:
            raise ValueError(
                f"I7 target z={target_z:.2f}m is below minimum {min_z:.2f}m"
            )

    def _abort_after_command_error(self, operation: str, error: Exception) -> None:
        abort_detail = ""
        try:
            abort_result = self._hardware.call_abort()
            abort_detail = f"; navigation abort requested: {abort_result['message']}"
        except Exception as abort_error:
            abort_detail = f"; navigation abort also failed: {abort_error}"
        raise RuntimeError(f"I7 {operation} failed: {error}{abort_detail}") from error

    def _require_initialized(self, health: Dict[str, Any]) -> None:
        if health.get("initialized") is True:
            return
        detail = ", ".join(str(value) for value in health.get("missing") or [])
        raise RuntimeError(
            "I7 is not initialized or telemetry is stale"
            + (f": {detail}" if detail else "")
        )

    def _require_control_ready(self) -> None:
        health = self.health()
        self._require_initialized(health)
        if health.get("manual_takeover_latched") is True:
            raise RuntimeError("manual takeover is latched; local init is required")
        if health.get("control_ready") is not True:
            raise RuntimeError(
                "I7 control is not ready: "
                f"state={health.get('nav_state')!r}, mode={health.get('mode')!r}, "
                f"armed={health.get('armed')!r}"
            )

    def _get_native_bgr(self) -> np.ndarray:
        """Return the RTSP frame without changing its pixel resolution."""
        with self._image_lock:
            return self._hardware.get_bgr()

    @staticmethod
    def _public_pose(pose: Dict[str, Any]) -> Dict[str, float]:
        return {
            "x": float(pose["x_m"]) * 100.0,
            "y": -float(pose["y_m"]) * 100.0,
            "z": float(pose["z_m"]) * 100.0,
            "yaw": I7Controller._normalize_yaw_deg(
                -math.degrees(float(pose["yaw_rad"]))
            ),
        }

    @staticmethod
    def _normalize_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    @staticmethod
    def _normalize_yaw_deg(value: float) -> float:
        return ((float(value) + 180.0) % 360.0) - 180.0

    @staticmethod
    def _as_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() not in {"", "0", "false", "no", "off"}
        return bool(value)

    @staticmethod
    def _timestamped_name(prefix: str, suffix: str) -> str:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return f"{prefix}_{stamp}{suffix}"


__all__ = ["I7Controller"]
