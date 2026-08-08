"""Robot Server controller for the VisBot OWL mini3L."""

from __future__ import annotations

import datetime as dt
import math
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, Optional

import cv2
import numpy as np

from ..hardware.owl import OwlHardware


class OwlController:
    """Translate the common Robot Server API into Captain and ROS topics.

    Public coordinates match the existing Tello/UE convention: centimetres,
    x forward, y right, z up, and clockwise-positive yaw. MAVROS odometry is
    treated as ROS ENU, so exposed world y and yaw are sign-inverted.
    """

    TAKEOFF_TASK_ID = 20
    LANDING_TASK_ID = 21
    NAVIGATION_TASK_ID = 105
    DRONE_ID = 0
    OUTPUT_LONG_EDGE = 640
    JPEG_QUALITY = 85
    POSITION_TOLERANCE_CM = 15.0
    LINEAR_SPEED_TOLERANCE_CM_S = 10.0
    POSITION_STABLE_SAMPLES = 5
    POSITION_POLL_HZ = 10.0
    YAW_TOLERANCE_DEG = 5.0
    YAW_PUBLISH_HZ = 20.0
    YAW_RATE_DEG_S = 30.0
    ASSUMED_SPEED_M_S = 0.5
    NAVIGATION_START_DELAY_S = 2.0
    GOAL_ACK_TIMEOUT_S = 2.0
    TAKEOFF_TIMEOUT_S = 30.0
    LANDING_TIMEOUT_S = 60.0
    FLIGHT_STATE_POLL_HZ = 10.0

    def __init__(
        self,
        image_dir: Optional[str] = None,
        hardware: Optional[OwlHardware] = None,
        *,
        position_tolerance_cm: float = POSITION_TOLERANCE_CM,
        linear_speed_tolerance_cm_s: float = LINEAR_SPEED_TOLERANCE_CM_S,
        position_stable_samples: int = POSITION_STABLE_SAMPLES,
        position_poll_hz: float = POSITION_POLL_HZ,
        yaw_tolerance_deg: float = YAW_TOLERANCE_DEG,
        yaw_publish_hz: float = YAW_PUBLISH_HZ,
        yaw_rate_deg_s: float = YAW_RATE_DEG_S,
        navigation_start_delay_s: float = NAVIGATION_START_DELAY_S,
        goal_ack_timeout_s: float = GOAL_ACK_TIMEOUT_S,
        takeoff_timeout_s: float = TAKEOFF_TIMEOUT_S,
        landing_timeout_s: float = LANDING_TIMEOUT_S,
        flight_state_poll_hz: float = FLIGHT_STATE_POLL_HZ,
    ) -> None:
        self._hardware = hardware or OwlHardware()
        self._operation_lock = threading.RLock()
        self._image_lock = threading.RLock()
        self._yaw_lock = threading.RLock()
        self._navigation_state = "stopped"
        self._yaw_active = False
        self._yaw_error: Optional[str] = None
        self._yaw_thread: Optional[threading.Thread] = None
        self._yaw_start_rad = 0.0
        self._yaw_delta_rad = 0.0
        self._yaw_target_rad = 0.0
        self._yaw_started_at = 0.0
        self._yaw_frame_id = "world"
        self._image_dir = Path(image_dir or "captures").expanduser().resolve()
        self._image_dir.mkdir(parents=True, exist_ok=True)
        self._position_tolerance_cm = max(1.0, float(position_tolerance_cm))
        self._linear_speed_tolerance_cm_s = max(
            0.0,
            float(linear_speed_tolerance_cm_s),
        )
        self._position_stable_samples = max(1, int(position_stable_samples))
        self._position_poll_hz = max(1.0, float(position_poll_hz))
        self._yaw_tolerance_deg = max(0.1, float(yaw_tolerance_deg))
        self._yaw_publish_hz = max(20.0, float(yaw_publish_hz))
        self._yaw_rate_rad_s = math.radians(
            max(1.0, float(yaw_rate_deg_s))
        )
        self._navigation_start_delay_s = max(
            0.0,
            float(navigation_start_delay_s),
        )
        self._goal_ack_timeout_s = max(0.1, float(goal_ack_timeout_s))
        self._takeoff_timeout_s = max(0.1, float(takeoff_timeout_s))
        self._landing_timeout_s = max(0.1, float(landing_timeout_s))
        self._flight_state_poll_hz = max(1.0, float(flight_state_poll_hz))

    def init(self) -> Dict[str, Any]:
        with self._operation_lock:
            result = self._hardware.connect()
            return {
                "ok": bool(result.get("ok", True)),
                "message": result.get("message", "initialized"),
                "health": self.health(),
            }

    def takeoff(self) -> Dict[str, Any]:
        with self._operation_lock:
            health = self.health()
            if health.get("initialized") is not True:
                raise RuntimeError("OWL is not initialized, call init first")
            if health.get("airborne") is True:
                if health.get("offboard") is True:
                    return {
                        "ok": True,
                        "message": "already airborne and OFFBOARD; no takeoff command sent",
                        "health": health,
                    }
                return {
                    "ok": False,
                    "error": (
                        "OWL is already airborne but not in OFFBOARD mode; "
                        "switch to OFFBOARD manually before starting the Agent"
                    ),
                    "health": health,
                }
            if health.get("landed_state") != OwlHardware.LANDED_STATE_ON_GROUND:
                raise RuntimeError(
                    "OWL takeoff state is unknown: "
                    f"landed_state={health.get('landed_state')!r}"
                )

            try:
                self._hardware.publish_control(
                    self.TAKEOFF_TASK_ID,
                    self.DRONE_ID,
                )
                final_health = self._wait_for_health(
                    lambda value: value.get("control_ready") is True,
                    self._takeoff_timeout_s,
                    "takeoff",
                )
                return {
                    "ok": True,
                    "message": "takeoff done",
                    "pose": self.get_pose()["pose"],
                    "health": final_health,
                }
            except Exception as exc:
                self._land_after_command_error("takeoff", exc)

    def get_rgb_meta(self, save: bool = True) -> Dict[str, Any]:
        frame_bgr = self._get_resized_bgr()
        result: Dict[str, Any] = {
            "ok": True,
            "height": int(frame_bgr.shape[0]),
            "width": int(frame_bgr.shape[1]),
        }
        if self._as_bool(save):
            path = self._image_dir / self._timestamped_name("image", ".jpg")
            if not cv2.imwrite(str(path), frame_bgr):
                raise RuntimeError(f"failed to save RGB frame: {path}")
            result["saved_to"] = str(path)
        return result

    def get_rgb_byte(self) -> bytes:
        frame_bgr = self._get_resized_bgr()
        success, encoded = cv2.imencode(
            ".jpg",
            frame_bgr,
            [cv2.IMWRITE_JPEG_QUALITY, self.JPEG_QUALITY],
        )
        if not success:
            raise RuntimeError("failed to encode OWL RGB frame as JPEG")
        return encoded.tobytes()

    def get_depth_meta(self, save: bool = True) -> Dict[str, Any]:
        return {
            "ok": False,
            "error": "OWL depth is estimated by DA3 in OwlClient",
        }

    def get_depth_np(self) -> Dict[str, Any]:
        return {
            "ok": False,
            "error": "OWL depth is estimated by DA3 in OwlClient",
        }

    def velocity(self, x: int, y: int, z: int, yaw: int) -> Dict[str, Any]:
        return {
            "ok": False,
            "error": "velocity control is not enabled for OWL v1",
        }

    def move_relative_xyz(self, x: int, y: int, z: int) -> Dict[str, Any]:
        result = self.move_relative_xyz_yaw(x=x, y=y, z=z, yaw=0)
        result["message"] = result["message"].replace(
            "move_relative_xyz_yaw",
            "move_relative_xyz",
        )
        result["command"].pop("yaw", None)
        return result

    def move_relative_xyz_yaw(
        self,
        x: int,
        y: int,
        z: int,
        yaw: int,
    ) -> Dict[str, Any]:
        with self._operation_lock:
            try:
                return self._move_relative_xyz_yaw(
                    int(x),
                    int(y),
                    int(z),
                    int(yaw),
                )
            except Exception as exc:
                self._land_after_command_error(
                    "move_relative_xyz_yaw",
                    exc,
                )

    def rotate(self, angle_deg: int) -> Dict[str, Any]:
        angle = int(angle_deg)
        result = self.move_relative_xyz_yaw(0, 0, 0, angle)
        result["message"] = result["message"].replace(
            "move_relative_xyz_yaw",
            "rotate",
        )
        result["command"] = {"angle_deg": angle}
        result["direction"] = (
            "none"
            if angle == 0
            else "clockwise" if angle > 0 else "counter_clockwise"
        )
        return result

    def _move_relative_xyz_yaw(
        self,
        x: int,
        y: int,
        z: int,
        yaw: int,
    ) -> Dict[str, Any]:
        self._require_control_ready()
        command = {"x": x, "y": y, "z": z, "yaw": yaw}
        start = self._hardware.get_pose_ros()
        if not any(command.values()):
            return {
                "ok": True,
                "message": "move_relative_xyz_yaw skipped",
                "command": command,
                "pose": self._public_pose(start),
            }

        start_yaw_rad = float(start["yaw_rad"])
        target_yaw_rad = self._normalize_angle_rad(
            start_yaw_rad - math.radians(yaw)
        )
        forward_m = x / 100.0
        right_m = y / 100.0
        up_m = z / 100.0
        target_x_m = (
            float(start["x_m"])
            + math.cos(start_yaw_rad) * forward_m
            + math.sin(start_yaw_rad) * right_m
        )
        target_y_m = (
            float(start["y_m"])
            + math.sin(start_yaw_rad) * forward_m
            - math.cos(start_yaw_rad) * right_m
        )
        target_z_m = float(start["z_m"]) + up_m
        has_translation = any((x, y, z))

        goal_ack_source = None
        if has_translation:
            self._ensure_navigation_started()
            self._activate_yaw_control(
                start_yaw_rad,
                target_yaw_rad,
                str(start["frame_id"]),
            )
            goal_sent_at = time.monotonic()
            self._hardware.publish_goal(
                target_x_m,
                target_y_m,
                target_z_m,
                (
                    0.0,
                    0.0,
                    math.sin(target_yaw_rad / 2.0),
                    math.cos(target_yaw_rad / 2.0),
                ),
                str(start["frame_id"]),
            )
            try:
                goal_ack = self._hardware.wait_for_goal_ack(
                    target_x_m,
                    target_y_m,
                    target_z_m,
                    after_monotonic=goal_sent_at,
                    timeout_s=self._goal_ack_timeout_s,
                )
            except Exception:
                self._navigation_state = "unknown"
                raise
            self._navigation_state = "ready"
            goal_ack_source = goal_ack.get("source")
        else:
            self._activate_yaw_control(
                start_yaw_rad,
                target_yaw_rad,
                str(start["frame_id"]),
            )

        distance_m = math.sqrt(
            forward_m * forward_m
            + right_m * right_m
            + up_m * up_m
        )
        translation_timeout_s = (
            distance_m / self.ASSUMED_SPEED_M_S * 3.0 + 5.0
        )
        rotation_timeout_s = (
            abs(math.radians(yaw)) / self._yaw_rate_rad_s + 5.0
        )
        timeout_s = min(
            120.0,
            max(8.0, translation_timeout_s, rotation_timeout_s),
        )
        final_pose = self._wait_for_pose(
            target_x_m,
            target_y_m,
            target_z_m,
            target_yaw_rad,
            require_position=has_translation,
            timeout_s=timeout_s,
        )
        return {
            "ok": True,
            "message": "move_relative_xyz_yaw done",
            "command": command,
            "pose": self._public_pose(final_pose),
            "target_ros": {
                "x_cm": target_x_m * 100.0,
                "y_cm": target_y_m * 100.0,
                "z_cm": target_z_m * 100.0,
                "yaw_deg": math.degrees(target_yaw_rad),
                "frame_id": str(start["frame_id"]),
            },
            "goal_ack_source": goal_ack_source,
        }

    def get_pose(self) -> Dict[str, Any]:
        pose = self._hardware.get_pose_ros()
        return {
            "ok": True,
            "pose": self._public_pose(pose),
            "units": {"position": "cm", "yaw": "degree"},
            "estimated": False,
            "frame": "agent_world_x_forward_y_right_z_up",
            "sources": {
                "position": OwlHardware.ODOM_TOPIC,
                "orientation": OwlHardware.ODOM_TOPIC,
            },
        }

    def land(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._deactivate_yaw_control()
            health = self.health()
            ros_initialized = health.get(
                "ros_initialized",
                health.get("initialized", False),
            )
            if ros_initialized is not True:
                self._navigation_state = "stopped"
                return {
                    "ok": True,
                    "message": "OWL is not initialized; no landing command sent",
                    "health": health,
                }

            already_landed = bool(
                health.get("state_ok") is True
                and health.get("extended_state_ok") is True
                and health.get("armed") is False
                and health.get("landed_state")
                == OwlHardware.LANDED_STATE_ON_GROUND
            )
            if already_landed:
                self._navigation_state = "stopped"
                return {
                    "ok": True,
                    "message": "already landed; ROS communication remains active",
                    "health": health,
                }

            self._navigation_state = "landing"
            self._hardware.publish_control(self.LANDING_TASK_ID, self.DRONE_ID)
            try:
                final_health = self._wait_for_health(
                    lambda value: (
                        value.get("state_ok") is True
                        and value.get("extended_state_ok") is True
                        and value.get("armed") is False
                        and value.get("landed_state")
                        == OwlHardware.LANDED_STATE_ON_GROUND
                    ),
                    self._landing_timeout_s,
                    "landing",
                )
            except Exception:
                self._navigation_state = "unknown"
                raise
            self._navigation_state = "stopped"
            return {
                "ok": True,
                "message": "landed; ROS communication remains active",
                "health": final_health,
            }

    def close(self) -> Dict[str, Any]:
        with self._operation_lock:
            landing_error: Optional[Exception] = None
            try:
                landing = self.land()
                if not landing.get("ok", False):
                    landing_error = RuntimeError(
                        str(landing.get("error") or landing.get("message"))
                    )
            except Exception as exc:
                landing_error = exc

            self._deactivate_yaw_control()
            close_error: Optional[Exception] = None
            try:
                self._hardware.close()
            except Exception as exc:
                close_error = exc
            self._navigation_state = "stopped"

            first_error = landing_error or close_error
            if first_error is not None:
                return {
                    "ok": False,
                    "message": "closed with landing or cleanup error",
                    "error": str(first_error),
                    "health": self.health(),
                }
            return {
                "ok": True,
                "message": "closed; ROS communication stopped",
                "health": self.health(),
            }

    def health(self) -> Dict[str, Any]:
        result = dict(self._hardware.health())
        with self._yaw_lock:
            result["yaw_control_active"] = self._yaw_active
            result["yaw_control_error"] = self._yaw_error
            result["yaw_target"] = (
                self._normalize_yaw_deg(-math.degrees(self._yaw_target_rad))
                if self._yaw_active
                else None
            )
        result["navigation_state"] = self._navigation_state
        result["navigation_started"] = bool(
            self._navigation_state == "ready"
            and result.get("control_ready") is True
        )
        return result

    def _ensure_navigation_started(self) -> None:
        self._require_control_ready()
        if self._navigation_state == "ready":
            return
        self._hardware.publish_control(
            self.NAVIGATION_TASK_ID,
            self.DRONE_ID,
        )
        self._navigation_state = "starting"
        if self._navigation_start_delay_s > 0:
            time.sleep(self._navigation_start_delay_s)
        try:
            self._require_control_ready()
        except Exception:
            self._navigation_state = "unknown"
            raise

    def _wait_for_pose(
        self,
        target_x_m: float,
        target_y_m: float,
        target_z_m: float,
        target_yaw_rad: float,
        *,
        require_position: bool,
        timeout_s: float,
    ) -> Dict[str, Any]:
        deadline = time.monotonic() + float(timeout_s)
        period_s = 1.0 / self._position_poll_hz
        stable_samples = 0
        last_pose = self._hardware.get_pose_ros()
        position_error_cm = 0.0
        yaw_error_deg = 0.0
        linear_speed_cm_s = 0.0
        while time.monotonic() < deadline:
            self._require_control_ready()
            self._raise_if_yaw_control_failed()
            last_pose = self._hardware.get_pose_ros()
            position_error_cm = 100.0 * math.sqrt(
                (target_x_m - float(last_pose["x_m"])) ** 2
                + (target_y_m - float(last_pose["y_m"])) ** 2
                + (target_z_m - float(last_pose["z_m"])) ** 2
            )
            yaw_error_deg = abs(
                math.degrees(
                    self._normalize_angle_rad(
                        target_yaw_rad - float(last_pose["yaw_rad"])
                    )
                )
            )
            linear_speed_cm_s = 100.0 * math.sqrt(
                float(last_pose["vx_m_s"]) ** 2
                + float(last_pose["vy_m_s"]) ** 2
                + float(last_pose["vz_m_s"]) ** 2
            )
            position_ok = (
                not require_position
                or position_error_cm <= self._position_tolerance_cm
            )
            speed_ok = (
                linear_speed_cm_s <= self._linear_speed_tolerance_cm_s
            )
            if (
                position_ok
                and speed_ok
                and yaw_error_deg <= self._yaw_tolerance_deg
            ):
                stable_samples += 1
                if stable_samples >= self._position_stable_samples:
                    return last_pose
            else:
                stable_samples = 0
            time.sleep(period_s)
        raise RuntimeError(
            "pose move timeout: "
            f"target=({target_x_m * 100.0:.1f}, "
            f"{target_y_m * 100.0:.1f}, {target_z_m * 100.0:.1f})cm "
            f"target_yaw={math.degrees(target_yaw_rad):.1f}deg "
            f"pose=({float(last_pose['x_m']) * 100.0:.1f}, "
            f"{float(last_pose['y_m']) * 100.0:.1f}, "
            f"{float(last_pose['z_m']) * 100.0:.1f})cm "
            f"position_error={position_error_cm:.1f}cm "
            f"yaw_error={yaw_error_deg:.1f}deg "
            f"linear_speed={linear_speed_cm_s:.1f}cm/s"
        )

    def _activate_yaw_control(
        self,
        start_yaw_rad: float,
        target_yaw_rad: float,
        frame_id: str,
    ) -> None:
        with self._yaw_lock:
            self._yaw_start_rad = self._normalize_angle_rad(start_yaw_rad)
            self._yaw_delta_rad = self._normalize_angle_rad(
                target_yaw_rad - start_yaw_rad
            )
            self._yaw_target_rad = self._normalize_angle_rad(target_yaw_rad)
            self._yaw_started_at = time.monotonic()
            self._yaw_frame_id = str(frame_id or "world")
            self._yaw_error = None
            self._yaw_active = True

        self._publish_yaw_sample()
        with self._yaw_lock:
            if self._yaw_thread is None or not self._yaw_thread.is_alive():
                self._yaw_thread = threading.Thread(
                    target=self._yaw_publish_loop,
                    name="owl-yaw-control",
                    daemon=True,
                )
                self._yaw_thread.start()

    def _deactivate_yaw_control(self) -> None:
        with self._yaw_lock:
            self._yaw_active = False

    def _yaw_publish_loop(self) -> None:
        period_s = 1.0 / self._yaw_publish_hz
        while True:
            with self._yaw_lock:
                if not self._yaw_active:
                    return
            try:
                self._publish_yaw_sample()
            except Exception as exc:
                with self._yaw_lock:
                    self._yaw_error = str(exc)
                    self._yaw_active = False
                try:
                    self.land()
                except Exception:
                    pass
                return
            time.sleep(period_s)

    def _publish_yaw_sample(self) -> None:
        with self._yaw_lock:
            if not self._yaw_active:
                return
            elapsed_s = max(0.0, time.monotonic() - self._yaw_started_at)
            max_step_rad = self._yaw_rate_rad_s * elapsed_s
            delta_abs_rad = abs(self._yaw_delta_rad)
            if max_step_rad >= delta_abs_rad:
                yaw_rad = self._yaw_target_rad
                yaw_rate_rad_s = 0.0
            else:
                direction = 1.0 if self._yaw_delta_rad >= 0.0 else -1.0
                yaw_rad = self._normalize_angle_rad(
                    self._yaw_start_rad + direction * max_step_rad
                )
                yaw_rate_rad_s = direction * self._yaw_rate_rad_s
            frame_id = self._yaw_frame_id
            self._hardware.publish_yaw_target(
                yaw_rad,
                yaw_rate_rad_s,
                frame_id,
            )

    def _raise_if_yaw_control_failed(self) -> None:
        with self._yaw_lock:
            error = self._yaw_error
        if error:
            raise RuntimeError(f"yaw control publisher failed: {error}")

    def _land_after_command_error(
        self,
        operation: str,
        command_error: Exception,
    ) -> None:
        self._navigation_state = "unknown"
        try:
            landing = self.land()
        except Exception as landing_error:
            raise RuntimeError(
                f"OWL {operation} failed: {command_error}; "
                f"emergency landing also failed: {landing_error}"
            ) from command_error
        raise RuntimeError(
            f"OWL {operation} failed: {command_error}; "
            f"landing requested: {landing.get('message', 'land command sent')}"
        ) from command_error

    def _require_control_ready(self) -> None:
        health = self.health()
        if health.get("control_ready") is True:
            return
        missing = health.get("missing") or []
        detail = ", ".join(str(value) for value in missing)
        if health.get("initialized") is not True:
            raise RuntimeError(
                "OWL is not initialized or telemetry is stale"
                + (f": {detail}" if detail else "")
            )
        if health.get("airborne") is not True:
            raise RuntimeError("OWL is not airborne; take off first")
        if health.get("offboard") is not True:
            raise RuntimeError(
                f"OWL must be in OFFBOARD mode, current mode={health.get('mode')!r}"
            )
        raise RuntimeError("OWL control is not ready")

    def _wait_for_health(
        self,
        predicate: Callable[[Dict[str, Any]], bool],
        timeout_s: float,
        operation: str,
    ) -> Dict[str, Any]:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        period_s = 1.0 / self._flight_state_poll_hz
        last_health = self.health()
        while time.monotonic() < deadline:
            last_health = self.health()
            if predicate(last_health):
                return last_health
            time.sleep(period_s)
        raise RuntimeError(
            f"OWL {operation} timeout: "
            f"mode={last_health.get('mode')!r}, "
            f"armed={last_health.get('armed')!r}, "
            f"landed_state={last_health.get('landed_state')!r}"
        )

    def _get_resized_bgr(self) -> np.ndarray:
        with self._image_lock:
            encoded, image_format = self._hardware.get_compressed_rgb()
            frame_bgr = cv2.imdecode(
                np.frombuffer(encoded, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if frame_bgr is None:
                raise RuntimeError(
                    f"failed to decode compressed OWL image format={image_format!r}"
                )
            height, width = frame_bgr.shape[:2]
            long_edge = max(height, width)
            if long_edge != self.OUTPUT_LONG_EDGE:
                scale = self.OUTPUT_LONG_EDGE / float(long_edge)
                output_width = max(1, int(round(width * scale)))
                output_height = max(1, int(round(height * scale)))
                frame_bgr = cv2.resize(
                    frame_bgr,
                    (output_width, output_height),
                    interpolation=(
                        cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
                    ),
                )
            return frame_bgr

    @staticmethod
    def _public_pose(pose_ros: Dict[str, Any]) -> Dict[str, float]:
        return {
            "x": float(pose_ros["x_m"]) * 100.0,
            "y": -float(pose_ros["y_m"]) * 100.0,
            "z": float(pose_ros["z_m"]) * 100.0,
            "yaw": OwlController._normalize_yaw_deg(
                -math.degrees(float(pose_ros["yaw_rad"]))
            ),
        }

    @staticmethod
    def _normalize_yaw_deg(value: float) -> float:
        return (float(value) + 180.0) % 360.0 - 180.0

    @staticmethod
    def _normalize_angle_rad(value: float) -> float:
        return math.atan2(math.sin(float(value)), math.cos(float(value)))

    @staticmethod
    def _as_bool(value: Any) -> bool:
        if isinstance(value, str):
            return value.strip().lower() not in {"false", "0", "no", "off"}
        return bool(value)

    @staticmethod
    def _timestamped_name(prefix: str, suffix: str) -> str:
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return f"{prefix}_{timestamp}{suffix}"


__all__ = ["OwlController"]
