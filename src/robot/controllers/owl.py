"""Robot Server controller for the VisBot OWL mini3L."""

from __future__ import annotations

import datetime as dt
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np

from ..hardware.owl import OwlHardware


class OwlController:
    """Translate the common Robot Server API into Captain and ROS topics.

    Public coordinates match the existing Tello/UE convention: centimetres,
    x forward, y right, z up, and clockwise-positive yaw. MAVROS odometry is
    treated as ROS ENU, so exposed world y and yaw are sign-inverted.
    """

    NAVIGATION_TASK_ID = 105
    DRONE_ID = 0
    OUTPUT_WIDTH = 640
    OUTPUT_HEIGHT = 480
    JPEG_QUALITY = 85
    POSITION_TOLERANCE_CM = 40.0
    POSITION_STABLE_SAMPLES = 3
    POSITION_POLL_HZ = 10.0
    ASSUMED_SPEED_M_S = 0.5
    NAVIGATION_START_DELAY_S = 2.0

    def __init__(
        self,
        image_dir: Optional[str] = None,
        hardware: Optional[OwlHardware] = None,
        *,
        position_tolerance_cm: float = POSITION_TOLERANCE_CM,
        position_stable_samples: int = POSITION_STABLE_SAMPLES,
        position_poll_hz: float = POSITION_POLL_HZ,
        navigation_start_delay_s: float = NAVIGATION_START_DELAY_S,
    ) -> None:
        self._hardware = hardware or OwlHardware()
        self._operation_lock = threading.RLock()
        self._image_lock = threading.RLock()
        self._navigation_started = False
        self._image_dir = Path(image_dir or "captures").expanduser().resolve()
        self._image_dir.mkdir(parents=True, exist_ok=True)
        self._position_tolerance_cm = max(1.0, float(position_tolerance_cm))
        self._position_stable_samples = max(1, int(position_stable_samples))
        self._position_poll_hz = max(1.0, float(position_poll_hz))
        self._navigation_start_delay_s = max(
            0.0,
            float(navigation_start_delay_s),
        )

    def init(self) -> Dict[str, Any]:
        with self._operation_lock:
            result = self._hardware.connect()
            return {
                "ok": bool(result.get("ok", True)),
                "message": result.get("message", "initialized"),
                "health": self.health(),
            }

    def takeoff(self) -> Dict[str, Any]:
        health = self.health()
        if health.get("airborne") is True:
            return {
                "ok": True,
                "message": "already airborne; no takeoff command sent",
                "health": health,
            }
        return {
            "ok": False,
            "error": (
                "automatic takeoff is disabled for OWL; run the Captain "
                "init/takeoff procedure manually before starting the Agent"
            ),
            "health": health,
        }

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
        with self._operation_lock:
            self._require_control_ready()
            command = {"x": int(x), "y": int(y), "z": int(z)}
            start = self._hardware.get_pose_ros()
            if not any(command.values()):
                return {
                    "ok": True,
                    "message": "move_relative_xyz skipped",
                    "command": command,
                    "pose": self._public_pose(start),
                }

            self._ensure_navigation_started()
            yaw_rad = float(start["yaw_rad"])
            forward_m = command["x"] / 100.0
            right_m = command["y"] / 100.0
            up_m = command["z"] / 100.0

            # ROS ENU/FLU: forward=(cos(yaw), sin(yaw)); the Agent's right
            # axis is the negative ROS body-left axis.
            target_x_m = (
                float(start["x_m"])
                + math.cos(yaw_rad) * forward_m
                + math.sin(yaw_rad) * right_m
            )
            target_y_m = (
                float(start["y_m"])
                + math.sin(yaw_rad) * forward_m
                - math.cos(yaw_rad) * right_m
            )
            target_z_m = float(start["z_m"]) + up_m

            self._hardware.publish_goal(
                target_x_m,
                target_y_m,
                target_z_m,
                (
                    0.0,
                    0.0,
                    math.sin(yaw_rad / 2.0),
                    math.cos(yaw_rad / 2.0),
                ),
                str(start["frame_id"]),
            )
            distance_m = math.sqrt(
                forward_m * forward_m
                + right_m * right_m
                + up_m * up_m
            )
            timeout_s = min(
                120.0,
                max(8.0, distance_m / self.ASSUMED_SPEED_M_S * 3.0 + 5.0),
            )
            final_pose = self._wait_for_position(
                target_x_m,
                target_y_m,
                target_z_m,
                timeout_s,
            )
            return {
                "ok": True,
                "message": "move_relative_xyz done",
                "command": command,
                "pose": self._public_pose(final_pose),
                "target_ros_m": {
                    "x": target_x_m,
                    "y": target_y_m,
                    "z": target_z_m,
                    "frame_id": str(start["frame_id"]),
                },
            }

    def move_relative_xyz_yaw(
        self,
        x: int,
        y: int,
        z: int,
        yaw: int,
    ) -> Dict[str, Any]:
        if int(yaw) != 0:
            raise NotImplementedError(
                "OWL yaw control is not validated; use XYZ-only commands"
            )
        return self.move_relative_xyz(x=x, y=y, z=z)

    def rotate(self, angle_deg: int) -> Dict[str, Any]:
        if int(angle_deg) == 0:
            return {
                "ok": True,
                "message": "rotate skipped",
                "command": {"angle_deg": 0},
                "pose": self.get_pose()["pose"],
            }
        raise NotImplementedError(
            "OWL yaw control is not validated; /reference/yawsetpoint and "
            "ExtTurnAroundTask still require verification"
        )

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

    def end(self) -> Dict[str, Any]:
        # Landing remains an explicit operator action for the first version.
        self._navigation_started = False
        return {
            "ok": True,
            "message": "OWL session ended; no landing command was sent",
            "health": self.health(),
        }

    def health(self) -> Dict[str, Any]:
        result = dict(self._hardware.health())
        result["navigation_started"] = self._navigation_started
        return result

    def _ensure_navigation_started(self) -> None:
        if self._navigation_started:
            return
        self._hardware.publish_control(
            self.NAVIGATION_TASK_ID,
            self.DRONE_ID,
        )
        self._navigation_started = True
        if self._navigation_start_delay_s > 0:
            time.sleep(self._navigation_start_delay_s)

    def _wait_for_position(
        self,
        target_x_m: float,
        target_y_m: float,
        target_z_m: float,
        timeout_s: float,
    ) -> Dict[str, Any]:
        deadline = time.monotonic() + float(timeout_s)
        period_s = 1.0 / self._position_poll_hz
        stable_samples = 0
        last_pose = self._hardware.get_pose_ros()
        while time.monotonic() < deadline:
            last_pose = self._hardware.get_pose_ros()
            error_cm = 100.0 * math.sqrt(
                (target_x_m - float(last_pose["x_m"])) ** 2
                + (target_y_m - float(last_pose["y_m"])) ** 2
                + (target_z_m - float(last_pose["z_m"])) ** 2
            )
            if error_cm <= self._position_tolerance_cm:
                stable_samples += 1
                if stable_samples >= self._position_stable_samples:
                    return last_pose
            else:
                stable_samples = 0
            time.sleep(period_s)
        raise RuntimeError(
            "position move timeout: "
            f"target=({target_x_m:.3f}, {target_y_m:.3f}, {target_z_m:.3f})m "
            f"pose=({float(last_pose['x_m']):.3f}, "
            f"{float(last_pose['y_m']):.3f}, "
            f"{float(last_pose['z_m']):.3f})m"
        )

    def _require_control_ready(self) -> None:
        health = self.health()
        if health.get("initialized") is not True:
            raise RuntimeError("OWL is not initialized or MAVROS is disconnected")
        if health.get("airborne") is not True:
            raise RuntimeError("OWL is not airborne; take off manually first")
        if str(health.get("mode", "")).upper() != "OFFBOARD":
            raise RuntimeError(
                f"OWL must be in OFFBOARD mode, current mode={health.get('mode')!r}"
            )
        if health.get("odom_ok") is not True:
            raise RuntimeError("OWL odometry is unavailable or stale")

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
            if frame_bgr.shape[:2] != (self.OUTPUT_HEIGHT, self.OUTPUT_WIDTH):
                frame_bgr = cv2.resize(
                    frame_bgr,
                    (self.OUTPUT_WIDTH, self.OUTPUT_HEIGHT),
                    interpolation=cv2.INTER_AREA,
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
    def _as_bool(value: Any) -> bool:
        if isinstance(value, str):
            return value.strip().lower() not in {"false", "0", "no", "off"}
        return bool(value)

    @staticmethod
    def _timestamped_name(prefix: str, suffix: str) -> str:
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return f"{prefix}_{timestamp}{suffix}"


__all__ = ["OwlController"]
