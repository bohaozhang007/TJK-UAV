from __future__ import annotations

import datetime as dt
import math
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from ..hardware.ue import SimDrone


class UEController:
    """Adapter from the Robot Server controller protocol to SimDrone."""

    MOVE_SPEED = SimDrone.MOVE_SPEED
    POSITION_TOLERANCE_CM = 5.0
    VERTICAL_TOLERANCE_CM = 4.0
    STABLE_SPEED_CM_S = 3.0
    POSITION_STABLE_SAMPLES = 4
    YAW_TOLERANCE_DEG = 2.0
    YAW_STABLE_RATE_DEG_S = 5.0
    YAW_STABLE_SAMPLES = 4
    CONTROL_HZ = 40.0

    # Faster response away from the target, with stronger derivative damping
    # near the target. Position/yaw tolerances remain unchanged.
    TRANSLATION_KP = 0.65
    TRANSLATION_KD = 0.16
    YAW_KP = 2.2
    YAW_KD = 0.12
    # SimDrone maps 50% yaw to the Blueprint's full normalized yaw input.
    MAX_YAW_COMMAND = 50.0

    def __init__(
        self,
        image_dir: str | Path | None = None,
        simulator: SimDrone | None = None,
    ) -> None:
        self._sim = simulator or SimDrone()
        self._operation_lock = threading.RLock()
        self._image_lock = threading.RLock()
        self._xy_origin: tuple[float, float] | None = None
        self._image_dir = Path(image_dir)
        self._image_dir.mkdir(parents=True, exist_ok=True)

    def init(self) -> dict[str, Any]:
        with self._operation_lock:
            was_connected = self._sim.connected
            result = self._sim.connect()
            if not was_connected or self._xy_origin is None:
                world_pose = self._sim.get_pose()
                self._xy_origin = (
                    float(world_pose["x"]),
                    float(world_pose["y"]),
                )
            return {
                "ok": bool(result.get("ok", True)),
                "message": result.get("message", "initialized"),
                "health": self.health(),
            }

    def takeoff(self) -> dict[str, Any]:
        with self._operation_lock:
            self._require_initialized()
            result = self._sim.takeoff()
            return {
                "ok": bool(result.get("ok", True)),
                "message": result.get("message", "takeoff done"),
                "pose": self._pose_copy(result.get("pose") or self._sim.get_pose()),
                "health": self.health(),
            }

    def get_rgb_meta(self, save: bool = True) -> dict[str, Any]:
        save = self._as_bool(save)
        frame_rgb = self._get_rgb()
        result: dict[str, Any] = {
            "ok": True,
            "height": int(frame_rgb.shape[0]),
            "width": int(frame_rgb.shape[1]),
        }
        if save:
            result["saved_to"] = str(self._save_rgb(frame_rgb))
        return result

    def get_rgb_byte(self) -> bytes:
        frame_rgb = self._get_rgb()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        success, encoded = cv2.imencode(".jpg", frame_bgr)
        if not success:
            raise RuntimeError("failed to encode Unreal RGB frame as JPEG")
        return encoded.tobytes()

    def get_depth_meta(self, save: bool = True) -> dict[str, Any]:
        save = self._as_bool(save)
        depth = self.get_depth_np()
        finite = depth[np.isfinite(depth)]
        result: dict[str, Any] = {
            "ok": True,
            "height": int(depth.shape[0]),
            "width": int(depth.shape[1]),
            "dtype": str(depth.dtype),
            "units": "cm",
            "min": float(finite.min()) if finite.size else None,
            "max": float(finite.max()) if finite.size else None,
        }
        if save:
            path = self._image_dir / self._timestamped_name("depth", ".npy")
            np.save(path, depth, allow_pickle=False)
            result["saved_to"] = str(path)
        return result

    def get_depth_np(self) -> np.ndarray:
        self._require_initialized()
        with self._image_lock:
            depth = np.asarray(self._sim.get_depth(), dtype=np.float32)
        if depth.ndim == 3 and depth.shape[-1] == 1:
            depth = depth[:, :, 0]
        if depth.ndim != 2:
            raise RuntimeError(f"depth must be HxW, got {depth.shape}")
        return depth

    def velocity(self, x: int, y: int, z: int, yaw: int) -> dict[str, Any]:
        with self._operation_lock:
            self._require_initialized()
            command = {
                "x": self._clamp_percent(x),
                "y": self._clamp_percent(y),
                "z": self._clamp_percent(z),
                "yaw": self._clamp_percent(yaw),
            }
            if any(command.values()):
                self._require_airborne()
            self._sim.set_velocity(**command)
            return {
                "ok": True,
                "message": "velocity command sent",
                "command": command,
            }

    def move_relative_xyz(self, x: int, y: int, z: int) -> dict[str, Any]:
        with self._operation_lock:
            self._require_initialized()
            self._require_airborne()
            command = {
                "x": int(x),
                "y": int(y),
                "z": int(z),
                "speed": self.MOVE_SPEED,
            }
            start = self._sim.get_pose()
            if not any((command["x"], command["y"], command["z"])):
                return {
                    "ok": True,
                    "message": "move_relative_xyz skipped",
                    "command": command,
                    "pose": self._pose_copy(start),
                }

            yaw_rad = math.radians(float(start["yaw"]))
            target = {
                "x": float(start["x"])
                + math.cos(yaw_rad) * command["x"]
                - math.sin(yaw_rad) * command["y"],
                "y": float(start["y"])
                + math.sin(yaw_rad) * command["x"]
                + math.cos(yaw_rad) * command["y"],
                "z": float(start["z"]) + command["z"],
                "yaw": float(start["yaw"]),
            }
            distance = math.sqrt(
                command["x"] ** 2 + command["y"] ** 2 + command["z"] ** 2
            )
            timeout_s = min(
                120.0,
                max(8.0, distance / max(1.0, float(command["speed"])) * 4.0 + 5.0),
            )
            pose = self._fly_to_position(target, command["speed"], timeout_s)
            return {
                "ok": True,
                "message": "move_relative_xyz done",
                "command": command,
                "pose": self._pose_copy(pose),
            }

    def move_relative_xyz_yaw(
        self,
        x: int,
        y: int,
        z: int,
        yaw: int,
    ) -> dict[str, Any]:
        """Move to one body-relative XYZ/yaw target in a single 4-axis loop."""
        with self._operation_lock:
            self._require_initialized()
            self._require_airborne()
            command = {
                "x": int(x),
                "y": int(y),
                "z": int(z),
                "yaw": int(yaw),
                "speed": self.MOVE_SPEED,
            }
            start = self._sim.get_pose()
            if not any(
                (
                    command["x"],
                    command["y"],
                    command["z"],
                    command["yaw"],
                )
            ):
                return {
                    "ok": True,
                    "message": "move_relative_xyz_yaw skipped",
                    "command": command,
                    "pose": self._pose_copy(start),
                }

            start_yaw_rad = math.radians(float(start["yaw"]))
            target = {
                "x": float(start["x"])
                + math.cos(start_yaw_rad) * command["x"]
                - math.sin(start_yaw_rad) * command["y"],
                "y": float(start["y"])
                + math.sin(start_yaw_rad) * command["x"]
                + math.cos(start_yaw_rad) * command["y"],
                "z": float(start["z"]) + command["z"],
                "yaw": self._normalize_yaw(
                    float(start["yaw"]) + command["yaw"]
                ),
            }
            distance = math.sqrt(
                command["x"] ** 2
                + command["y"] ** 2
                + command["z"] ** 2
            )
            translation_timeout_s = (
                distance / max(1.0, float(command["speed"])) * 4.0 + 5.0
            )
            rotation_timeout_s = abs(command["yaw"]) / 20.0 + 5.0
            timeout_s = min(
                120.0,
                max(8.0, translation_timeout_s, rotation_timeout_s),
            )
            pose = self._fly_to_position(
                target,
                command["speed"],
                timeout_s,
                require_yaw=True,
            )
            return {
                "ok": True,
                "message": "move_relative_xyz_yaw done",
                "command": command,
                "pose": self._pose_copy(pose),
            }

    def rotate(self, angle_deg: int) -> dict[str, Any]:
        with self._operation_lock:
            self._require_initialized()
            self._require_airborne()
            angle = int(angle_deg)
            if angle == 0:
                return {
                    "ok": True,
                    "message": "rotate skipped",
                    "direction": "none",
                    "command": {"angle_deg": 0},
                    "pose": self._pose_copy(self._sim.get_pose()),
                }

            start = self._sim.get_pose()
            target_yaw = self._normalize_yaw(float(start["yaw"]) + angle)
            timeout_s = min(60.0, max(6.0, abs(angle) / 20.0 + 5.0))
            pose = self._rotate_to_yaw(target_yaw, timeout_s)
            return {
                "ok": True,
                "message": "rotate done",
                "direction": "clockwise" if angle > 0 else "counter_clockwise",
                "command": {"angle_deg": abs(angle)},
                "pose": self._pose_copy(pose),
            }

    def get_pose(self) -> dict[str, Any]:
        self._require_initialized()
        return {
            "ok": True,
            "pose": self._pose_copy(self._sim.get_pose()),
            "units": {"position": "cm", "yaw": "degree"},
            "estimated": False,
            "sources": {
                "x": "Unreal object pose relative to initialization",
                "y": "Unreal object pose relative to initialization",
                "z": "Unreal object absolute pose",
                "yaw": "Unreal object absolute pose",
            },
        }

    def get_motion_tolerances(self) -> dict[str, Any]:
        """Return a conservative 3D bound for UE pose completion."""
        return {
            "position_tolerance_cm": math.hypot(
                self.POSITION_TOLERANCE_CM,
                self.VERTICAL_TOLERANCE_CM,
            ),
            "yaw_tolerance_deg": self.YAW_TOLERANCE_DEG,
            "position_error_metric": "euclidean_3d",
            "source": "pose_completion_enclosing_bound",
        }

    def land(self) -> dict[str, Any]:
        with self._operation_lock:
            if not self._sim.connected:
                return {
                    "ok": True,
                    "message": "simulator is not connected; no landing command sent",
                    "health": self.health(),
                }
            if not self._sim.airborne:
                return {
                    "ok": True,
                    "message": "already landed; simulator communication remains active",
                    "health": self.health(),
                }
            result = self._sim.land()
            return {
                "ok": bool(result.get("ok", True)),
                "message": result.get(
                    "message",
                    "landed; simulator communication remains active",
                ),
                "pose": self._pose_copy(result.get("pose") or self._sim.get_pose()),
                "health": self.health(),
            }

    def close(self) -> dict[str, Any]:
        with self._operation_lock:
            if not self._sim.connected:
                self._xy_origin = None
                return {"ok": True, "message": "already closed", "health": self.health()}

            first_error: Exception | None = None
            if self._sim.airborne:
                try:
                    landing = self.land()
                    if not landing.get("ok", False):
                        first_error = RuntimeError(
                            str(landing.get("error") or landing.get("message"))
                        )
                except Exception as exc:
                    first_error = exc
            result = self._sim.close()
            self._xy_origin = None
            if not result.get("ok", True) and first_error is None:
                first_error = RuntimeError(str(result.get("error") or result.get("message")))
            if first_error is not None:
                return {
                    "ok": False,
                    "message": "closed with landing or cleanup error",
                    "error": str(first_error),
                    "health": self.health(),
                }
            return {"ok": True, "message": "closed", "health": self.health()}

    def health(self) -> dict[str, Any]:
        return dict(self._sim.health())

    def _fly_to_position(
        self,
        target: dict[str, float],
        max_command: int,
        timeout_s: float,
        require_yaw: bool = False,
    ) -> dict[str, float]:
        period = 1.0 / self.CONTROL_HZ
        deadline = time.monotonic() + float(timeout_s)
        previous_pose = self._sim.get_pose()
        previous_time = time.monotonic()
        position_stable_samples = 0
        yaw_stable_samples = 0

        try:
            while time.monotonic() < deadline:
                loop_start = time.monotonic()
                pose = self._sim.get_pose()
                sample_dt = max(1e-3, loop_start - previous_time)
                vx_world = (float(pose["x"]) - float(previous_pose["x"])) / sample_dt
                vy_world = (float(pose["y"]) - float(previous_pose["y"])) / sample_dt
                vz = (float(pose["z"]) - float(previous_pose["z"])) / sample_dt
                yaw_delta = self._normalize_yaw(
                    float(pose["yaw"]) - float(previous_pose["yaw"])
                )
                yaw_rate = yaw_delta / sample_dt

                dx = target["x"] - float(pose["x"])
                dy = target["y"] - float(pose["y"])
                dz = target["z"] - float(pose["z"])
                yaw_rad = math.radians(float(pose["yaw"]))
                forward_error = math.cos(yaw_rad) * dx + math.sin(yaw_rad) * dy
                right_error = -math.sin(yaw_rad) * dx + math.cos(yaw_rad) * dy
                forward_speed = math.cos(yaw_rad) * vx_world + math.sin(yaw_rad) * vy_world
                right_speed = -math.sin(yaw_rad) * vx_world + math.cos(yaw_rad) * vy_world

                horizontal_error = math.hypot(dx, dy)
                horizontal_speed = math.hypot(vx_world, vy_world)
                position_stable = (
                    horizontal_error <= self.POSITION_TOLERANCE_CM
                    and abs(dz) <= self.VERTICAL_TOLERANCE_CM
                    and horizontal_speed <= self.STABLE_SPEED_CM_S
                    and abs(vz) <= self.STABLE_SPEED_CM_S
                )
                position_stable_samples = (
                    position_stable_samples + 1 if position_stable else 0
                )

                yaw_error = self._normalize_yaw(
                    target["yaw"] - float(pose["yaw"])
                )
                yaw_stable = (
                    abs(yaw_error) <= self.YAW_TOLERANCE_DEG
                    and abs(yaw_rate) <= self.YAW_STABLE_RATE_DEG_S
                )
                yaw_stable_samples = (
                    yaw_stable_samples + 1 if yaw_stable else 0
                )
                if (
                    position_stable_samples >= self.POSITION_STABLE_SAMPLES
                    and (
                        not require_yaw
                        or yaw_stable_samples >= self.YAW_STABLE_SAMPLES
                    )
                ):
                    return pose

                command_x = self._pd_command(forward_error, forward_speed, max_command)
                command_y = self._pd_command(right_error, right_speed, max_command)
                command_z = self._pd_command(dz, vz, max_command)
                command_yaw = max(
                    -self.MAX_YAW_COMMAND,
                    min(
                        self.MAX_YAW_COMMAND,
                        self.YAW_KP * yaw_error - self.YAW_KD * yaw_rate,
                    ),
                )
                if abs(yaw_error) <= self.YAW_TOLERANCE_DEG:
                    command_yaw = 0.0

                self._sim.set_velocity(command_x, command_y, command_z, command_yaw)
                previous_pose = pose
                previous_time = loop_start
                remaining = period - (time.monotonic() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)
        finally:
            self._sim.stop()

        pose = self._sim.get_pose()
        raise RuntimeError(
            "waypoint timeout: "
            f"target=({target['x']:.1f}, {target['y']:.1f}, "
            f"{target['z']:.1f}, yaw={target['yaw']:.1f}) "
            f"pose=({pose['x']:.1f}, {pose['y']:.1f}, "
            f"{pose['z']:.1f}, yaw={pose['yaw']:.1f})"
        )

    def _rotate_to_yaw(self, target_yaw: float, timeout_s: float) -> dict[str, float]:
        period = 1.0 / self.CONTROL_HZ
        deadline = time.monotonic() + float(timeout_s)
        previous_pose = self._sim.get_pose()
        previous_time = time.monotonic()
        stable_samples = 0

        try:
            while time.monotonic() < deadline:
                loop_start = time.monotonic()
                pose = self._sim.get_pose()
                sample_dt = max(1e-3, loop_start - previous_time)
                yaw_delta = self._normalize_yaw(
                    float(pose["yaw"]) - float(previous_pose["yaw"])
                )
                yaw_rate = yaw_delta / sample_dt
                error = self._normalize_yaw(target_yaw - float(pose["yaw"]))

                stable = (
                    abs(error) <= self.YAW_TOLERANCE_DEG
                    and abs(yaw_rate) <= self.YAW_STABLE_RATE_DEG_S
                )
                stable_samples = stable_samples + 1 if stable else 0
                if stable_samples >= self.YAW_STABLE_SAMPLES:
                    return pose

                command_yaw = max(
                    -self.MAX_YAW_COMMAND,
                    min(
                        self.MAX_YAW_COMMAND,
                        self.YAW_KP * error - self.YAW_KD * yaw_rate,
                    ),
                )
                if abs(error) <= self.YAW_TOLERANCE_DEG:
                    command_yaw = 0.0
                self._sim.set_velocity(0, 0, 0, command_yaw)
                previous_pose = pose
                previous_time = loop_start
                remaining = period - (time.monotonic() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)
        finally:
            self._sim.stop()

        pose = self._sim.get_pose()
        raise RuntimeError(
            f"rotation timeout: target_yaw={target_yaw:.1f}, yaw={pose['yaw']:.1f}"
        )

    def _get_rgb(self) -> np.ndarray:
        self._require_initialized()
        with self._image_lock:
            frame = np.asarray(self._sim.get_rgb(), dtype=np.uint8)
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise RuntimeError(f"RGB frame must be HxWx3, got {frame.shape}")
        return frame

    def _save_rgb(self, frame_rgb: np.ndarray) -> Path:
        path = self._image_dir / self._timestamped_name("image", ".jpg")
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(str(path), frame_bgr):
            raise RuntimeError(f"failed to save RGB frame: {path}")
        return path

    def _require_initialized(self) -> None:
        if not self._sim.connected:
            raise RuntimeError("drone is not initialized, call init first")

    def _require_airborne(self) -> None:
        if not self._sim.airborne:
            raise RuntimeError("drone is not airborne, call takeoff first")

    @classmethod
    def _pd_command(cls, error: float, velocity: float, limit: int) -> float:
        command = (
            cls.TRANSLATION_KP * float(error)
            - cls.TRANSLATION_KD * float(velocity)
        )
        return max(-float(limit), min(float(limit), command))

    def _pose_copy(self, pose: dict[str, Any]) -> dict[str, float]:
        if self._xy_origin is None:
            raise RuntimeError("UE pose origin is not initialized")
        origin_x, origin_y = self._xy_origin
        return {
            "x": float(pose["x"]) - origin_x,
            "y": float(pose["y"]) - origin_y,
            "z": float(pose["z"]),
            "yaw": UEController._normalize_yaw(float(pose["yaw"])),
        }

    @staticmethod
    def _normalize_yaw(value: float) -> float:
        return (float(value) + 180.0) % 360.0 - 180.0

    @staticmethod
    def _clamp_percent(value: int | float) -> int:
        return max(-100, min(100, int(value)))

    @staticmethod
    def _as_bool(value: Any) -> bool:
        if isinstance(value, str):
            return value.strip().lower() not in {"false", "0", "no", "off"}
        return bool(value)

    @staticmethod
    def _timestamped_name(prefix: str, suffix: str) -> str:
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return f"{prefix}_{timestamp}{suffix}"
