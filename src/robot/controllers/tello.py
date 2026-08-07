from __future__ import annotations

import datetime as dt
import math
import threading
from pathlib import Path
from typing import Any

import cv2
from ..hardware.tello import MOVE_SPEED as TELLO_MOVE_SPEED, Tello


class TelloController:
    """Thread-safe controller for one physical Tello drone."""

    DEFAULT_TAKEOFF_HEIGHT_CM = 100.0
    MOVE_SPEED = TELLO_MOVE_SPEED

    def __init__(self, image_dir: str | Path | None = None) -> None:
        self._lock = threading.RLock()
        self._frame_lock = threading.RLock()
        self._tello: Tello | None = None
        self._frame_read = None
        self._initialized = False
        self._airborne = False
        self._pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self._height_origin_cm = 0.0
        self._yaw_origin_deg: float | None = None
        self._image_dir = Path(image_dir)
        self._image_dir.mkdir(parents=True, exist_ok=True)

    def init(self) -> dict[str, Any]:
        with self._lock:
            if self._initialized:
                return {
                    "ok": True,
                    "message": "already initialized",
                    "health": self.health(),
                }

            tello = Tello()
            frame_read = None
            try:
                tello.connect()
                tello.streamon()
                frame_read = tello.get_frame_read()
            except Exception:
                self._cleanup_tello(tello, frame_read)
                raise

            with self._frame_lock:
                self._tello = tello
                self._frame_read = frame_read
                self._initialized = True
                self._airborne = False
                self._pose.update(x=0.0, y=0.0, z=0.0, yaw=0.0)
                self._initialize_pose_reference(tello)

            return {
                "ok": True,
                "message": "initialized",
                "health": self.health(),
            }

    def takeoff(self) -> dict[str, Any]:
        with self._lock:
            tello = self._require_tello()
            if self._airborne:
                return {
                    "ok": True,
                    "message": "already airborne",
                    "health": self.health(),
                }

            tello.takeoff()
            self._airborne = True
            updated_axes = self._refresh_telemetry_pose(tello)
            if "z" not in updated_axes:
                self._pose["z"] = self.DEFAULT_TAKEOFF_HEIGHT_CM
            return {
                "ok": True,
                "message": "takeoff done",
                "pose": self._pose_copy(),
                "health": self.health(),
            }

    def get_rgb_meta(self, save: bool = True) -> dict[str, Any]:
        if isinstance(save, str):
            save = save.strip().lower() not in {"false", "0", "no", "off"}
        else:
            save = bool(save)

        frame = self._get_latest_frame()
        output: dict[str, Any] = {
            "ok": True,
            "height": int(frame.shape[0]),
            "width": int(frame.shape[1]),
        }

        if save:
            output["saved_to"] = str(self._save_frame(frame, self._image_dir))
        return output

    def get_rgb_byte(self) -> bytes:
        return self._encode_frame_jpeg_bytes(self._get_latest_frame())

    def get_depth_meta(self, save: bool = True) -> dict[str, Any]:
        return {
            "ok": False,
            "error": "depth is not supported by TelloController",
        }

    def get_depth_np(self) -> dict[str, Any]:
        return {
            "ok": False,
            "error": "depth is not supported by TelloController",
        }

    def velocity(self, x: int, y: int, z: int, yaw: int) -> dict[str, Any]:
        with self._lock:
            tello = self._require_tello()
            vx = self._clamp(x)
            vy = self._clamp(y)
            vz = self._clamp(z)
            vyaw = self._clamp(yaw)

            # djitellopy order: left/right, forward/backward, up/down, yaw.
            tello.send_rc_control(vy, vx, vz, vyaw)
            return {
                "ok": True,
                "message": "velocity command sent",
                "command": {"x": vx, "y": vy, "z": vz, "yaw": vyaw},
            }

    def move_relative_xyz(self, x: int, y: int, z: int) -> dict[str, Any]:
        with self._lock:
            tello = self._require_tello()
            self._refresh_telemetry_pose(tello)
            heading_deg = self._pose["yaw"]
            command = {
                "x": int(x),
                "y": int(y),
                "z": int(z),
                "speed": self.MOVE_SPEED,
            }
            tello.go_xyz_speed(
                command["x"],
                command["y"],
                command["z"],
                command["speed"],
            )
            self._update_xy(command["x"], command["y"], heading_deg)
            updated_axes = self._refresh_telemetry_pose(tello)
            if "z" not in updated_axes:
                self._pose["z"] += command["z"]
            return {
                "ok": True,
                "message": "move_relative_xyz command sent",
                "command": command,
                "pose": self._pose_copy(),
            }

    def rotate(self, angle_deg: int) -> dict[str, Any]:
        with self._lock:
            tello = self._require_tello()
            angle = int(angle_deg)
            if angle == 0:
                return {
                    "ok": True,
                    "message": "rotate skipped",
                    "direction": "none",
                    "command": {"angle_deg": 0},
                }

            if angle > 0:
                tello.rotate_clockwise(angle)
                direction = "clockwise"
                executed_angle = angle
            else:
                executed_angle = abs(angle)
                tello.rotate_counter_clockwise(executed_angle)
                direction = "counter_clockwise"

            updated_axes = self._refresh_telemetry_pose(tello)
            if "yaw" not in updated_axes:
                self._pose["yaw"] = self._normalize_yaw(self._pose["yaw"] + angle)
            return {
                "ok": True,
                "message": "rotate command sent",
                "direction": direction,
                "command": {"angle_deg": executed_angle},
                "pose": self._pose_copy(),
            }

    def get_pose(self) -> dict[str, Any]:
        with self._lock:
            if self._initialized and self._tello is not None:
                self._refresh_telemetry_pose(self._tello)
            return {
                "ok": True,
                "pose": self._pose_copy(),
                "units": {"position": "cm", "yaw": "degree"},
                "estimated": True,
                "sources": {
                    "x": "move_relative_xyz estimate",
                    "y": "move_relative_xyz estimate",
                    "z": "Tello height telemetry",
                    "yaw": "Tello attitude telemetry",
                },
            }

    def land(self) -> dict[str, Any]:
        with self._lock:
            if not self._initialized:
                return {
                    "ok": True,
                    "message": "Tello is not initialized; no landing command sent",
                    "health": self.health(),
                }

            tello = self._require_tello()
            if not self._airborne:
                return {
                    "ok": True,
                    "message": "already landed; Tello communication remains active",
                    "health": self.health(),
                }

            try:
                tello.land()
                self._airborne = False
                self._refresh_telemetry_pose(tello)
                self._pose["z"] = 0.0
            except Exception as exc:
                return {
                    "ok": False,
                    "message": "failed to land",
                    "error": str(exc),
                    "health": self.health(),
                }
            return {
                "ok": True,
                "message": "landed; Tello communication remains active",
                "health": self.health(),
            }

    def close(self) -> dict[str, Any]:
        with self._lock:
            if not self._initialized:
                return {
                    "ok": True,
                    "message": "already closed",
                    "health": self.health(),
                }

            landing_error = None
            if self._airborne:
                landing = self.land()
                if not landing.get("ok", False):
                    landing_error = RuntimeError(
                        str(landing.get("error") or landing.get("message"))
                    )

            tello = self._require_tello()

            with self._frame_lock:
                frame_read = self._frame_read
                self._frame_read = None

            cleanup_error = self._cleanup_tello(tello, frame_read)
            with self._frame_lock:
                self._tello = None
                self._initialized = False
                self._airborne = False

            first_error = landing_error or cleanup_error
            if first_error is not None:
                return {
                    "ok": False,
                    "message": "closed with landing or cleanup error",
                    "error": str(first_error),
                    "health": self.health(),
                }
            return {"ok": True, "message": "closed", "health": self.health()}

    def health(self) -> dict[str, Any]:
        with self._lock:
            battery = None
            if self._initialized and self._tello is not None:
                try:
                    battery = self._tello.get_battery()
                except Exception as exc:
                    battery = f"error: {exc}"
            return {
                "initialized": self._initialized,
                "airborne": self._airborne,
                "battery": battery,
            }

    def _get_latest_frame(self):
        # Video decoding runs in djitellopy's own thread. A separate short-held
        # lock keeps snapshots safe without waiting for blocking flight commands.
        with self._frame_lock:
            if not self._initialized:
                raise RuntimeError("drone is not initialized, call init first")
            if self._frame_read is None:
                raise RuntimeError("frame reader not ready")
            frame = self._frame_read.frame
            if frame is None:
                raise RuntimeError("empty frame")
            return frame.copy()

    def _require_tello(self) -> Tello:
        if not self._initialized or self._tello is None:
            raise RuntimeError("drone is not initialized, call init first")
        return self._tello

    def _initialize_pose_reference(self, tello: Tello) -> None:
        try:
            self._height_origin_cm = float(tello.get_height())
        except Exception:
            self._height_origin_cm = 0.0

        try:
            self._yaw_origin_deg = float(tello.get_yaw())
        except Exception:
            self._yaw_origin_deg = None

        self._refresh_telemetry_pose(tello)

    def _refresh_telemetry_pose(self, tello: Tello) -> set[str]:
        updated_axes: set[str] = set()
        try:
            height_cm = float(tello.get_height())
            self._pose["z"] = height_cm - self._height_origin_cm
            updated_axes.add("z")
        except Exception:
            pass

        try:
            yaw_deg = float(tello.get_yaw())
            if self._yaw_origin_deg is None:
                self._yaw_origin_deg = yaw_deg
            self._pose["yaw"] = self._normalize_yaw(yaw_deg - self._yaw_origin_deg)
            updated_axes.add("yaw")
        except Exception:
            pass
        return updated_axes

    def _update_xy(self, x: float, y: float, heading_deg: float) -> None:
        # move_relative_xyz uses body-relative coordinates. Convert them to the
        # initial world frame using x=forward, y=right and clockwise-positive yaw.
        theta = math.radians(heading_deg)
        world_x = math.cos(theta) * x - math.sin(theta) * y
        world_y = math.sin(theta) * x + math.cos(theta) * y
        self._pose["x"] += world_x
        self._pose["y"] += world_y

    def _pose_copy(self) -> dict[str, float]:
        return {name: float(value) for name, value in self._pose.items()}

    @staticmethod
    def _normalize_yaw(value: float) -> float:
        return (float(value) + 180.0) % 360.0 - 180.0

    @staticmethod
    def _cleanup_tello(tello, frame_read=None) -> Exception | None:
        first_error = None
        if frame_read is not None and hasattr(frame_read, "stop"):
            try:
                frame_read.stop()
            except Exception as exc:
                first_error = first_error or exc

        try:
            tello.streamoff()
        except Exception as exc:
            first_error = first_error or exc

        try:
            tello.end()
        except Exception as exc:
            first_error = first_error or exc
        return first_error

    @staticmethod
    def _encode_frame_jpeg_bytes(frame) -> bytes:
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        success, encoded = cv2.imencode(".jpg", frame_bgr)
        if not success:
            raise RuntimeError("failed to encode frame as jpeg")
        return encoded.tobytes()

    def _save_frame(self, frame, directory: Path) -> Path:
        directory.mkdir(parents=True, exist_ok=True)
        file_path = directory / f"image_{dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.jpg"

        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(str(file_path), frame_bgr):
            raise RuntimeError("failed to save image")
        return file_path

    @staticmethod
    def _clamp(value: int) -> int:
        return max(-100, min(100, int(value)))
