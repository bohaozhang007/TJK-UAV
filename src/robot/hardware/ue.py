from __future__ import annotations

import json
import socket
import threading
import time
import warnings
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

from unrealcv.api import UnrealCv_API
from unrealcv.launcher import RunUnreal


class SimDrone:
    """Minimal single-drone UnrealCV hardware driver.

    Public motion coordinates follow the Tello-facing server convention:
    ``x`` is forward, ``y`` is right, ``z`` is up, and positive yaw is
    clockwise. Positions are centimetres and yaw is degrees.
    """

    # Closed-loop waypoint moves may use up to 70% of the Blueprint's
    # available velocity. Final accuracy is enforced by UEController's pose
    # feedback rather than by keeping the entire trajectory artificially slow.
    MOVE_SPEED = 70
    DEFAULT_ENV_BIN = (
        Path.home()
        / ".unrealcv"
        / "UnrealEnv"
        / "Collection_WinNoEditor"
        / "Collection"
        / "Binaries"
        / "Win64"
        / "Collection.exe"
    )
    DEFAULT_SETTING_DIR = (
        Path.home()
        / "unrealzoo-gym"
        / "gym_unrealcv"
        / "envs"
        / "setting"
        / "Track"
    )
    # DEFAULT_INITIAL_POSE = (30.0, 970.0, 520.0, -60.0)
    # DEFAULT_INITIAL_POSE = (-24210.0, 1072.0, 323.0, -60.0) 适用于 <= v8
    DEFAULT_INITIAL_POSE = (-24189.0, 278.0, 437.0, 0.0) # >= v9

    def __init__(
        self,
        *,
        env_map: str = "DowntownWest",
        resolution: tuple[int, int] = (640, 480),
        initial_pose: Iterable[float] | None = DEFAULT_INITIAL_POSE,
        start_airborne: bool = True,
        takeoff_height_cm: float = 100.0,
        offscreen: bool = False,
        gpu_id: int | None = None,
        launch_sleep_s: float = 10.0,
        command_timeout_s: float = 0.75,
    ) -> None:
        self.env_bin = self.DEFAULT_ENV_BIN
        self.env_map = str(env_map)
        self.setting_file = self.DEFAULT_SETTING_DIR / f"{self.env_map}.json"
        setting = self._load_setting(self.setting_file)
        drone_setting = self._load_first_drone_setting(setting, self.setting_file)
        self.drone_name = drone_setting["name"]
        self.drone_scale = drone_setting["scale"]
        self.camera_relative_location = drone_setting["relative_location"]
        self.camera_relative_rotation = drone_setting["relative_rotation"]
        self.other_agent_names = self._load_other_agent_names(setting, self.drone_name)
        self.removed_agent_names: tuple[str, ...] = ()
        self.configured_camera_id = drone_setting["camera_id"]
        self.camera_id = self.configured_camera_id
        self.camera_name = ""
        self.camera_id_source = "setting"
        self.resolution = (int(resolution[0]), int(resolution[1]))
        self.initial_pose, self.initial_pose_source = self._resolve_initial_pose(
            initial_pose,
            drone_setting,
            setting,
            self.setting_file,
        )

        self.start_airborne = bool(start_airborne)
        self.takeoff_height_cm = float(takeoff_height_cm)
        self.offscreen = bool(offscreen)
        self.gpu_id = gpu_id
        self.launch_sleep_s = float(launch_sleep_s)
        self.command_timeout_s = max(0.0, float(command_timeout_s))

        self._io_lock = threading.RLock()
        self._state_lock = threading.RLock()
        self._launcher: RunUnreal | None = None
        self._api: UnrealCv_API | None = None
        self._connected = False
        self._airborne = False
        self._landing_z_cm: float | None = None
        self._last_velocity_at = 0.0
        self._velocity_active = False
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        with self._state_lock:
            return self._connected

    @property
    def airborne(self) -> bool:
        with self._state_lock:
            return self._airborne

    def connect(self) -> dict[str, Any]:
        with self._state_lock:
            if self._connected:
                return {"ok": True, "message": "already connected", "health": self.health()}

        if not self.env_bin.is_file():
            raise FileNotFoundError(f"Unreal environment binary not found: {self.env_bin}")

        launcher: RunUnreal | None = None
        api: UnrealCv_API | None = None
        try:
            launcher = RunUnreal(ENV_BIN=str(self.env_bin), ENV_MAP=self.env_map)
            ip, port = launcher.start(
                resolution=self.resolution,
                offscreen=self.offscreen,
                gpu_id=self.gpu_id,
                sleep_time=self.launch_sleep_s,
            )
            api = UnrealCv_API(
                port=port,
                ip=ip,
                resolution=self.resolution,
                mode="tcp",
            )
            api.set_map(self.env_map)

            with self._state_lock:
                self._launcher = launcher
                self._api = api

            self._verify_drone()
            self._remove_other_agents()
            self._set_drone_scale()
            self._set_velocity_raw(0.0, 0.0, 0.0, 0.0)
            if self.initial_pose is not None:
                self._set_world_pose(*self.initial_pose)
            self._set_drone_camera()
            with self._io_lock:
                self._request(f"vbp {self.drone_name} set_viewport")
            self._resolve_camera_id()

            raw_pose = self.get_world_pose()
            with self._state_lock:
                self._landing_z_cm = raw_pose["z"]
                self._connected = True
                self._airborne = self.start_airborne
                self._velocity_active = False

            self._start_watchdog()
            return {"ok": True, "message": "connected", "health": self.health()}
        except Exception:
            if api is not None:
                try:
                    self._disconnect_unrealcv_client(api.client)
                except Exception:
                    pass
            if launcher is not None:
                try:
                    launcher.close()
                except Exception:
                    pass
            with self._state_lock:
                self._launcher = None
                self._api = None
                self._connected = False
            raise

    def takeoff(self, height_cm: float | None = None) -> dict[str, Any]:
        self._require_connected()
        with self._state_lock:
            if self._airborne:
                return {"ok": True, "message": "already airborne", "pose": self.get_pose()}

        height = self.takeoff_height_cm if height_cm is None else float(height_cm)
        if height <= 0:
            raise ValueError("takeoff height must be positive")

        world_pose = self.get_world_pose()
        self._set_velocity_raw(0.0, 0.0, 0.0, 0.0)
        self._set_world_pose(
            world_pose["x"],
            world_pose["y"],
            world_pose["z"] + height,
            world_pose["yaw"],
        )
        with self._state_lock:
            self._airborne = True
        return {"ok": True, "message": "takeoff done", "pose": self.get_pose()}

    def land(self) -> dict[str, Any]:
        self._require_connected()
        self.stop()
        with self._state_lock:
            landing_z_cm = self._landing_z_cm
        if landing_z_cm is not None:
            world_pose = self.get_world_pose()
            self._set_world_pose(
                world_pose["x"],
                world_pose["y"],
                landing_z_cm,
                world_pose["yaw"],
            )
        with self._state_lock:
            self._airborne = False
        return {"ok": True, "message": "landed", "pose": self.get_pose()}

    def set_velocity(self, x: float, y: float, z: float, yaw: float) -> dict[str, Any]:
        """Send a Tello-compatible percentage command in the range [-100, 100]."""
        self._require_connected()
        command = tuple(self._clamp_percent(value) for value in (x, y, z, yaw))
        x_action = command[0] / 100.0
        y_action = command[1] / 100.0
        z_action = command[2] / 100.0
        yaw_action = float(np.clip(command[3] / 100.0 * 2.0, -1.0, 1.0))
        self._set_velocity_raw(x_action, y_action, z_action, yaw_action)
        with self._state_lock:
            self._last_velocity_at = time.monotonic()
            self._velocity_active = any(abs(value) > 1e-9 for value in command)
        return {
            "ok": True,
            "command": {"x": command[0], "y": command[1], "z": command[2], "yaw": command[3]},
        }

    def stop(self) -> dict[str, Any]:
        self._require_connected()
        self._set_velocity_raw(0.0, 0.0, 0.0, 0.0)
        with self._state_lock:
            self._velocity_active = False
            self._last_velocity_at = time.monotonic()
        return {"ok": True, "message": "stopped"}

    def get_world_pose(self) -> dict[str, float]:
        api = self._require_api()
        with self._io_lock:
            values = [float(value) for value in api.get_obj_pose(self.drone_name)]
        if len(values) < 5:
            raise RuntimeError(f"invalid pose returned for {self.drone_name}: {values}")
        return {
            "x": values[0],
            "y": values[1],
            "z": values[2],
            "yaw": self._normalize_yaw(values[4]),
        }

    def get_pose(self) -> dict[str, float]:
        return self.get_world_pose()

    def get_rgb(self) -> np.ndarray:
        """Return an HxWx3 uint8 RGB image."""
        api = self._require_api()
        with self._io_lock:
            frame_bgr = api.get_image(self.camera_id, "lit", mode="png")
        if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3:
            raise RuntimeError("UnrealCV returned an invalid RGB frame")
        return cv2.cvtColor(frame_bgr[:, :, :3], cv2.COLOR_BGR2RGB)

    def get_depth(self) -> np.ndarray:
        """Return an HxW float32 depth image in Unreal centimetres."""
        api = self._require_api()
        with self._io_lock:
            depth = np.asarray(api.get_depth(self.camera_id), dtype=np.float32)
        if depth.ndim == 3 and depth.shape[-1] == 1:
            depth = depth[:, :, 0]
        if depth.ndim != 2:
            raise RuntimeError(f"UnrealCV returned invalid depth shape: {depth.shape}")
        return depth

    def health(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "initialized": self._connected,
                "airborne": self._airborne,
                "simulator": "unrealcv",
                "map": self.env_map,
                "drone": self.drone_name,
                "drone_scale": list(self.drone_scale),
                "camera_id": self.camera_id,
                "configured_camera_id": self.configured_camera_id,
                "camera_name": self.camera_name,
                "camera_id_source": self.camera_id_source,
                "camera_relative_location": list(self.camera_relative_location),
                "camera_relative_rotation": list(self.camera_relative_rotation),
                "removed_agents": list(self.removed_agent_names),
                "setting_file": str(self.setting_file),
                "initial_pose_source": self.initial_pose_source,
                "starts_airborne": self.start_airborne,
            }

    def close(self) -> dict[str, Any]:
        with self._state_lock:
            if not self._connected and self._launcher is None and self._api is None:
                return {"ok": True, "message": "already closed", "health": self.health()}

        self._stop_watchdog()
        first_error: Exception | None = None
        try:
            if self._api is not None:
                with self._io_lock:
                    try:
                        self._set_velocity_raw(0.0, 0.0, 0.0, 0.0)
                    except Exception as exc:
                        first_error = first_error or exc
                    try:
                        self._disconnect_unrealcv_client(self._api.client)
                    except Exception as exc:
                        first_error = first_error or exc
        finally:
            if self._launcher is not None:
                try:
                    self._launcher.close()
                except Exception as exc:
                    first_error = first_error or exc
            with self._state_lock:
                self._api = None
                self._launcher = None
                self._connected = False
                self._airborne = False
                self._landing_z_cm = None
                self._velocity_active = False

        if first_error is not None:
            return {
                "ok": False,
                "message": "closed with cleanup error",
                "error": str(first_error),
                "health": self.health(),
            }
        return {"ok": True, "message": "closed", "health": self.health()}

    @staticmethod
    def _disconnect_unrealcv_client(client: Any, join_timeout_s: float = 5.0) -> None:
        """Stop UnrealCV's receiver before closing its socket.

        UnrealCV 1.1.7 closes the socket first. Its receiver then treats the
        intentional shutdown as a remote disconnect and tries to join itself,
        raising ``RuntimeError: cannot join current thread``.  Stopping the
        queue-driven receiver first avoids that shutdown race.
        """
        receiver = getattr(client, "t", None)
        if receiver is threading.current_thread():
            raise RuntimeError("cannot disconnect UnrealCV from its receiver thread")

        if receiver is not None and receiver.is_alive():
            receive_queue = getattr(client, "recv_num_q", None)
            if receive_queue is None:
                raise RuntimeError("UnrealCV client has no receiver control queue")
            receive_queue.put(None)
            receiver.join(timeout=max(0.0, float(join_timeout_s)))
            if receiver.is_alive():
                raise RuntimeError("UnrealCV receiver thread did not stop")

        sock = getattr(client, "sock", None)
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            finally:
                try:
                    sock.close()
                finally:
                    client.sock = None
        if receiver is not None:
            client.t = None

    def _verify_drone(self) -> None:
        object_names = self._get_object_names()
        if self.drone_name not in object_names:
            raise RuntimeError(f"drone object not found in Unreal scene: {self.drone_name}")

    def _remove_other_agents(self, timeout_s: float = 5.0) -> None:
        present_objects = set(self._get_object_names())
        targets = [name for name in self.other_agent_names if name in present_objects]
        if not targets:
            self.removed_agent_names = ()
            return

        with self._io_lock:
            for name in targets:
                self._request(f"vset /object/{name}/destroy")

        deadline = time.monotonic() + max(0.0, float(timeout_s))
        remaining = set(targets)
        while remaining and time.monotonic() < deadline:
            remaining.intersection_update(self._get_object_names())
            if remaining:
                time.sleep(0.05)
        if remaining:
            raise RuntimeError(
                "failed to remove non-drone agents: " + ", ".join(sorted(remaining))
            )
        self.removed_agent_names = tuple(targets)

    def _get_object_names(self) -> list[str]:
        api = self._require_api()
        with self._io_lock:
            objects = api.get_objects()
        if isinstance(objects, str):
            return objects.split()
        return [str(value) for value in objects or []]

    def _resolve_camera_id(self) -> int:
        api = self._require_api()
        with self._io_lock:
            camera_count = int(api.get_camera_num())
            if camera_count <= 0:
                raise RuntimeError("Unreal scene has no registered cameras")

            try:
                camera_names = [str(name) for name in list(api.get_camera_list() or [])]
            except Exception:
                camera_names = []

            drone_name_lower = self.drone_name.lower()
            for camera_id, camera_name in enumerate(camera_names):
                if camera_id >= camera_count:
                    break
                if drone_name_lower in camera_name.lower():
                    self.camera_id = camera_id
                    self.camera_name = camera_name
                    self.camera_id_source = "camera_name"
                    return camera_id

            try:
                drone_location = np.asarray(
                    api.get_obj_location(self.drone_name),
                    dtype=float,
                )[:3]
                nearest_camera_id: int | None = None
                nearest_distance: float | None = None
                for camera_id in range(camera_count):
                    try:
                        camera_location = api.get_cam_location(camera_id)
                    except Exception:
                        try:
                            api.register_camera(camera_id)
                            camera_location = api.get_cam_location(camera_id)
                        except Exception:
                            continue
                    distance = float(
                        np.linalg.norm(np.asarray(camera_location, dtype=float)[:3] - drone_location)
                    )
                    if nearest_distance is None or distance < nearest_distance:
                        nearest_camera_id = camera_id
                        nearest_distance = distance

                if nearest_camera_id is not None:
                    self.camera_id = nearest_camera_id
                    self.camera_name = (
                        camera_names[nearest_camera_id]
                        if nearest_camera_id < len(camera_names)
                        else ""
                    )
                    self.camera_id_source = "nearest_to_drone"
                    return nearest_camera_id
            except Exception:
                pass

            if 0 <= self.configured_camera_id < camera_count:
                self.camera_id = self.configured_camera_id
                self.camera_name = (
                    camera_names[self.camera_id]
                    if self.camera_id < len(camera_names)
                    else ""
                )
                self.camera_id_source = "setting_fallback"
                return self.camera_id

        raise RuntimeError(f"unable to resolve camera for {self.drone_name}")

    def _set_drone_scale(self) -> None:
        api = self._require_api()
        with self._io_lock:
            api.set_obj_scale(self.drone_name, list(self.drone_scale))

    def _set_drone_camera(self) -> None:
        camera_pose = (*self.camera_relative_location, *self.camera_relative_rotation)
        camera_pose_text = " ".join(str(value) for value in camera_pose)
        with self._io_lock:
            self._request(f"vbp {self.drone_name} set_cam {camera_pose_text}")

    @staticmethod
    def _load_setting(setting_file: Path) -> dict[str, Any]:
        if not setting_file.is_file():
            raise FileNotFoundError(f"environment setting not found: {setting_file}")
        with setting_file.open("r", encoding="utf-8") as file:
            setting = json.load(file)
        if not isinstance(setting, dict):
            raise RuntimeError(f"environment setting must be an object: {setting_file}")
        return setting

    @staticmethod
    def _load_first_drone_setting(
        setting: dict[str, Any],
        setting_file: Path,
    ) -> dict[str, Any]:
        drone = setting.get("agents", {}).get("drone")
        if not isinstance(drone, dict):
            raise RuntimeError(f"no drone configuration in {setting_file}")

        names = drone.get("name")
        camera_ids = drone.get("cam_id")
        if not isinstance(names, list) or not names:
            raise RuntimeError(f"drone.name is empty in {setting_file}")
        if not isinstance(camera_ids, list) or len(camera_ids) < len(names):
            raise RuntimeError(f"drone.cam_id does not match drone.name in {setting_file}")

        def vector(name: str, default: list[float]) -> tuple[float, float, float]:
            value = drone.get(name, default)
            if isinstance(value, (list, tuple)) and value and isinstance(value[0], (list, tuple)):
                value = value[0]
            if not isinstance(value, (list, tuple)) or len(value) != 3:
                raise RuntimeError(f"agents.drone.{name} must have three values in {setting_file}")
            return tuple(float(item) for item in value)

        return {
            "name": str(names[0]),
            "camera_id": int(camera_ids[0]),
            "scale": vector("scale", [1.0, 1.0, 1.0]),
            "relative_location": vector("relative_location", [0.0, 0.0, 0.0]),
            "relative_rotation": vector("relative_rotation", [0.0, 0.0, 0.0]),
            "config": drone,
        }

    @staticmethod
    def _load_other_agent_names(
        setting: dict[str, Any],
        controlled_drone_name: str,
    ) -> tuple[str, ...]:
        names: list[str] = []
        agents = setting.get("agents", {})
        if not isinstance(agents, dict):
            return ()
        for agent_config in agents.values():
            if not isinstance(agent_config, dict):
                continue
            configured_names = agent_config.get("name", [])
            if not isinstance(configured_names, list):
                continue
            for name in configured_names:
                value = str(name)
                if value != controlled_drone_name and value not in names:
                    names.append(value)
        return tuple(names)

    @classmethod
    def _resolve_initial_pose(
        cls,
        requested_pose: Iterable[float] | None,
        drone_setting: dict[str, Any],
        setting: dict[str, Any],
        setting_file: Path,
    ) -> tuple[tuple[float, float, float, float] | None, str]:
        if requested_pose is not None:
            return cls._coerce_pose(requested_pose, "initial_pose argument"), "argument"

        drone_config = drone_setting["config"]
        for key in ("initial_pose", "pose"):
            configured_pose = cls._first_vector(drone_config.get(key))
            if configured_pose is not None:
                return cls._coerce_pose(configured_pose, f"agents.drone.{key}"), f"agents.drone.{key}"

        configured_location = None
        location_source = ""
        for key in ("initial_location", "location"):
            configured_location = cls._first_vector(drone_config.get(key))
            if configured_location is not None:
                location_source = f"agents.drone.{key}"
                break
        if configured_location is not None:
            if len(configured_location) < 3:
                raise RuntimeError(f"{location_source} must contain x, y, z in {setting_file}")
            configured_yaw = cls._first_scalar(
                drone_config.get("initial_yaw", drone_config.get("yaw", 0.0))
            )
            return (
                float(configured_location[0]),
                float(configured_location[1]),
                float(configured_location[2]),
                float(configured_yaw),
            ), location_source

        safe_start = cls._first_vector(setting.get("safe_start"))
        if safe_start is not None:
            if len(safe_start) < 3:
                raise RuntimeError(f"safe_start must contain x, y, z in {setting_file}")
            return (
                float(safe_start[0]),
                float(safe_start[1]),
                float(safe_start[2]),
                0.0,
            ), "safe_start[0]"

        warnings.warn(
            f"no initial drone pose or safe_start found in {setting_file}; "
            "the drone will keep its Unreal scene position",
            RuntimeWarning,
            stacklevel=2,
        )
        return None, "unreal_scene_default"

    @staticmethod
    def _coerce_pose(values: Iterable[float], source: str) -> tuple[float, float, float, float]:
        pose = tuple(float(value) for value in values)
        if len(pose) == 4:
            return pose
        if len(pose) >= 6:
            return pose[0], pose[1], pose[2], pose[4]
        raise ValueError(f"{source} must be (x, y, z, yaw) or a 6-value Unreal pose")

    @staticmethod
    def _first_vector(value: Any) -> list[Any] | tuple[Any, ...] | None:
        if not isinstance(value, (list, tuple)) or not value:
            return None
        first = value[0]
        if isinstance(first, (list, tuple)):
            return first
        return value

    @staticmethod
    def _first_scalar(value: Any) -> float:
        while isinstance(value, (list, tuple)):
            if not value:
                return 0.0
            value = value[0]
        return float(value)

    def _set_world_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        api = self._require_api()
        with self._io_lock:
            api.set_obj_location(self.drone_name, [float(x), float(y), float(z)])
            self._request(f"vbp {self.drone_name} set_rotation {float(yaw)}")

    def _set_velocity_raw(self, x: float, y: float, z: float, yaw: float) -> None:
        params = " ".join(str(float(value)) for value in (x, y, z, yaw))
        with self._io_lock:
            self._request(f"vbp {self.drone_name} set_move {params}")

    def _request(self, command: str, retries: int = 100) -> Any:
        api = self._require_api()
        for _ in range(max(1, int(retries))):
            response = api.client.request(command, -1)
            if response is not None:
                return response
            time.sleep(0.01)
        raise RuntimeError(f"UnrealCV request failed: {command}")

    def _require_api(self) -> UnrealCv_API:
        with self._state_lock:
            api = self._api
        if api is None:
            raise RuntimeError("simulator is not connected, call connect first")
        return api

    def _require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError("simulator is not connected, call connect first")

    def _start_watchdog(self) -> None:
        if self.command_timeout_s <= 0:
            return
        self._watchdog_stop.clear()
        thread = threading.Thread(
            target=self._watchdog_loop,
            daemon=True,
            name="ue-drone-command-watchdog",
        )
        self._watchdog_thread = thread
        thread.start()

    def _stop_watchdog(self) -> None:
        self._watchdog_stop.set()
        thread = self._watchdog_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._watchdog_thread = None

    def _watchdog_loop(self) -> None:
        interval = min(0.1, max(0.02, self.command_timeout_s / 4.0))
        while not self._watchdog_stop.wait(interval):
            with self._state_lock:
                should_stop = (
                    self._connected
                    and self._velocity_active
                    and time.monotonic() - self._last_velocity_at >= self.command_timeout_s
                )
            if should_stop:
                try:
                    self.stop()
                except Exception:
                    pass

    @staticmethod
    def _clamp_percent(value: float) -> float:
        return max(-100.0, min(100.0, float(value)))

    @staticmethod
    def _normalize_yaw(value: float) -> float:
        return (float(value) + 180.0) % 360.0 - 180.0

    def __enter__(self) -> "SimDrone":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
