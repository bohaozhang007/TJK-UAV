from __future__ import annotations

import io
import json
import math
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np


JsonObject = Dict[str, Any]


class BaseClient:
    """Agent-facing client for the unified Robot Server HTTP API.

    ``capture(raw=True)`` transfers RGB and depth through HTTP byte streams.
    ``capture(raw=False)`` asks the Robot Server to save files and then reads
    the paths returned by the metadata endpoints. File mode therefore requires
    the client and server to share a filesystem.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8765,
        timeout_s: float = 180.0,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self.base_url = f"http://{self.host}:{self.port}"
        self._opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({})
        )
        self._motion_tolerances: Optional[JsonObject] = None

    def _request(
        self,
        method: str,
        path: str,
        payload: Optional[JsonObject] = None,
        *,
        accept: str = "application/json",
        timeout_s: Optional[float] = None,
    ) -> Tuple[bytes, str]:
        body = None
        headers = {"Accept": accept}
        if payload is not None:
            body = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json; charset=utf-8"

        request = urllib.request.Request(
            f"{self.base_url}{path}",
            data=body,
            headers=headers,
            method=method,
        )
        timeout = self.timeout_s if timeout_s is None else float(timeout_s)
        try:
            with self._opener.open(request, timeout=timeout) as response:
                return response.read(), response.headers.get_content_type()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"HTTP {exc.code} from {path}: {detail}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(
                f"Cannot connect to Robot Server at {self.base_url}: {exc}"
            ) from exc

    def _request_json(
        self,
        method: str,
        path: str,
        payload: Optional[JsonObject] = None,
        *,
        timeout_s: Optional[float] = None,
    ) -> JsonObject:
        raw, content_type = self._request(
            method,
            path,
            payload,
            accept="application/json",
            timeout_s=timeout_s,
        )
        try:
            result = json.loads(raw.decode("utf-8")) if raw else {}
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError(
                f"Expected JSON from {path}, got {content_type}"
            ) from exc
        if not isinstance(result, dict):
            raise RuntimeError(f"Expected a JSON object from {path}")
        return result

    @staticmethod
    def _require_ok(result: JsonObject, operation: str) -> JsonObject:
        if result.get("ok", True) is False:
            detail = result.get("error") or result.get("message") or result
            raise RuntimeError(f"{operation} failed: {detail}")
        return result

    def health(self) -> JsonObject:
        return self._request_json(
            "GET",
            "/health",
            timeout_s=min(self.timeout_s, 2.0),
        )

    def get_motion_tolerances(self, *, refresh: bool = False) -> JsonObject:
        """Return and cache the Robot Server's motion-completion contract."""
        if self._motion_tolerances is not None and not refresh:
            return dict(self._motion_tolerances)

        result = self._require_ok(
            self._request_json(
                "GET",
                "/motion_tolerances",
                timeout_s=min(self.timeout_s, 2.0),
            ),
            "motion_tolerances",
        )
        tolerances = result.get("motion_tolerances")
        if not isinstance(tolerances, dict):
            raise RuntimeError(
                "motion_tolerances failed: response does not contain a mapping"
            )

        parsed = {}
        for key in ("position_tolerance_cm", "yaw_tolerance_deg"):
            value = tolerances.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise RuntimeError(
                    f"motion_tolerances failed: {key} must be numeric"
                )
            value = float(value)
            if not math.isfinite(value) or value < 0.0:
                raise RuntimeError(
                    f"motion_tolerances failed: {key} must be finite and >= 0"
                )
            parsed[key] = value

        metric = tolerances.get("position_error_metric")
        if metric != "euclidean_3d":
            raise RuntimeError(
                "motion_tolerances failed: position_error_metric must be "
                f"'euclidean_3d', got {metric!r}"
            )
        parsed["position_error_metric"] = metric
        parsed["source"] = str(tolerances.get("source", "unknown"))
        self._motion_tolerances = parsed
        return dict(parsed)

    @staticmethod
    def quantize_motion(
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        dyaw: float = 0.0,
    ) -> JsonObject:
        """Return the integer centimetres/degrees sent by current clients."""
        return {
            "x": int(round(dx)),
            "y": int(round(dy)),
            "z": int(round(dz)),
            "yaw": int(round(dyaw)),
        }

    def _health_state(self) -> JsonObject:
        result = self._require_ok(self.health(), "health")
        health = result.get("health")
        if not isinstance(health, dict):
            raise RuntimeError("health failed: response does not contain health data")
        return health

    def _init(self) -> JsonObject:
        return self._require_ok(
            self._request_json("POST", "/init", {}),
            "init",
        )

    def _takeoff(self) -> JsonObject:
        return self._require_ok(
            self._request_json("POST", "/takeoff", {}),
            "takeoff",
        )

    def start(self) -> JsonObject:
        """Initialize and take off only when the current health requires it."""
        init_result = None
        takeoff_result = None
        motion_tolerances = self.get_motion_tolerances(refresh=True)

        health_state = self._health_state()
        if health_state.get("initialized") is not True:
            init_result = self._init()
            health_state = self._health_state()

        if health_state.get("initialized") is not True:
            raise RuntimeError("health failed: robot is not initialized")

        if health_state.get("airborne") is not True:
            takeoff_result = self._takeoff()

        health_state = self._health_state()
        if health_state.get("initialized") is not True:
            raise RuntimeError("health failed: robot is not initialized after start")
        if health_state.get("airborne") is not True:
            raise RuntimeError("health failed: robot is not airborne after start")

        return {
            "ok": True,
            "init": init_result,
            "health": health_state,
            "motion_tolerances": motion_tolerances,
            "takeoff": takeoff_result,
        }

    def capture(
        self,
        include_depth: bool = True,
        raw: bool = True,
    ) -> np.ndarray | Tuple[np.ndarray, np.ndarray]:
        """Return RGB, or ``(RGB, depth)`` when ``include_depth`` is true.

        RGB uses ``uint8`` in RGB channel order. Depth dtype and centimetre
        values are preserved exactly as provided by the Robot Server.
        """
        if raw:
            rgb = self._capture_rgb_bytes()
            if include_depth:
                return rgb, self._capture_depth_bytes()
            return rgb

        rgb = self._capture_rgb_file()
        if include_depth:
            return rgb, self._capture_depth_file()
        return rgb

    def _capture_rgb_bytes(self) -> np.ndarray:
        raw, content_type = self._request(
            "GET",
            "/get_rgb_byte",
            accept="image/jpeg",
        )
        if content_type == "application/json":
            self._raise_json_failure(raw, "get_rgb_byte")

        frame_bgr = cv2.imdecode(
            np.frombuffer(raw, dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        if frame_bgr is None:
            raise RuntimeError(
                f"Failed to decode RGB response with content type {content_type}"
            )
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    def _capture_depth_bytes(self) -> np.ndarray:
        raw, content_type = self._request(
            "GET",
            "/get_depth_np",
            accept="application/x-npy",
        )
        if content_type == "application/json":
            self._raise_json_failure(raw, "get_depth_np")
        try:
            depth = np.load(io.BytesIO(raw), allow_pickle=False)
        except Exception as exc:
            raise RuntimeError(
                f"Failed to decode depth response with content type {content_type}"
            ) from exc
        if not isinstance(depth, np.ndarray):
            raise RuntimeError("get_depth_np did not return a NumPy array")
        return depth

    def _capture_rgb_file(self) -> np.ndarray:
        result = self._require_ok(
            self._request_json("POST", "/get_rgb_meta", {"save": True}),
            "get_rgb_meta",
        )
        path = self._result_path(result, "get_rgb_meta")
        frame_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if frame_bgr is None:
            raise RuntimeError(f"Failed to read RGB file: {path}")
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    def _capture_depth_file(self) -> np.ndarray:
        result = self._require_ok(
            self._request_json("POST", "/get_depth_meta", {"save": True}),
            "get_depth_meta",
        )
        path = self._result_path(result, "get_depth_meta")
        try:
            depth = np.load(path, allow_pickle=False)
        except Exception as exc:
            raise RuntimeError(f"Failed to read depth file: {path}") from exc
        if not isinstance(depth, np.ndarray):
            raise RuntimeError(f"Depth file does not contain a NumPy array: {path}")
        return depth

    @staticmethod
    def _result_path(result: JsonObject, operation: str) -> Path:
        value = result.get("saved_to") or result.get("file_path") or result.get("path")
        if not value:
            raise RuntimeError(f"{operation} did not return a saved file path")
        path = Path(str(value))
        if not path.is_file():
            raise RuntimeError(
                f"{operation} returned an inaccessible file path: {path}"
            )
        return path

    @staticmethod
    def _raise_json_failure(raw: bytes, operation: str) -> None:
        try:
            result = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"{operation} returned invalid JSON") from exc
        if isinstance(result, dict):
            detail = result.get("error") or result.get("message") or result
        else:
            detail = result
        raise RuntimeError(f"{operation} failed: {detail}")

    def move_relative(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        dyaw: float = 0.0,
    ) -> JsonObject:
        """Combine relative XYZ translation and yaw rotation."""
        command = self.quantize_motion(dx, dy, dz, dyaw)
        x, y, z = command["x"], command["y"], command["z"]
        angle_deg = command["yaw"]

        translation = None
        rotation = None
        if any((x, y, z)):
            translation = self._require_ok(
                self._request_json(
                    "POST",
                    "/move_relative_xyz",
                    {"x": x, "y": y, "z": z},
                ),
                "move_relative_xyz",
            )
        if angle_deg:
            rotation = self._require_ok(
                self._request_json(
                    "POST",
                    "/rotate",
                    {"angle_deg": angle_deg},
                ),
                "rotate",
            )

        return {
            "ok": True,
            "translation": translation,
            "rotation": rotation,
        }

    def move_rel_xyz_yaw(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw: float = 0.0,
        timeout_s: float | None = None,
    ) -> JsonObject:
        """Send one combined relative XYZ-and-yaw motion command."""
        command = self.quantize_motion(x, y, z, yaw)
        if not any(command.values()):
            return {
                "ok": True,
                "message": "move_relative_xyz_yaw skipped",
                "command": command,
            }
        payload = dict(command)
        if timeout_s is not None:
            timeout_s = float(timeout_s)
            if not math.isfinite(timeout_s) or timeout_s <= 0.0:
                raise ValueError(
                    f"motion timeout must be finite and positive, got {timeout_s!r}"
                )
            payload["timeout_s"] = timeout_s
        return self._require_ok(
            self._request_json(
                "POST",
                "/move_relative_xyz_yaw",
                payload,
            ),
            "move_relative_xyz_yaw",
        )

    def get_pose(self) -> JsonObject:
        """Return x/y/z in centimetres and yaw normalized to [-180, 180)."""
        result = self._require_ok(
            self._request_json("GET", "/get_pose"),
            "get_pose",
        )
        pose = result.get("pose", result)
        if not isinstance(pose, dict):
            raise RuntimeError("get_pose response does not contain a pose object")

        missing = [name for name in ("x", "y", "z", "yaw") if name not in pose]
        if missing:
            raise RuntimeError(f"get_pose response is missing: {', '.join(missing)}")
        try:
            x = float(pose["x"])
            y = float(pose["y"])
            z = float(pose["z"])
            yaw = (float(pose["yaw"]) + 180.0) % 360.0 - 180.0
        except (TypeError, ValueError) as exc:
            raise RuntimeError("get_pose returned non-numeric pose values") from exc
        return {"x": x, "y": y, "z": z, "yaw": yaw}

    def land(self) -> JsonObject:
        return self._request_json("POST", "/land", {})

    def close(self) -> JsonObject:
        return self._request_json("POST", "/close", {})
