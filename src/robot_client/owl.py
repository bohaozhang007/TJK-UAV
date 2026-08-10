"""Agent-facing client for the VisBot OWL mini3L Robot Server."""

from __future__ import annotations

import io
import json
import urllib.error
import urllib.request
from typing import Tuple

import cv2
import numpy as np

from .base import BaseClient, JsonObject


class _Da3DepthServiceClient:
    """Small HTTP client for the local persistent DA3 inference service."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8770,
        timeout_s: float = 180.0,
    ) -> None:
        self.base_url = f"http://{host}:{int(port)}"
        self.timeout_s = float(timeout_s)
        self._opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({})
        )

    def health(self) -> JsonObject:
        request = urllib.request.Request(
            f"{self.base_url}/health",
            headers={"Accept": "application/json"},
            method="GET",
        )
        raw, content_type, _headers = self._open(request, timeout_s=2.0)
        try:
            result = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError(
                "Expected JSON from DA3 /health, "
                f"got {content_type}"
            ) from exc
        if not isinstance(result, dict):
            raise RuntimeError("DA3 /health did not return a JSON object")
        if result.get("ok") is not True or result.get("ready") is not True:
            detail = result.get("error") or result.get("message") or result
            raise RuntimeError(f"DA3 service is not ready: {detail}")
        if result.get("metric") is not True:
            raise RuntimeError("DA3 service is not configured for metric depth")
        if result.get("depth_unit") != "cm":
            raise RuntimeError(
                "DA3 service depth unit must be cm, got "
                f"{result.get('depth_unit')!r}"
            )
        return result

    def estimate_depth_cm(self, frame_rgb: np.ndarray) -> np.ndarray:
        frame_rgb = np.asarray(frame_rgb)
        if (
            frame_rgb.ndim != 3
            or frame_rgb.shape[2] != 3
            or frame_rgb.dtype != np.uint8
        ):
            raise ValueError("DA3 input RGB must be a uint8 HxWx3 array")

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        success, encoded = cv2.imencode(
            ".jpg",
            frame_bgr,
            [cv2.IMWRITE_JPEG_QUALITY, 95],
        )
        if not success:
            raise RuntimeError("Failed to encode RGB frame for DA3 service")

        request = urllib.request.Request(
            f"{self.base_url}/estimate",
            data=encoded.tobytes(),
            headers={
                "Accept": "application/x-npy",
                "Content-Type": "image/jpeg",
            },
            method="POST",
        )
        raw, content_type, headers = self._open(request)
        if content_type == "application/json":
            self._raise_json_failure(raw, "DA3 estimate")
        if content_type != "application/x-npy":
            raise RuntimeError(
                "Expected application/x-npy from DA3 /estimate, "
                f"got {content_type}"
            )
        if headers.get("X-Depth-Unit") != "cm":
            raise RuntimeError(
                "DA3 /estimate response is missing X-Depth-Unit: cm"
            )

        try:
            depth = np.load(io.BytesIO(raw), allow_pickle=False)
        except Exception as exc:
            raise RuntimeError("Failed to decode DA3 depth response") from exc
        if not isinstance(depth, np.ndarray) or depth.ndim != 2:
            raise RuntimeError(
                "DA3 depth must be a 2D NumPy array, got "
                f"{getattr(depth, 'shape', None)}"
            )
        if depth.shape != frame_rgb.shape[:2]:
            raise RuntimeError(
                f"DA3 depth shape={depth.shape} does not match "
                f"RGB shape={frame_rgb.shape[:2]}"
            )
        depth = np.asarray(depth, dtype=np.float32)
        valid = np.isfinite(depth) & (depth > 0)
        if not np.any(valid):
            raise RuntimeError("DA3 returned no valid positive depth values")
        return depth

    def _open(self, request, *, timeout_s=None):
        timeout = self.timeout_s if timeout_s is None else float(timeout_s)
        try:
            with self._opener.open(request, timeout=timeout) as response:
                return (
                    response.read(),
                    response.headers.get_content_type(),
                    response.headers,
                )
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(
                f"HTTP {exc.code} from DA3 service: {detail}"
            ) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(
                f"Cannot connect to DA3 service at {self.base_url}: {exc}"
            ) from exc

    @staticmethod
    def _raise_json_failure(raw: bytes, operation: str) -> None:
        try:
            result = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"{operation} returned invalid JSON") from exc
        detail = (
            result.get("error") or result.get("message") or result
            if isinstance(result, dict)
            else result
        )
        raise RuntimeError(f"{operation} failed: {detail}")


class OwlClient(BaseClient):
    """OWL HTTP client with depth from a local persistent DA3 service.

    The Robot Server provides RGB, pose, health, and motion. Metric depth is
    estimated on the Agent computer and never requested from the drone.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8765,
        timeout_s: float = 180.0,
        *,
        depth_service_host: str = "127.0.0.1",
        depth_service_port: int = 8770,
    ) -> None:
        super().__init__(
            host=host,
            port=port,
            timeout_s=timeout_s,
        )
        self._depth_service = _Da3DepthServiceClient(
            host=depth_service_host,
            port=depth_service_port,
            timeout_s=timeout_s,
        )

    def start(self) -> JsonObject:
        # Fail before any Robot Server takeoff fallback if DA3 is unavailable.
        depth_health = self._depth_service.health()
        result = super().start()
        health = self._health_state()
        missing = [
            name
            for name in (
                "odom_ok",
                "rgb_ok",
                "yaw_link_ok",
                "control_ready",
            )
            if health.get(name) is not True
        ]
        if missing:
            raise RuntimeError(
                "OWL is not ready for Agent control: " + ", ".join(missing)
            )

        # The Robot Server is now initialized and RGB is known to be fresh.
        # Run one real camera frame through DA3 before any Agent motion.
        frame_rgb, depth_cm = self.capture(include_depth=True)
        result["health"] = health
        result["depth_service"] = depth_health
        result["depth_warmup"] = {
            "rgb_shape": list(frame_rgb.shape),
            "depth_shape": list(depth_cm.shape),
            "depth_dtype": str(depth_cm.dtype),
            "depth_unit": "cm",
            "valid_ratio": float(
                np.mean(np.isfinite(depth_cm) & (depth_cm > 0))
            ),
        }
        return result

    def capture(
        self,
        include_depth: bool = True,
        raw: bool = True,
    ) -> np.ndarray | Tuple[np.ndarray, np.ndarray]:
        frame_rgb = super().capture(include_depth=False, raw=raw)
        if not isinstance(frame_rgb, np.ndarray):
            raise RuntimeError("Robot Server did not return an RGB array")
        if not include_depth:
            return frame_rgb
        depth_cm = self._depth_service.estimate_depth_cm(frame_rgb)
        return frame_rgb, depth_cm

    def move_relative(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        dyaw: float = 0.0,
    ) -> JsonObject:
        command = self.quantize_motion(dx, dy, dz, dyaw)
        x, y, z = command["x"], command["y"], command["z"]
        yaw = command["yaw"]
        if not any((x, y, z, yaw)):
            return {
                "ok": True,
                "message": "move_relative_xyz_yaw skipped",
                "command": {"x": x, "y": y, "z": z, "yaw": yaw},
            }
        return self._require_ok(
            self._request_json(
                "POST",
                "/move_relative_xyz_yaw",
                {"x": x, "y": y, "z": z, "yaw": yaw},
            ),
            "move_relative_xyz_yaw",
        )


__all__ = ["OwlClient"]
