"""Local client for the owl_ego v21 contract in docs/owl_ego_contract.md.

No ROS or flight-control implementation belongs in this module.
"""
from __future__ import annotations

import base64
from dataclasses import dataclass
import math
import threading
import time
from urllib.parse import quote
import uuid

import cv2
import numpy as np

from .base import BaseClient
from .owl import _Da3DepthServiceClient


@dataclass(frozen=True)
class Observation:
    frame_id: str
    timestamp_s: float
    received_monotonic: float
    age_s: float
    pose: dict
    rgb: np.ndarray
    intrinsics: np.ndarray
    world_from_camera: np.ndarray
    localization_epoch: str
    world_frame: str


def decode_observation(data, *, max_age_s=0.5, max_sync_error_s=0.05):
    """K applies to rectified RGB; transform is optical camera -> ROS ENU, cm."""
    if data.get("ok") is not True or data.get("rectified") is not True:
        raise ValueError("Observation must be successful and rectified")
    for key in ("frame_id", "localization_epoch", "world_frame"):
        if not isinstance(data.get(key), str) or not data[key]:
            raise ValueError(f"Missing observation {key}")
    values = [float(data[k]) for k in ("timestamp_s", "age_s", "sync_error_s")]
    if not all(math.isfinite(v) and v >= 0 for v in values):
        raise ValueError("Invalid observation timing")
    stamp, age, sync = values
    if age > max_age_s or sync > max_sync_error_s:
        raise ValueError(f"Stale/unsynchronized observation: age={age}, sync={sync}")
    pose = {k: float(data["pose"][k]) for k in ("x", "y", "z", "yaw")}
    if not all(math.isfinite(v) for v in pose.values()):
        raise ValueError("Non-finite capture pose")
    encoded = base64.b64decode(data["rgb_jpeg_base64"], validate=True)
    bgr = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None or list(bgr.shape[:2][::-1]) != data["image_size"]:
        raise ValueError("Invalid observation JPEG/image_size")
    k = np.asarray(data["intrinsics"], dtype=float)
    t = np.asarray(data["world_from_camera_optical_cm"], dtype=float)
    if (k.shape != (3, 3) or not np.isfinite(k).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1]) or abs(np.linalg.det(k)) < 1e-9):
        raise ValueError("Invalid RGB camera intrinsics")
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1])
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-4)):
        raise ValueError("Invalid optical-camera-to-ENU rigid transform")
    return Observation(data["frame_id"], stamp, time.monotonic(), age, pose,
                       cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), k, t,
                       data["localization_epoch"], data["world_frame"])


class OwlEgoClient(BaseClient):
    SUPPORTS_MOTION_TIMEOUT = True

    def __init__(self, *args, observation_max_age_s=0.5,
                 observation_max_sync_error_s=0.05, **kwargs):
        super().__init__(*args, **kwargs)
        self.depth_service = _Da3DepthServiceClient(timeout_s=self.timeout_s)
        self.observation_max_age_s = observation_max_age_s
        self.observation_max_sync_error_s = observation_max_sync_error_s
        self.last_observation = None
        self._frame_identity = None
        self._image_shape = None
        self.session_id = None
        self._lease_stop = threading.Event()
        self._lease_thread = None
        self._lease_error = None

    def rpc(self, method, path, payload=None, *, timeout_s=3.0):
        result = self._request_json(method, path, payload, timeout_s=timeout_s)
        if result.get("ok") is not True:
            raise RuntimeError(f"{path}: {result.get('error', result)}")
        return result

    def check_lease(self):
        if self._lease_error is not None:
            raise RuntimeError(f"Robot control lease failed: {self._lease_error}")

    def _heartbeat_loop(self):
        while not self._lease_stop.wait(0.5):
            try:
                self.rpc("POST", "/v21/heartbeat", {"session_id": self.session_id},
                         timeout_s=2.0)
            except Exception as exc:
                self._lease_error = exc
                return

    def start(self):
        self.depth_service.health()
        caps = self.rpc("GET", "/v21/capabilities")
        if caps.get("backend") != "owl_ego" or caps.get("protocol_version") != 1:
            raise RuntimeError("Robot must implement owl_ego protocol_version=1")
        for key in ("async_navigation", "cancel_and_hold", "synchronized_observation",
                    "control_lease", "relative_xyz_yaw"):
            if caps.get(key) is not True:
                raise RuntimeError(f"Missing Robot capability: {key}")
        # Validate calibration and metric depth before acquiring flight control.
        obs = self.observe()
        self.estimate_depth(obs)
        result = self.rpc("POST", "/v21/session", {"request_id": str(uuid.uuid4())})
        self.session_id = result["session_id"]
        if not isinstance(self.session_id, str) or not self.session_id:
            raise RuntimeError("Invalid session_id")
        self._lease_thread = threading.Thread(target=self._heartbeat_loop,
                                              name="owl-ego-heartbeat", daemon=True)
        self._lease_thread.start()
        return super().start()

    def _request_json(self, method, path, payload=None, **kwargs):
        # Existing init/takeoff/relative-motion/land calls carry the same lease.
        if method == "POST" and self.session_id and not path.startswith("/v21/"):
            payload = {**(payload or {}), "session_id": self.session_id,
                       "request_id": str(uuid.uuid4())}
        return super()._request_json(method, path, payload, **kwargs)

    def observe(self):
        self.check_lease()
        started = time.monotonic()
        obs = decode_observation(self.rpc("GET", "/v21/observation"),
                                 max_age_s=self.observation_max_age_s,
                                 max_sync_error_s=self.observation_max_sync_error_s)
        if obs.age_s + time.monotonic() - started > self.observation_max_age_s:
            raise RuntimeError("Observation is stale after network transfer")
        identity = (obs.world_frame, obs.localization_epoch)
        if self._frame_identity is None:
            self._frame_identity = identity
        elif identity != self._frame_identity:
            raise RuntimeError("Localization frame/epoch changed; mission must abort")
        if self._image_shape is None:
            self._image_shape = obs.rgb.shape
        elif self._image_shape != obs.rgb.shape:
            raise RuntimeError("RGB resolution changed during mission")
        return obs

    def estimate_depth(self, observation):
        return self.depth_service.estimate_depth_cm(observation.rgb)

    def capture(self, include_depth=True, raw=True):
        obs = self.observe()
        self.last_observation = obs
        return (obs.rgb, self.estimate_depth(obs)) if include_depth else obs.rgb

    def get_pose(self):
        self.check_lease()
        result = self.rpc("GET", "/get_pose")
        if self._frame_identity and result.get("localization_epoch") != self._frame_identity[1]:
            raise RuntimeError("Localization epoch changed in pose response")
        pose = {k: float(result["pose"][k]) for k in ("x", "y", "z", "yaw")}
        if not all(math.isfinite(v) for v in pose.values()):
            raise RuntimeError("Invalid Robot pose")
        return pose

    def navigate(self, pose):
        self.check_lease()
        return self.rpc("POST", "/v21/navigation", {
            "session_id": self.session_id, "request_id": str(uuid.uuid4()),
            "localization_epoch": self._frame_identity[1], "pose": dict(pose),
        })["task_id"]

    def navigation_status(self, task_id):
        self.check_lease()
        result = self.rpc("GET", "/v21/navigation/status?task_id=" + quote(task_id, safe=""))
        if result.get("task_id") != task_id:
            raise RuntimeError("Mismatched navigation task_id")
        if result.get("status") not in {"accepted", "planning", "executing", "stopping",
                                        "arrived", "cancelled", "failed"}:
            raise RuntimeError("Invalid navigation status")
        return result

    def cancel_navigation(self, task_id):
        return self.rpc("POST", "/v21/navigation/cancel", {
            "session_id": self.session_id, "task_id": task_id,
            "request_id": str(uuid.uuid4()),
        })

    def close(self):
        # Release must hold rather than disarm. Lease expiry covers network loss.
        try:
            if self.session_id:
                return self.rpc("POST", "/v21/session/release",
                                {"session_id": self.session_id})
        finally:
            self._lease_stop.set()
            if self._lease_thread:
                self._lease_thread.join(timeout=3)
