"""Local client for the owl_ego v21 contract in docs/owl_ego_contract.md.

No ROS or flight-control implementation belongs in this module.
"""
from __future__ import annotations

import base64
from dataclasses import dataclass, field
import math
import http.client
import threading
import time
from urllib.parse import quote
import urllib.error
import uuid

import cv2
import numpy as np

from .base import BaseClient, RobotHTTPError
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
    rectified: bool = True
    calibration_quality: str = "calibrated"
    geometry_assumptions: dict = field(default_factory=dict)


def decode_observation(data, *, max_age_s=0.5, max_sync_error_s=0.05,
                       allow_approximate_geometry=False):
    """K applies to rectified RGB; transform is optical camera -> ROS ENU, cm."""
    quality = data.get("calibration_quality")
    geometry = data.get("geometry_assumptions", {})
    rectified = data.get("rectified")
    if data.get("ok") is not True or type(rectified) is not bool:
        raise ValueError("Invalid observation success/rectification flag")
    if quality == "calibrated":
        if not rectified:
            raise ValueError("Calibrated observation must be rectified")
    elif quality == "approximate" and allow_approximate_geometry:
        if not isinstance(geometry, dict):
            raise ValueError("Missing geometry assumptions")
        if (geometry.get("extrinsics") != "body_coincident_fixed"
                or geometry.get("camera_translation") != "body_coincident_assumption"):
            raise ValueError("Unsupported approximate extrinsics")
        mode = geometry.get("intrinsics")
        if mode == "approximate_fov":
            fov = geometry.get("assumed_horizontal_fov_deg")
            if (isinstance(fov, bool) or not isinstance(fov, (int, float))
                    or not math.isfinite(fov) or not 1 < fov < 179
                    or geometry.get("principal_point") != "image_center"
                    or geometry.get("square_pixels_assumed") is not True
                    or geometry.get("distortion") != "unknown_not_corrected" or rectified):
                raise ValueError("Invalid approximate FOV profile")
        elif mode != "camera_info" or not rectified or geometry.get("distortion") != "corrected":
            raise ValueError("Unsupported approximate intrinsics")
    else:
        raise ValueError("Geometry quality requires calibrated rectified data or explicit approximate opt-in")
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
    if quality == "approximate":
        if not np.allclose(t[:3, 3], [pose["x"], -pose["y"], pose["z"]], atol=1e-3):
            raise ValueError("Camera translation contradicts body-coincident profile")
        if geometry["intrinsics"] == "approximate_fov":
            w, h = data["image_size"]
            f = w / (2 * math.tan(math.radians(geometry["assumed_horizontal_fov_deg"]) / 2))
            if not np.allclose(k, [[f, 0, w/2], [0, f, h/2], [0, 0, 1]], atol=1e-4):
                raise ValueError("Intrinsics contradict approximate FOV profile")
    return Observation(data["frame_id"], stamp, time.monotonic(), age, pose,
                       cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), k, t,
                       data["localization_epoch"], data["world_frame"], rectified,
                       quality, dict(geometry))


class OwlEgoClient(BaseClient):
    SUPPORTS_MOTION_TIMEOUT = True

    def __init__(self, *args, observation_max_age_s=0.5,
                 observation_max_sync_error_s=0.05, allow_approximate_geometry=False,
                 auto_arm=False, stop_timeout_s=8.0, observation_retry_s=0.5, **kwargs):
        super().__init__(*args, **kwargs)
        if type(allow_approximate_geometry) is not bool or type(auto_arm) is not bool:
            raise ValueError("Geometry opt-in and auto_arm must be booleans")
        for value in (stop_timeout_s, observation_retry_s):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError("Wait budgets must be finite and positive")
        self.allow_approximate_geometry = allow_approximate_geometry
        self.auto_arm = auto_arm
        self.stop_timeout_s = stop_timeout_s
        self.observation_retry_s = observation_retry_s
        self._rgb_unavailable_since = None
        self._starting_task = None
        self._starting_since = None
        self.event = lambda *args, **kwargs: None
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
        self._lease_cv = threading.Condition()
        self._lease_sent = None
        self._lease_recovering = False
        self._lease_failures = 0
        self._recovery_version = 0
        self._checked_recovery_version = 0
        self._last_task_id = None
        self._motion_failure = None
        self._observation_ready = threading.Event()
        self._observation_ready.set()
        self._observation_error = None

    def rpc(self, method, path, payload=None, *, timeout_s=3.0):
        result = self._request_json(method, path, payload, timeout_s=timeout_s)
        if result.get("ok") is not True:
            raise RuntimeError(f"{path}: {result.get('error', result)}")
        return result

    def check_lease(self):
        with self._lease_cv:
            while True:
                if self._lease_error is not None:
                    raise RuntimeError(f"Robot control lease failed: {self._lease_error}")
                if self._lease_sent is not None and time.monotonic() >= self._lease_sent + 5:
                    self._fail_lease("local 5 s lease budget exhausted")
                    continue
                if not self._lease_recovering:
                    return
                self._lease_cv.wait(timeout=0.05)

    @staticmethod
    def _transport_failure(exc):
        """Only network failures, never HTTP rejection or JSON/program errors."""
        while exc is not None:
            if isinstance(exc, (RobotHTTPError, urllib.error.HTTPError)):
                return False
            if isinstance(exc, (TimeoutError, ConnectionError, http.client.RemoteDisconnected)):
                return True
            if isinstance(exc, urllib.error.URLError):
                return isinstance(exc.reason, (TimeoutError, ConnectionError))
            exc = exc.__cause__
        return False

    def _fail_lease(self, reason):
        with self._lease_cv:
            if self._lease_error is None:
                self._lease_error = RuntimeError(str(reason))
                self.event("lease_failed", reason=str(reason), failures=self._lease_failures)
            self._lease_cv.notify_all()

    def _begin_recovery(self):
        with self._lease_cv:
            self._lease_recovering = True
            self._lease_cv.notify_all()

    def _renew_lease(self):
        """One attempt, bounded by the LAST ACKNOWLEDGED request's send time."""
        with self._lease_cv:
            if self._lease_error is not None or self._lease_sent is None:
                return False
            sent = time.monotonic()
            deadline = self._lease_sent + 5.0
            if sent >= deadline:
                self._fail_lease("local 5 s lease budget exhausted")
                return False
        error = None
        try:
            self.rpc("POST", "/v21/heartbeat", {"session_id": self.session_id},
                     timeout_s=min(2.0, deadline-sent))
        except Exception as exc:
            error = exc
        ended = time.monotonic()
        with self._lease_cv:
            self.event("lease_attempt", sent_monotonic=sent, elapsed_s=ended-sent,
                       remaining_s=max(0., deadline-ended), success=error is None,
                       consecutive_failures=self._lease_failures + (error is not None))
            if self._lease_error is not None:
                return False  # Never clear an independently latched failure.
            if ended >= deadline:
                self._fail_lease("lease recovery deadline exceeded; late reply cannot revive session")
                return False
            if error is not None:
                self._lease_failures += 1
                if not self._transport_failure(error):
                    self._fail_lease(error)
                    return False
                self._begin_recovery()
                self.event("lease_retry", reason=str(error), remaining_s=deadline-ended)
                return True
            self._lease_sent = sent  # NOT response time; failures never update this.
            if self._lease_recovering:
                self._recovery_version += 1
                self.event("lease_recovered", failures=self._lease_failures,
                           remaining_s=max(0., sent+5-ended))
            self._lease_recovering = False
            self._lease_failures = 0
            self._lease_cv.notify_all()
            return True

    def _heartbeat_loop(self):
        while not self._lease_stop.is_set():
            with self._lease_cv:
                remaining = self._lease_sent + 5 - time.monotonic()
                self._lease_cv.wait(timeout=max(0., min(
                    0.1 if self._lease_recovering else 0.5, remaining)))
            if self._lease_stop.is_set() or not self._renew_lease():
                return

    def start(self):
        if self.session_id:
            raise RuntimeError("Close the existing session before starting a fresh mission")
        self._frame_identity = self._image_shape = self.last_observation = None
        self._lease_stop.clear()
        self._lease_error = None
        self._lease_sent = None
        self._lease_recovering = False
        self._lease_failures = self._recovery_version = self._checked_recovery_version = 0
        self._last_task_id = self._motion_failure = None
        self._observation_error = None
        self._observation_ready.set()
        self._rgb_unavailable_since = self._starting_task = self._starting_since = None
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 90:
            raise ValueError("Transport timeout must exceed the 90 s landing budget")
        self.depth_service.health()
        caps = self.rpc("GET", "/v21/capabilities")
        if caps.get("backend") != "owl_ego" or caps.get("protocol_version") != 1:
            raise RuntimeError("Robot must implement owl_ego protocol_version=1")
        for key in ("async_navigation", "cancel_and_hold", "synchronized_observation",
                    "control_lease", "relative_xyz_yaw"):
            if caps.get(key) is not True:
                raise RuntimeError(f"Missing Robot capability: {key}")
        if self.auto_arm and caps.get("software_takeoff") is not True:
            raise RuntimeError("Missing Robot capability: software_takeoff")
        self.get_motion_tolerances(refresh=True)
        # Validate calibration and metric depth before acquiring flight control.
        obs = self.observe()
        self.estimate_depth(obs)
        acquisition_sent = time.monotonic()
        result = self.rpc("POST", "/v21/session", {"request_id": str(uuid.uuid4())})
        self.session_id = result["session_id"]
        if not isinstance(self.session_id, str) or not self.session_id:
            raise RuntimeError("Invalid session_id")
        self._lease_sent = acquisition_sent
        self.check_lease()
        self._lease_thread = threading.Thread(target=self._heartbeat_loop,
                                              name="owl-ego-heartbeat", daemon=True)
        self._lease_thread.start()
        self._init()  # A new lease always initializes, even after an old session.
        result = super().start()
        self.wait_stopped()
        return result

    def _request_json(self, method, path, payload=None, **kwargs):
        if method == "POST" and path in {"/init", "/takeoff", "/v21/navigation", "/move_relative_xyz_yaw"}:
            self._wait_observation_recovery()
            self.check_lease()
            if self._motion_failure is not None:
                raise RuntimeError(f"Motion failure latched: {self._motion_failure}")
            if (path in {"/v21/navigation", "/move_relative_xyz_yaw"}
                    and self._recovery_version != self._checked_recovery_version):
                self.flight_health()
        # Existing init/takeoff/relative-motion/land calls carry the same lease.
        if method == "POST" and self.session_id and not path.startswith("/v21/"):
            payload = dict(payload or {})
            payload.setdefault("session_id", self.session_id)
            payload.setdefault("request_id", str(uuid.uuid4()))
        self.event("http_request", method=method, path=path,
                   payload={k: v for k, v in (payload or {}).items() if k != "session_id"})
        try:
            return super()._request_json(method, path, payload, **kwargs)
        except Exception as exc:
            if (self._lease_sent is not None and self._transport_failure(exc)
                    and method == "GET" and (path in {"/health", "/get_pose"}
                    or path.startswith("/v21/navigation/status?"))):
                self._begin_recovery()
                self.event("telemetry_recovery", path=path, error=str(exc))
                self.check_lease()
                # Read only, once, after confirmed renewal. Never replay a motion.
                return super()._request_json(method, path, payload, **kwargs)
            # No automatic mutation retry. Retain the exact request for explicit
            # reconciliation/replay; never generate a fresh action on timeout.
            exc.request_method, exc.request_path = method, path
            exc.request_payload = dict(payload) if payload is not None else None
            self.event("http_error", method=method, path=path, error=str(exc),
                       request_id=(payload or {}).get("request_id"))
            raise

    def _takeoff(self):
        if self.timeout_s <= 60:
            raise ValueError("Takeoff transport timeout must exceed 60 s")
        result = self.rpc("POST", "/takeoff", {"auto_arm": self.auto_arm}, timeout_s=self.timeout_s)
        self._completed_motion(result)
        self.wait_stopped()
        return result

    def flight_health(self):
        self.check_lease()
        if self._motion_failure is not None:
            raise RuntimeError(f"Motion failure latched: {self._motion_failure}")
        h = self._health_state()
        recovery_version = self._recovery_version
        self.event("flight_health", health=h)
        if h.get("manual_takeover") or h.get("control_owner") not in (None, "agent"):
            self._fail_lease("operator takeover")
            self.check_lease()
        for key in ("initialized", "airborne", "control_ready", "hold_ready", "odom_ok"):
            if h.get(key) is not True:
                raise RuntimeError(f"Missing flight capability: {key}")
        if self._frame_identity is None or h.get("localization_epoch") != self._frame_identity[1]:
            raise RuntimeError("Localization reset during mission")
        if h.get("manual_takeover") or h.get("landing") or h.get("conflicting_publishers") or h.get("error"):
            raise RuntimeError(f"Flight authority/motion failed: {h}")
        if h.get("frame_alignment_ok") is False:
            raise RuntimeError("Robot world alignment invalid")
        phase = h.get("planner_state")
        if phase not in {"starting", "ready", "not_required"}:
            raise RuntimeError(f"Planner unavailable: {phase}")
        if phase == "ready" and h.get("planner_ok") is not True:
            raise RuntimeError("Ready planner lacks heartbeat")
        if phase == "starting":
            task = h.get("active_task_id")
            if not task:
                raise RuntimeError("Starting planner has no task")
            if self._starting_task != task:
                self._starting_task, self._starting_since = task, time.monotonic()
            elif time.monotonic() - self._starting_since > 10:
                raise RuntimeError("Planner startup exceeded 10 s")
            state = self.navigation_status(task)
            # Status is a later snapshot: a same-task terminal transition is legal.
            if state["status"] not in {"accepted", "planning", "executing", "stopping", "arrived", "cancelled"}:
                raise RuntimeError(f"Planner task failed: {state}")
        else:
            self._starting_task = self._starting_since = None
        if h.get("rgb_ok") is True:
            self._rgb_unavailable_since = None
        elif (h.get("observation_error_code") == "observation_unavailable"
              and h.get("observation_retryable") is True):
            now = time.monotonic()
            if self._rgb_unavailable_since is None:
                self._rgb_unavailable_since = now
            if now - self._rgb_unavailable_since > self.observation_retry_s:
                raise RuntimeError("Synchronized observation unavailable beyond retry budget")
        else:
            raise RuntimeError(f"Invalid observation health: {h}")
        if recovery_version != self._checked_recovery_version:
            if self._last_task_id:
                state = self.navigation_status(self._last_task_id)
                if state["status"] == "failed":
                    self._motion_failure = str(state)
                    raise RuntimeError(f"Original task failed during recovery: {state}")
                if state["status"] in {"arrived", "cancelled"} and state.get("stopped") is not True:
                    raise RuntimeError("Recovered terminal task lacks stop confirmation")
                self.event("lease_task_reconciled", state=state, health=h)
            self._checked_recovery_version = recovery_version
        return h

    def wait_stopped(self):
        deadline = time.monotonic() + self.stop_timeout_s
        while True:
            h = self.flight_health()
            if h.get("stopped") is True and "active_task_id" in h and h["active_task_id"] is None:
                return h
            if time.monotonic() >= deadline:
                raise RuntimeError("Current stop confirmation timed out")
            time.sleep(0.05)

    def _completed_motion(self, result, *, require_task=False):
        if require_task and not result.get("task_id"):
            raise RuntimeError("Motion response lacks task_id for completion confirmation")
        if result.get("task_id"):
            self._last_task_id = result["task_id"]
            state = self.navigation_status(result["task_id"])
            self.event("motion_completed", state=state)
            if state["status"] != "arrived" or state.get("stopped") is not True:
                raise RuntimeError(f"Robot motion did not arrive: {state}")

    def move_relative(self, dx=0., dy=0., dz=0., dyaw=0.):
        return self.move_rel_xyz_yaw(dx, dy, dz, dyaw)

    def move_rel_xyz_yaw(self, x=0., y=0., z=0., yaw=0., timeout_s=None):
        budget = 15.0 if timeout_s is None else timeout_s
        if isinstance(budget, bool) or not math.isfinite(budget) or not 0 < budget <= 170:
            raise ValueError("Relative motion timeout must be in (0,170]")
        if self.timeout_s <= budget:
            raise ValueError("Transport timeout must exceed motion timeout")
        self.wait_stopped()
        done, outcome = threading.Event(), {}
        def send():
            try:
                outcome["result"] = super(OwlEgoClient, self).move_rel_xyz_yaw(x,y,z,yaw,budget)
            except BaseException as exc:
                outcome["error"] = exc
            finally:
                done.set()
        worker = threading.Thread(target=send, name="owl-ego-relative", daemon=True)
        worker.start()
        owned_task = None
        try:
            while not done.wait(0.1):
                h = self.flight_health()
                active = h.get("active_task_id")
                if active:
                    if owned_task and active != owned_task:
                        raise RuntimeError("Relative task ownership changed")
                    owned_task = active
                    self._last_task_id = active
                self.get_pose()  # Timestamped samples for XYZ diagnostics and epoch checks.
            if "error" in outcome:
                raise outcome["error"]
            self._completed_motion(outcome["result"], require_task=any(
                self.quantize_motion(x,y,z,yaw).values()))
            self.wait_stopped()
            return outcome["result"]
        except BaseException:
            self._motion_failure = "relative action failed or result uncertain"
            if owned_task:
                try:
                    self.cancel_navigation(owned_task)
                except Exception as exc:
                    self.event("relative_cancel_failed", task_id=owned_task, error=str(exc))
            raise

    def land(self):
        self.check_lease()
        if self.timeout_s <= 90:
            raise ValueError("Landing transport timeout must exceed 90 s")
        # The blocking Robot wait confirms fresh ON_GROUND/disarmed. Do not apply
        # airborne/epoch checks to this already accepted landing.
        result = self.rpc("POST", "/land", {}, timeout_s=self.timeout_s)
        self._completed_motion(result, require_task=True)
        self._frame_identity = self._image_shape = self.last_observation = None
        self.event("landed", result=result)
        return result

    def _wait_observation_recovery(self):
        while not self._observation_ready.wait(0.05):
            self.check_lease()
        if self._observation_error is not None:
            raise RuntimeError(f"Observation recovery failed: {self._observation_error}")

    def observe(self):
        try:
            obs = self._observe_with_recovery()
        except Exception as exc:
            self._observation_error = exc
            self.event("observation_failed", error=str(exc))
            raise
        else:
            if not self._observation_ready.is_set():
                self.event("observation_recovered", frame_id=obs.frame_id)
            return obs
        finally:
            # Failure remains latched for motion; cancellation/landing bypass it.
            self._observation_ready.set()

    def _observe_with_recovery(self):
        self.check_lease()
        began = time.monotonic()
        deadline = began + self.observation_retry_s
        network_deadline = began + 2.0
        attempts = 0
        while True:
            self.check_lease()
            started = time.monotonic()
            if started >= deadline:
                raise TimeoutError("Observation recovery budget exhausted")
            attempts += 1
            try:
                data = self.rpc("GET", "/v21/observation",
                                timeout_s=min(0.5, deadline-started))
                if time.monotonic() >= deadline:
                    raise TimeoutError("Observation response exceeded recovery budget")
                break
            except Exception as exc:
                transport = self._transport_failure(exc)
                transient = (isinstance(exc, RobotHTTPError) and exc.status == 503
                             and exc.error_code == "observation_unavailable" and exc.retryable)
                if transport:
                    deadline = network_deadline
                if not (transport or transient) or time.monotonic() + 0.05 >= deadline:
                    raise
                self._observation_ready.clear()
                self.event("observation_retry", attempt=attempts, error=str(exc),
                           elapsed_s=time.monotonic()-began,
                           remaining_s=deadline-time.monotonic())
                time.sleep(0.05)
        obs = decode_observation(data,
                                 max_age_s=self.observation_max_age_s,
                                 max_sync_error_s=self.observation_max_sync_error_s,
                                 allow_approximate_geometry=self.allow_approximate_geometry)
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
        self.event("observation", frame_id=obs.frame_id, pose=obs.pose,
                   localization_epoch=obs.localization_epoch, rectified=obs.rectified,
                   calibration_quality=obs.calibration_quality,
                   geometry_assumptions=obs.geometry_assumptions,
                   intrinsics=obs.intrinsics.tolist(),
                   world_from_camera_optical_cm=obs.world_from_camera.tolist())
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
        self.event("pose_sample", pose=pose, localization_epoch=result.get("localization_epoch"))
        return pose

    def navigate(self, pose):
        self.wait_stopped()
        self._last_task_id = self.rpc("POST", "/v21/navigation", {
            "session_id": self.session_id, "request_id": str(uuid.uuid4()),
            "localization_epoch": self._frame_identity[1], "pose": dict(pose),
        })["task_id"]
        return self._last_task_id

    def navigation_status(self, task_id):
        self.check_lease()
        result = self.rpc("GET", "/v21/navigation/status?task_id=" + quote(task_id, safe=""))
        if result.get("task_id") != task_id:
            raise RuntimeError("Mismatched navigation task_id")
        if result.get("status") not in {"accepted", "planning", "executing", "stopping",
                                        "arrived", "cancelled", "failed"}:
            raise RuntimeError("Invalid navigation status")
        if result["status"] == "failed":
            self._motion_failure = result.get("error", "Robot task failed")
        self.event("navigation_status", state=result)
        return result

    def cancel_navigation(self, task_id):
        return self.rpc("POST", "/v21/navigation/cancel", {
            "session_id": self.session_id, "task_id": task_id,
            "request_id": str(uuid.uuid4()),
        })

    def close(self):
        # Release must hold rather than disarm. Lease expiry covers network loss.
        try:
            if self.session_id and self._lease_error is None:
                return self.rpc("POST", "/v21/session/release",
                                {"session_id": self.session_id})
        finally:
            self._lease_stop.set()
            with self._lease_cv:
                self._lease_cv.notify_all()
            if self._lease_thread:
                self._lease_thread.join(timeout=3)
            self.session_id = None
