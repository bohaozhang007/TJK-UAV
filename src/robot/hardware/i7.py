"""ROS1 and RTSP communication layer for the I7 UAV."""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any, Dict, Optional

import cv2
import numpy as np


class I7Hardware:
    """Cache I7 telemetry and camera frames and expose nav-node commands."""

    LANDED_STATE_ON_GROUND = 1
    LANDED_STATE_IN_AIR = 2

    def __init__(
        self,
        *,
        node_name: str,
        rtsp_url: str,
        ready_timeout_s: float,
        ros_services_ready_timeout_s: float,
        state_max_age_s: float,
        odom_max_age_s: float,
        rgb_max_age_s: float,
        nav_max_age_s: float,
        planner_max_age_s: float,
        battery_max_age_s: float,
        condition_wait_timeout_s: float,
        camera_open_retry_s: float,
        camera_read_retry_s: float,
        camera_join_timeout_s: float,
        rtsp_io_timeout_us: int,
        camera_buffer_size: int,
        control_queue_size: int,
        status_queue_size: int,
        odom_topic: str,
        state_topic: str,
        extended_state_topic: str,
        battery_topic: str,
        nav_state_topic: str,
        nav_active_goal_topic: str,
        planner_heartbeat_topic: str,
        goal_topic: str,
        takeoff_service: str,
        landing_service: str,
        force_land_service: str,
        abort_service: str,
        reinitialize_service: str,
    ) -> None:
        self.node_name = self._nonempty_string("node_name", node_name)
        self.rtsp_url = self._nonempty_string("rtsp_url", rtsp_url)
        self.ready_timeout_s = self._positive_threshold(
            "ready_timeout_s", ready_timeout_s
        )
        self.ros_services_ready_timeout_s = self._positive_threshold(
            "ros_services_ready_timeout_s", ros_services_ready_timeout_s
        )
        self.state_max_age_s = self._positive_threshold(
            "state_max_age_s", state_max_age_s
        )
        self.odom_max_age_s = self._positive_threshold(
            "odom_max_age_s", odom_max_age_s
        )
        self.rgb_max_age_s = self._positive_threshold(
            "rgb_max_age_s", rgb_max_age_s
        )
        self.nav_max_age_s = self._positive_threshold(
            "nav_max_age_s", nav_max_age_s
        )
        self.planner_max_age_s = self._positive_threshold(
            "planner_max_age_s", planner_max_age_s
        )
        self.battery_max_age_s = self._positive_threshold(
            "battery_max_age_s", battery_max_age_s
        )
        self.condition_wait_timeout_s = self._positive_threshold(
            "condition_wait_timeout_s", condition_wait_timeout_s
        )
        self.camera_open_retry_s = self._positive_threshold(
            "camera_open_retry_s", camera_open_retry_s
        )
        self.camera_read_retry_s = self._positive_threshold(
            "camera_read_retry_s", camera_read_retry_s
        )
        self.camera_join_timeout_s = self._positive_threshold(
            "camera_join_timeout_s", camera_join_timeout_s
        )
        self.rtsp_io_timeout_us = int(
            self._positive_threshold("rtsp_io_timeout_us", rtsp_io_timeout_us)
        )
        self.camera_buffer_size = int(
            self._positive_threshold("camera_buffer_size", camera_buffer_size)
        )
        self.control_queue_size = int(
            self._positive_threshold("control_queue_size", control_queue_size)
        )
        self.status_queue_size = int(
            self._positive_threshold("status_queue_size", status_queue_size)
        )
        self.odom_topic = self._nonempty_string("odom_topic", odom_topic)
        self.state_topic = self._nonempty_string("state_topic", state_topic)
        self.extended_state_topic = self._nonempty_string(
            "extended_state_topic", extended_state_topic
        )
        self.battery_topic = self._nonempty_string("battery_topic", battery_topic)
        self.nav_state_topic = self._nonempty_string("nav_state_topic", nav_state_topic)
        self.nav_active_goal_topic = self._nonempty_string(
            "nav_active_goal_topic", nav_active_goal_topic
        )
        self.planner_heartbeat_topic = self._nonempty_string(
            "planner_heartbeat_topic", planner_heartbeat_topic
        )
        self.goal_topic = self._nonempty_string("goal_topic", goal_topic)
        self.takeoff_service = self._nonempty_string(
            "takeoff_service", takeoff_service
        )
        self.landing_service = self._nonempty_string(
            "landing_service", landing_service
        )
        self.force_land_service = self._nonempty_string(
            "force_land_service", force_land_service
        )
        self.abort_service = self._nonempty_string("abort_service", abort_service)
        self.reinitialize_service = self._nonempty_string(
            "reinitialize_service", reinitialize_service
        )
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._started = False
        self._services_ready = False
        self._rospy = None
        self._message_types: Dict[str, Any] = {}
        self._publishers = []
        self._subscribers = []
        self._goal_pub = None
        self._service_clients: Dict[str, Any] = {}

        self._state: Optional[Dict[str, Any]] = None
        self._extended_state: Optional[Dict[str, Any]] = None
        self._odom: Optional[Dict[str, Any]] = None
        self._battery: Optional[Dict[str, Any]] = None
        self._nav: Optional[Dict[str, Any]] = None
        self._active_goal: Optional[Dict[str, Any]] = None
        self._planner_heartbeat_received_monotonic = 0.0
        self._rgb: Optional[Dict[str, Any]] = None

        self._camera_stop = threading.Event()
        self._camera_thread: Optional[threading.Thread] = None
        self._camera_error: Optional[str] = None

    def connect(self, timeout_s: Optional[float] = None) -> Dict[str, Any]:
        timeout_s = (
            self.ready_timeout_s if timeout_s is None else float(timeout_s)
        )
        with self._condition:
            was_started = self._started
            if not self._started:
                self._start_ros_locked()
                self._start_camera_locked()

        self._wait_for_services(
            timeout_s=min(self.ros_services_ready_timeout_s, timeout_s)
        )
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while True:
                health = self._health_locked()
                if health["initialized"]:
                    return {
                        "ok": True,
                        "message": (
                            "I7 hardware already initialized"
                            if was_started
                            else "I7 hardware initialized"
                        ),
                        "health": health,
                    }
                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0:
                    missing = ", ".join(health["missing"]) or "unknown"
                    raise RuntimeError(
                        f"I7 initialization timeout: missing or stale: {missing}"
                    )
                self._condition.wait(
                    timeout=min(self.condition_wait_timeout_s, remaining_s)
                )

    def _start_ros_locked(self) -> None:
        try:
            import rospy
            from geometry_msgs.msg import PoseStamped
            from mavros_msgs.msg import ExtendedState, State
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import BatteryState
            from std_msgs.msg import Empty, String
            from std_srvs.srv import Trigger
        except ImportError as exc:
            raise RuntimeError(
                "I7 hardware requires ROS Noetic Python packages"
            ) from exc

        if not rospy.core.is_initialized():
            rospy.init_node(self.node_name, anonymous=False, disable_signals=True)

        self._rospy = rospy
        self._message_types = {"PoseStamped": PoseStamped}
        self._goal_pub = rospy.Publisher(
            self.goal_topic,
            PoseStamped,
            queue_size=self.status_queue_size,
        )
        self._publishers = [self._goal_pub]
        self._subscribers = [
            rospy.Subscriber(
                self.odom_topic,
                Odometry,
                self._odom_callback,
                queue_size=self.control_queue_size,
            ),
            rospy.Subscriber(
                self.state_topic,
                State,
                self._state_callback,
                queue_size=self.control_queue_size,
            ),
            rospy.Subscriber(
                self.extended_state_topic,
                ExtendedState,
                self._extended_state_callback,
                queue_size=self.control_queue_size,
            ),
            rospy.Subscriber(
                self.battery_topic,
                BatteryState,
                self._battery_callback,
                queue_size=self.status_queue_size,
            ),
            rospy.Subscriber(
                self.nav_state_topic,
                String,
                self._nav_callback,
                queue_size=self.status_queue_size,
            ),
            rospy.Subscriber(
                self.nav_active_goal_topic,
                PoseStamped,
                self._active_goal_callback,
                queue_size=self.status_queue_size,
            ),
            rospy.Subscriber(
                self.planner_heartbeat_topic,
                Empty,
                self._planner_heartbeat_callback,
                queue_size=self.control_queue_size,
            ),
        ]
        self._service_clients = {
            "takeoff": rospy.ServiceProxy(self.takeoff_service, Trigger),
            "land": rospy.ServiceProxy(self.landing_service, Trigger),
            "force_land": rospy.ServiceProxy(self.force_land_service, Trigger),
            "abort": rospy.ServiceProxy(self.abort_service, Trigger),
            "reinitialize": rospy.ServiceProxy(self.reinitialize_service, Trigger),
        }
        self._started = True

    def _wait_for_services(self, timeout_s: float) -> None:
        if self._rospy is None:
            raise RuntimeError("ROS is not initialized")
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        for service_name in (
            self.takeoff_service,
            self.landing_service,
            self.force_land_service,
            self.abort_service,
            self.reinitialize_service,
        ):
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0:
                raise RuntimeError(f"I7 navigation service unavailable: {service_name}")
            self._rospy.wait_for_service(service_name, timeout=remaining_s)
        with self._condition:
            self._services_ready = True
            self._condition.notify_all()

    def _start_camera_locked(self) -> None:
        if self._camera_thread is not None and self._camera_thread.is_alive():
            return
        self._camera_stop.clear()
        self._camera_error = None
        self._camera_thread = threading.Thread(
            target=self._camera_loop,
            name="i7-rtsp-reader",
            daemon=True,
        )
        self._camera_thread.start()

    def _camera_loop(self) -> None:
        # Keep latency bounded: TCP avoids corrupt UDP frames and the relay runs locally.
        os.environ.setdefault(
            "OPENCV_FFMPEG_CAPTURE_OPTIONS",
            f"rtsp_transport;tcp|stimeout;{self.rtsp_io_timeout_us}",
        )
        while not self._camera_stop.is_set():
            capture = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, self.camera_buffer_size)
            if not capture.isOpened():
                with self._condition:
                    self._camera_error = f"cannot open RTSP stream {self.rtsp_url}"
                    self._condition.notify_all()
                capture.release()
                self._camera_stop.wait(self.camera_open_retry_s)
                continue
            with self._condition:
                self._camera_error = None
            while not self._camera_stop.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    with self._condition:
                        self._camera_error = f"RTSP read failed: {self.rtsp_url}"
                        self._condition.notify_all()
                    break
                with self._condition:
                    self._rgb = {
                        "frame_bgr": frame.copy(),
                        "width": int(frame.shape[1]),
                        "height": int(frame.shape[0]),
                        "received_monotonic": time.monotonic(),
                    }
                    self._camera_error = None
                    self._condition.notify_all()
            capture.release()
            self._camera_stop.wait(self.camera_read_retry_s)

    def call_takeoff(self) -> Dict[str, Any]:
        return self._call_trigger("takeoff")

    def call_land(self) -> Dict[str, Any]:
        return self._call_trigger("land")

    def call_force_land(self) -> Dict[str, Any]:
        return self._call_trigger("force_land")

    def call_abort(self) -> Dict[str, Any]:
        return self._call_trigger("abort")

    def call_reinitialize(self) -> Dict[str, Any]:
        return self._call_trigger("reinitialize")

    def _call_trigger(self, name: str) -> Dict[str, Any]:
        with self._lock:
            self._require_started()
            client = self._service_clients[name]
        response = client()
        result = {"ok": bool(response.success), "message": str(response.message)}
        if not response.success:
            raise RuntimeError(str(response.message) or f"I7 {name} failed")
        return result

    def publish_goal(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        yaw_deg_ros: float,
        frame_id: str,
        *,
        use_planner: bool = True,
    ) -> None:
        values = (x_m, y_m, z_m, yaw_deg_ros)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("I7 goal contains a non-finite value")
        with self._lock:
            self._require_started()
            if not self._publisher_connected(self._goal_pub):
                raise RuntimeError(f"i7_nav is not subscribed to {self.goal_topic}")
            message = self._message_types["PoseStamped"]()
            message.header.stamp = self._rospy.Time.now()
            message.header.frame_id = str(frame_id or "camera_init")
            message.pose.position.x = float(x_m)
            message.pose.position.y = float(y_m)
            message.pose.position.z = float(z_m)
            # /cxr_goal preserves the existing uav_nav convention: yaw degrees in w.
            # orientation.x is an I7-only marker: 0=EGO, 1=direct yaw at current pose.
            message.pose.orientation.x = 0.0 if use_planner else 1.0
            message.pose.orientation.w = float(yaw_deg_ros)
            publisher = self._goal_pub
        publisher.publish(message)

    def wait_for_goal_ack(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        *,
        uses_planner: bool,
        after_monotonic: float,
        timeout_s: float,
        tolerance_m: float,
    ) -> Dict[str, Any]:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while True:
                goal = self._active_goal
                if self._goal_matches(
                    goal,
                    x_m,
                    y_m,
                    z_m,
                    after_monotonic,
                    tolerance_m,
                    uses_planner,
                ):
                    return dict(goal or {})
                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0:
                    raise RuntimeError(
                        "i7_nav did not acknowledge goal on "
                        f"{self.nav_active_goal_topic}"
                    )
                self._condition.wait(
                    timeout=min(self.condition_wait_timeout_s, remaining_s)
                )

    def get_pose_ros(self) -> Dict[str, Any]:
        with self._lock:
            if not self._fresh(self._odom, time.monotonic(), self.odom_max_age_s):
                raise RuntimeError(f"missing or stale odometry from {self.odom_topic}")
            return dict(self._odom or {})

    def get_bgr(self) -> np.ndarray:
        with self._lock:
            if not self._fresh(self._rgb, time.monotonic(), self.rgb_max_age_s):
                detail = f": {self._camera_error}" if self._camera_error else ""
                raise RuntimeError(f"missing or stale I7 RTSP frame{detail}")
            return np.asarray(self._rgb["frame_bgr"]).copy()

    def health(self) -> Dict[str, Any]:
        with self._lock:
            return self._health_locked()

    def close(self) -> None:
        self._camera_stop.set()
        camera_thread = self._camera_thread
        if camera_thread is not None and camera_thread.is_alive():
            camera_thread.join(timeout=self.camera_join_timeout_s)
        with self._lock:
            handles = list(self._subscribers) + list(self._publishers)
            self._subscribers = []
            self._publishers = []
            self._goal_pub = None
            self._service_clients = {}
            self._started = False
            self._services_ready = False
            self._camera_thread = None
            self._state = None
            self._extended_state = None
            self._odom = None
            self._battery = None
            self._nav = None
            self._active_goal = None
            self._planner_heartbeat_received_monotonic = 0.0
            self._rgb = None
        for handle in handles:
            try:
                handle.unregister()
            except Exception:
                pass

    def _state_callback(self, message: Any) -> None:
        with self._condition:
            self._state = {
                "connected": bool(message.connected),
                "armed": bool(message.armed),
                "mode": str(message.mode),
                "received_monotonic": time.monotonic(),
            }
            self._condition.notify_all()

    def _extended_state_callback(self, message: Any) -> None:
        with self._condition:
            self._extended_state = {
                "landed_state": int(message.landed_state),
                "received_monotonic": time.monotonic(),
            }
            self._condition.notify_all()

    def _odom_callback(self, message: Any) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        quaternion = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        with self._condition:
            self._odom = {
                "x_m": float(pose.position.x),
                "y_m": float(pose.position.y),
                "z_m": float(pose.position.z),
                "yaw_rad": self._quaternion_yaw(quaternion),
                "vx_m_s": float(twist.linear.x),
                "vy_m_s": float(twist.linear.y),
                "vz_m_s": float(twist.linear.z),
                "frame_id": str(message.header.frame_id or "camera_init"),
                "received_monotonic": time.monotonic(),
            }
            self._condition.notify_all()

    def _battery_callback(self, message: Any) -> None:
        percentage = float(message.percentage)
        voltage = float(message.voltage)
        with self._condition:
            self._battery = {
                "percentage": percentage if math.isfinite(percentage) else None,
                "voltage": voltage if math.isfinite(voltage) else None,
                "received_monotonic": time.monotonic(),
            }
            self._condition.notify_all()

    def _nav_callback(self, message: Any) -> None:
        try:
            payload = json.loads(str(message.data))
            if not isinstance(payload, dict):
                raise ValueError("state payload is not an object")
        except Exception as exc:
            payload = {"state": "ERROR", "last_error": f"invalid nav state: {exc}"}
        payload["received_monotonic"] = time.monotonic()
        with self._condition:
            self._nav = payload
            self._condition.notify_all()

    def _active_goal_callback(self, message: Any) -> None:
        with self._condition:
            self._active_goal = {
                "x_m": float(message.pose.position.x),
                "y_m": float(message.pose.position.y),
                "z_m": float(message.pose.position.z),
                "frame_id": str(message.header.frame_id),
                "uses_planner": float(message.pose.orientation.x) <= 0.5,
                "received_monotonic": time.monotonic(),
            }
            self._condition.notify_all()

    def _planner_heartbeat_callback(self, _message: Any) -> None:
        with self._condition:
            self._planner_heartbeat_received_monotonic = time.monotonic()
            self._condition.notify_all()

    def _health_locked(self) -> Dict[str, Any]:
        now = time.monotonic()
        state = dict(self._state or {})
        extended = dict(self._extended_state or {})
        nav = dict(self._nav or {})
        battery = dict(self._battery or {})
        state_ok = self._fresh(self._state, now, self.state_max_age_s)
        extended_ok = self._fresh(self._extended_state, now, self.state_max_age_s)
        odom_ok = self._fresh(self._odom, now, self.odom_max_age_s)
        rgb_ok = self._fresh(self._rgb, now, self.rgb_max_age_s)
        nav_ok = self._fresh(self._nav, now, self.nav_max_age_s)
        planner_heartbeat_ok = bool(
            self._planner_heartbeat_received_monotonic > 0.0
            and now - self._planner_heartbeat_received_monotonic
            <= self.planner_max_age_s
        )
        planner_ok = bool(
            planner_heartbeat_ok and nav_ok and nav.get("planner_ok") is True
        )
        battery_ok = self._fresh(self._battery, now, self.battery_max_age_s)
        camera_width = (
            int(self._rgb["width"])
            if self._rgb is not None and "width" in self._rgb
            else None
        )
        camera_height = (
            int(self._rgb["height"])
            if self._rgb is not None and "height" in self._rgb
            else None
        )
        connected = bool(state.get("connected", False))
        goal_link_ok = self._publisher_connected(self._goal_pub)
        initialized = bool(
            self._started
            and self._services_ready
            and state_ok
            and extended_ok
            and odom_ok
            and rgb_ok
            and nav_ok
            and planner_ok
            and connected
            and goal_link_ok
        )
        mode = str(state.get("mode") or "")
        landed_state = extended.get("landed_state")
        airborne = bool(
            state_ok
            and extended_ok
            and state.get("armed") is True
            and landed_state == self.LANDED_STATE_IN_AIR
        )
        missing = []
        for name, ready in (
            ("ros", self._started),
            ("nav_services", self._services_ready),
            ("mavros_state", state_ok),
            ("mavros_connected", connected),
            ("extended_state", extended_ok),
            ("odom", odom_ok),
            ("rgb", rgb_ok),
            ("i7_nav_state", nav_ok),
            ("ego_planner_heartbeat", planner_ok),
            ("i7_goal_link", goal_link_ok),
        ):
            if not ready:
                missing.append(name)
        return {
            "ros_initialized": self._started,
            "initialized": initialized,
            "airborne": airborne,
            "offboard": mode.upper() == "OFFBOARD",
            "control_ready": bool(initialized and nav.get("control_ready") is True),
            "manual_takeover_latched": bool(nav.get("manual_takeover_latched", False)),
            "nav_state": nav.get("state"),
            "nav_last_error": nav.get("last_error"),
            "control_session_active": bool(nav.get("control_session_active", False)),
            "ground_z_m": nav.get("ground_z_m"),
            "min_height_m": nav.get("min_height_m"),
            "max_height_m": nav.get("max_height_m"),
            "connected": connected,
            "armed": bool(state.get("armed", False)),
            "mode": mode,
            "landed_state": landed_state,
            "state_ok": state_ok,
            "extended_state_ok": extended_ok,
            "odom_ok": odom_ok,
            "rgb_ok": rgb_ok,
            "nav_ok": nav_ok,
            "planner_ok": planner_ok,
            "planner_heartbeat_ok": planner_heartbeat_ok,
            "planner_heartbeat_topic": self.planner_heartbeat_topic,
            "goal_link_ok": goal_link_ok,
            "battery_ok": battery_ok,
            "battery_percentage": battery.get("percentage"),
            "battery_voltage": battery.get("voltage"),
            "camera_source": self.rtsp_url,
            "camera_width": camera_width,
            "camera_height": camera_height,
            "camera_error": self._camera_error,
            "missing": missing,
        }

    @staticmethod
    def _publisher_connected(publisher: Any) -> bool:
        try:
            return publisher is not None and int(publisher.get_num_connections()) > 0
        except Exception:
            return False

    @staticmethod
    def _positive_threshold(name: str, value: float) -> float:
        result = float(value)
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError(f"I7 {name} must be finite and positive")
        return result

    @staticmethod
    def _nonempty_string(name: str, value: str) -> str:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"I7 {name} must be a non-empty string")
        return value.strip()

    @staticmethod
    def _fresh(value: Optional[Dict[str, Any]], now: float, max_age_s: float) -> bool:
        return bool(
            value is not None
            and now - float(value.get("received_monotonic", 0.0)) <= max_age_s
        )

    @staticmethod
    def _goal_matches(
        goal: Optional[Dict[str, Any]],
        x_m: float,
        y_m: float,
        z_m: float,
        after_monotonic: float,
        tolerance_m: float,
        uses_planner: bool,
    ) -> bool:
        if goal is None or float(goal.get("received_monotonic", 0.0)) < after_monotonic:
            return False
        if bool(goal.get("uses_planner")) is not bool(uses_planner):
            return False
        return math.sqrt(
            (float(goal["x_m"]) - x_m) ** 2
            + (float(goal["y_m"]) - y_m) ** 2
            + (float(goal["z_m"]) - z_m) ** 2
        ) <= max(0.0, float(tolerance_m))

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError("I7 hardware is not initialized, call init first")

    @staticmethod
    def _quaternion_yaw(quaternion_xyzw: tuple[float, float, float, float]) -> float:
        x, y, z, w = quaternion_xyzw
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


__all__ = ["I7Hardware"]
