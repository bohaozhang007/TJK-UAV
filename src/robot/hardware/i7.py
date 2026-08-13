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

    ODOM_TOPIC = "/Odometry"
    STATE_TOPIC = "/mavros/state"
    EXTENDED_STATE_TOPIC = "/mavros/extended_state"
    BATTERY_TOPIC = "/mavros/battery"
    NAV_STATE_TOPIC = "/i7_nav/state"
    NAV_ACTIVE_GOAL_TOPIC = "/i7_nav/active_goal"
    PLANNER_HEARTBEAT_TOPIC = "/drone_0_traj_server/heartbeat"
    GOAL_TOPIC = "/cxr_goal"

    TAKEOFF_SERVICE = "/i7_nav/takeoff"
    LAND_SERVICE = "/i7_nav/land"
    FORCE_LAND_SERVICE = "/i7_nav/force_land"
    ABORT_SERVICE = "/i7_nav/abort"
    REINITIALIZE_SERVICE = "/i7_nav/reinitialize"

    READY_TIMEOUT_S = 15.0
    STATE_MAX_AGE_S = 2.0
    ODOM_MAX_AGE_S = 1.0
    RGB_MAX_AGE_S = 2.0
    NAV_MAX_AGE_S = 2.0
    PLANNER_MAX_AGE_S = 2.0
    BATTERY_MAX_AGE_S = 10.0

    LANDED_STATE_ON_GROUND = 1
    LANDED_STATE_IN_AIR = 2

    def __init__(
        self,
        node_name: str = "i7_robot_server",
        rtsp_url: str = "rtsp://127.0.0.1:8554/k40t",
    ) -> None:
        self.node_name = node_name
        self.rtsp_url = rtsp_url
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

    def connect(self, timeout_s: float = READY_TIMEOUT_S) -> Dict[str, Any]:
        with self._condition:
            was_started = self._started
            if not self._started:
                self._start_ros_locked()
                self._start_camera_locked()

        self._wait_for_services(timeout_s=min(5.0, max(0.1, timeout_s)))
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
                self._condition.wait(timeout=min(0.1, remaining_s))

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
        self._goal_pub = rospy.Publisher(self.GOAL_TOPIC, PoseStamped, queue_size=10)
        self._publishers = [self._goal_pub]
        self._subscribers = [
            rospy.Subscriber(self.ODOM_TOPIC, Odometry, self._odom_callback, queue_size=20),
            rospy.Subscriber(self.STATE_TOPIC, State, self._state_callback, queue_size=20),
            rospy.Subscriber(
                self.EXTENDED_STATE_TOPIC,
                ExtendedState,
                self._extended_state_callback,
                queue_size=20,
            ),
            rospy.Subscriber(
                self.BATTERY_TOPIC,
                BatteryState,
                self._battery_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                self.NAV_STATE_TOPIC,
                String,
                self._nav_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                self.NAV_ACTIVE_GOAL_TOPIC,
                PoseStamped,
                self._active_goal_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                self.PLANNER_HEARTBEAT_TOPIC,
                Empty,
                self._planner_heartbeat_callback,
                queue_size=20,
            ),
        ]
        self._service_clients = {
            "takeoff": rospy.ServiceProxy(self.TAKEOFF_SERVICE, Trigger),
            "land": rospy.ServiceProxy(self.LAND_SERVICE, Trigger),
            "force_land": rospy.ServiceProxy(self.FORCE_LAND_SERVICE, Trigger),
            "abort": rospy.ServiceProxy(self.ABORT_SERVICE, Trigger),
            "reinitialize": rospy.ServiceProxy(self.REINITIALIZE_SERVICE, Trigger),
        }
        self._started = True

    def _wait_for_services(self, timeout_s: float) -> None:
        if self._rospy is None:
            raise RuntimeError("ROS is not initialized")
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        for service_name in (
            self.TAKEOFF_SERVICE,
            self.LAND_SERVICE,
            self.FORCE_LAND_SERVICE,
            self.ABORT_SERVICE,
            self.REINITIALIZE_SERVICE,
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
            "rtsp_transport;tcp|stimeout;3000000",
        )
        while not self._camera_stop.is_set():
            capture = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                with self._condition:
                    self._camera_error = f"cannot open RTSP stream {self.rtsp_url}"
                    self._condition.notify_all()
                capture.release()
                self._camera_stop.wait(0.5)
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
            self._camera_stop.wait(0.2)

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
                raise RuntimeError(f"i7_nav is not subscribed to {self.GOAL_TOPIC}")
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
        tolerance_m: float = 0.05,
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
                        f"{self.NAV_ACTIVE_GOAL_TOPIC}"
                    )
                self._condition.wait(timeout=min(0.1, remaining_s))

    def get_pose_ros(self) -> Dict[str, Any]:
        with self._lock:
            if not self._fresh(self._odom, time.monotonic(), self.ODOM_MAX_AGE_S):
                raise RuntimeError(f"missing or stale odometry from {self.ODOM_TOPIC}")
            return dict(self._odom or {})

    def get_bgr(self) -> np.ndarray:
        with self._lock:
            if not self._fresh(self._rgb, time.monotonic(), self.RGB_MAX_AGE_S):
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
            camera_thread.join(timeout=3.5)
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
        state_ok = self._fresh(self._state, now, self.STATE_MAX_AGE_S)
        extended_ok = self._fresh(self._extended_state, now, self.STATE_MAX_AGE_S)
        odom_ok = self._fresh(self._odom, now, self.ODOM_MAX_AGE_S)
        rgb_ok = self._fresh(self._rgb, now, self.RGB_MAX_AGE_S)
        nav_ok = self._fresh(self._nav, now, self.NAV_MAX_AGE_S)
        planner_heartbeat_ok = bool(
            self._planner_heartbeat_received_monotonic > 0.0
            and now - self._planner_heartbeat_received_monotonic
            <= self.PLANNER_MAX_AGE_S
        )
        planner_ok = bool(
            planner_heartbeat_ok and nav_ok and nav.get("planner_ok") is True
        )
        battery_ok = self._fresh(self._battery, now, self.BATTERY_MAX_AGE_S)
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
            "planner_heartbeat_topic": self.PLANNER_HEARTBEAT_TOPIC,
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
