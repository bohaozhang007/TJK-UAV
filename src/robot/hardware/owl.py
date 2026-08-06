"""ROS1 communication layer for the VisBot OWL mini3L.

This module deliberately contains only ROS topic I/O and cached raw state.
Coordinate conversion, motion completion checks, image resizing, and safety
policy belong to ``controllers.owl``.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Dict, Optional, Tuple


class OwlHardware:
    """Thin ROS1 adapter for the topics exposed by the OWL flight stack."""

    ODOM_TOPIC = "/mavros/local_position/odom"
    RGB_TOPIC = "/visbot_media_g/gimbal_camera/image_raw/compressed"
    STATE_TOPIC = "/mavros/state"
    EXTENDED_STATE_TOPIC = "/mavros/extended_state"
    BATTERY_TOPIC = "/mavros/battery"
    CONTROL_TOPIC = "/control"
    GOAL_TOPIC = "/move_base_simple/goal"

    LANDED_STATE_ON_GROUND = 1
    LANDED_STATE_IN_AIR = 2

    def __init__(self, node_name: str = "owl_robot_server") -> None:
        self.node_name = node_name
        self._lock = threading.RLock()
        self._started = False
        self._rospy = None
        self._message_types: Dict[str, Any] = {}
        self._publishers = []
        self._subscribers = []
        self._control_pub = None
        self._goal_pub = None

        self._state: Optional[Dict[str, Any]] = None
        self._extended_state: Optional[Dict[str, Any]] = None
        self._odom: Optional[Dict[str, Any]] = None
        self._rgb: Optional[Dict[str, Any]] = None
        self._battery: Optional[Dict[str, Any]] = None

    def connect(self) -> Dict[str, Any]:
        """Initialize rospy and subscribe to all required raw topics."""
        with self._lock:
            if self._started:
                return {"ok": True, "message": "ROS hardware already initialized"}

            try:
                import rospy
                from controller_msgs.msg import control as Control
                from geometry_msgs.msg import PoseStamped
                from mavros_msgs.msg import ExtendedState, State
                from nav_msgs.msg import Odometry
                from sensor_msgs.msg import BatteryState, CompressedImage
            except ImportError as exc:
                raise RuntimeError(
                    "OWL hardware requires ROS Noetic Python packages "
                    "(rospy, controller_msgs, geometry_msgs, mavros_msgs, "
                    "nav_msgs, sensor_msgs)"
                ) from exc

            if not rospy.core.is_initialized():
                rospy.init_node(
                    self.node_name,
                    anonymous=False,
                    disable_signals=True,
                )

            self._rospy = rospy
            self._message_types = {
                "Control": Control,
                "PoseStamped": PoseStamped,
            }
            self._control_pub = rospy.Publisher(
                self.CONTROL_TOPIC,
                Control,
                queue_size=10,
            )
            self._goal_pub = rospy.Publisher(
                self.GOAL_TOPIC,
                PoseStamped,
                queue_size=10,
            )
            self._publishers = [self._control_pub, self._goal_pub]
            self._subscribers = [
                rospy.Subscriber(
                    self.ODOM_TOPIC,
                    Odometry,
                    self._odom_callback,
                    queue_size=10,
                ),
                rospy.Subscriber(
                    self.RGB_TOPIC,
                    CompressedImage,
                    self._rgb_callback,
                    queue_size=1,
                    buff_size=8 * 1024 * 1024,
                ),
                rospy.Subscriber(
                    self.STATE_TOPIC,
                    State,
                    self._state_callback,
                    queue_size=10,
                ),
                rospy.Subscriber(
                    self.EXTENDED_STATE_TOPIC,
                    ExtendedState,
                    self._extended_state_callback,
                    queue_size=10,
                ),
                rospy.Subscriber(
                    self.BATTERY_TOPIC,
                    BatteryState,
                    self._battery_callback,
                    queue_size=10,
                ),
            ]
            self._started = True
            return {"ok": True, "message": "ROS hardware initialized"}

    def publish_control(self, cmd: int, drone_id: int = 0) -> None:
        with self._lock:
            self._require_started()
            message = self._message_types["Control"]()
            message.header.stamp = self._rospy.Time.now()
            message.cmd = int(cmd)
            message.drone_id = int(drone_id)
            publisher = self._control_pub
        publisher.publish(message)

    def publish_goal(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        orientation_xyzw: Tuple[float, float, float, float],
        frame_id: str,
    ) -> None:
        with self._lock:
            self._require_started()
            message = self._message_types["PoseStamped"]()
            message.header.stamp = self._rospy.Time.now()
            message.header.frame_id = str(frame_id)
            message.pose.position.x = float(x_m)
            message.pose.position.y = float(y_m)
            message.pose.position.z = float(z_m)
            (
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ) = tuple(float(value) for value in orientation_xyzw)
            publisher = self._goal_pub
        publisher.publish(message)

    def get_pose_ros(self) -> Dict[str, Any]:
        with self._lock:
            if self._odom is None:
                raise RuntimeError(f"no odometry received from {self.ODOM_TOPIC}")
            return dict(self._odom)

    def get_compressed_rgb(self) -> Tuple[bytes, str]:
        with self._lock:
            if self._rgb is None:
                raise RuntimeError(f"no image received from {self.RGB_TOPIC}")
            return bytes(self._rgb["data"]), str(self._rgb["format"])

    def health(self) -> Dict[str, Any]:
        with self._lock:
            now = time.monotonic()
            state = dict(self._state) if self._state is not None else {}
            extended = (
                dict(self._extended_state)
                if self._extended_state is not None
                else {}
            )
            battery = dict(self._battery) if self._battery is not None else {}

            state_ok = self._fresh(self._state, now, 2.0)
            extended_state_ok = self._fresh(self._extended_state, now, 2.0)
            odom_ok = self._fresh(self._odom, now, 2.0)
            rgb_ok = self._fresh(self._rgb, now, 2.0)
            battery_ok = self._fresh(self._battery, now, 10.0)
            initialized = bool(
                self._started
                and state_ok
                and state.get("connected") is True
            )
            landed_state = extended.get("landed_state")
            airborne = bool(
                initialized
                and extended_state_ok
                and state.get("armed") is True
                and landed_state == self.LANDED_STATE_IN_AIR
            )
            mode = str(state.get("mode") or "")
            control_ready = bool(
                initialized
                and airborne
                and mode.upper() == "OFFBOARD"
                and odom_ok
                and rgb_ok
            )
            return {
                "initialized": initialized,
                "airborne": airborne,
                "control_ready": control_ready,
                "connected": bool(state.get("connected", False)),
                "armed": bool(state.get("armed", False)),
                "mode": mode,
                "landed_state": landed_state,
                "odom_ok": odom_ok,
                "rgb_ok": rgb_ok,
                "battery_ok": battery_ok,
                "battery_percentage": battery.get("percentage"),
                "battery_voltage": battery.get("voltage"),
            }

    def close(self) -> None:
        with self._lock:
            subscribers = list(self._subscribers)
            publishers = list(self._publishers)
            self._subscribers = []
            self._publishers = []
            self._control_pub = None
            self._goal_pub = None
            self._started = False
        for handle in subscribers + publishers:
            try:
                handle.unregister()
            except Exception:
                pass

    def _state_callback(self, message: Any) -> None:
        with self._lock:
            self._state = {
                "connected": bool(message.connected),
                "armed": bool(message.armed),
                "guided": bool(message.guided),
                "manual_input": bool(message.manual_input),
                "mode": str(message.mode),
                "system_status": int(message.system_status),
                "received_monotonic": time.monotonic(),
            }

    def _extended_state_callback(self, message: Any) -> None:
        with self._lock:
            self._extended_state = {
                "vtol_state": int(message.vtol_state),
                "landed_state": int(message.landed_state),
                "received_monotonic": time.monotonic(),
            }

    def _odom_callback(self, message: Any) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        quaternion = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        with self._lock:
            self._odom = {
                "x_m": float(pose.position.x),
                "y_m": float(pose.position.y),
                "z_m": float(pose.position.z),
                "orientation_xyzw": quaternion,
                "yaw_rad": self._quaternion_yaw(quaternion),
                "vx_m_s": float(twist.linear.x),
                "vy_m_s": float(twist.linear.y),
                "vz_m_s": float(twist.linear.z),
                "frame_id": str(message.header.frame_id or "world"),
                "child_frame_id": str(message.child_frame_id),
                "received_monotonic": time.monotonic(),
            }

    def _rgb_callback(self, message: Any) -> None:
        with self._lock:
            self._rgb = {
                "data": bytes(message.data),
                "format": str(message.format or "jpeg"),
                "received_monotonic": time.monotonic(),
            }

    def _battery_callback(self, message: Any) -> None:
        percentage = float(message.percentage)
        voltage = float(message.voltage)
        with self._lock:
            self._battery = {
                "percentage": percentage if math.isfinite(percentage) else None,
                "voltage": voltage if math.isfinite(voltage) else None,
                "received_monotonic": time.monotonic(),
            }

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError("ROS hardware is not initialized, call init first")

    @staticmethod
    def _fresh(
        value: Optional[Dict[str, Any]],
        now: float,
        max_age_s: float,
    ) -> bool:
        if value is None:
            return False
        timestamp = value.get("received_monotonic")
        return timestamp is not None and now - float(timestamp) <= max_age_s

    @staticmethod
    def _quaternion_yaw(
        quaternion_xyzw: Tuple[float, float, float, float]
    ) -> float:
        x, y, z, w = quaternion_xyzw
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)


__all__ = ["OwlHardware"]
