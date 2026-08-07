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
    YAW_TARGET_TOPIC = "/reference/yawsetpoint"
    PLANNING_GOAL_TOPIC = "/planning/goal"
    CAPTAIN_TARGET_POSE_TOPIC = "/captain/target_pose"

    READY_TIMEOUT_S = 10.0
    STATE_MAX_AGE_S = 2.0
    ODOM_MAX_AGE_S = 2.0
    RGB_MAX_AGE_S = 2.0
    BATTERY_MAX_AGE_S = 10.0

    LANDED_STATE_ON_GROUND = 1
    LANDED_STATE_IN_AIR = 2

    def __init__(self, node_name: str = "owl_robot_server") -> None:
        self.node_name = node_name
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._started = False
        self._rospy = None
        self._message_types: Dict[str, Any] = {}
        self._publishers = []
        self._subscribers = []
        self._control_pub = None
        self._goal_pub = None
        self._yaw_target_pub = None

        self._state: Optional[Dict[str, Any]] = None
        self._extended_state: Optional[Dict[str, Any]] = None
        self._odom: Optional[Dict[str, Any]] = None
        self._rgb: Optional[Dict[str, Any]] = None
        self._battery: Optional[Dict[str, Any]] = None
        self._planning_goal: Optional[Dict[str, Any]] = None
        self._captain_target_pose: Optional[Dict[str, Any]] = None

    def connect(self, timeout_s: float = READY_TIMEOUT_S) -> Dict[str, Any]:
        """Initialize ROS and wait until telemetry and Captain links are ready."""
        with self._condition:
            was_started = self._started
            if not self._started:
                self._start_ros()

            deadline = time.monotonic() + max(0.0, float(timeout_s))
            while True:
                health = self._health_locked()
                if health["initialized"]:
                    return {
                        "ok": True,
                        "message": (
                            "ROS hardware already initialized"
                            if was_started
                            else "ROS hardware initialized"
                        ),
                        "health": health,
                    }

                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0:
                    missing = ", ".join(health["missing"]) or "unknown"
                    raise RuntimeError(
                        "OWL initialization timeout: "
                        f"missing or stale: {missing}"
                    )
                self._condition.wait(timeout=min(0.1, remaining_s))

    def _start_ros(self) -> None:
        """Create ROS handles. Caller must hold ``self._condition``."""
        try:
            import rospy
            from controller_msgs.msg import YawTarget
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
            "YawTarget": YawTarget,
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
        self._yaw_target_pub = rospy.Publisher(
            self.YAW_TARGET_TOPIC,
            YawTarget,
            queue_size=10,
        )
        self._publishers = [
            self._control_pub,
            self._goal_pub,
            self._yaw_target_pub,
        ]
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
            rospy.Subscriber(
                self.PLANNING_GOAL_TOPIC,
                PoseStamped,
                self._planning_goal_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                self.CAPTAIN_TARGET_POSE_TOPIC,
                PoseStamped,
                self._captain_target_pose_callback,
                queue_size=10,
            ),
        ]
        self._started = True

    def publish_control(self, cmd: int, drone_id: int = 0) -> None:
        with self._lock:
            self._require_started()
            if not self._publisher_connected(self._control_pub):
                raise RuntimeError(
                    f"Captain is not subscribed to {self.CONTROL_TOPIC}"
                )
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
            if not self._publisher_connected(self._goal_pub):
                raise RuntimeError(
                    f"Captain is not subscribed to {self.GOAL_TOPIC}"
                )
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

    def publish_yaw_target(
        self,
        yaw_rad: float,
        yaw_rate_rad_s: float = 0.0,
        frame_id: str = "world",
    ) -> None:
        with self._lock:
            self._require_started()
            if not self._publisher_connected(self._yaw_target_pub):
                raise RuntimeError(
                    "mavros_controller is not subscribed to "
                    f"{self.YAW_TARGET_TOPIC}"
                )
            message = self._message_types["YawTarget"]()
            message.header.stamp = self._rospy.Time.now()
            message.header.frame_id = str(frame_id)
            message.yaw = float(yaw_rad)
            message.yaw_dot = float(yaw_rate_rad_s)
            publisher = self._yaw_target_pub
        publisher.publish(message)

    def wait_for_goal_ack(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        *,
        after_monotonic: float,
        timeout_s: float,
        tolerance_m: float = 0.05,
    ) -> Dict[str, Any]:
        """Wait until Captain echoes a matching goal toward its planner."""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while True:
                for source, goal in (
                    (self.PLANNING_GOAL_TOPIC, self._planning_goal),
                    (self.CAPTAIN_TARGET_POSE_TOPIC, self._captain_target_pose),
                ):
                    if self._goal_matches(
                        goal,
                        x_m,
                        y_m,
                        z_m,
                        after_monotonic,
                        tolerance_m,
                    ):
                        result = dict(goal or {})
                        result["source"] = source
                        return result

                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0:
                    raise RuntimeError(
                        "Captain did not acknowledge goal on "
                        f"{self.PLANNING_GOAL_TOPIC} or "
                        f"{self.CAPTAIN_TARGET_POSE_TOPIC}"
                    )
                self._condition.wait(timeout=min(0.1, remaining_s))

    def get_pose_ros(self) -> Dict[str, Any]:
        with self._lock:
            if self._odom is None:
                raise RuntimeError(f"no odometry received from {self.ODOM_TOPIC}")
            if not self._fresh(
                self._odom,
                time.monotonic(),
                self.ODOM_MAX_AGE_S,
            ):
                raise RuntimeError(f"stale odometry from {self.ODOM_TOPIC}")
            return dict(self._odom)

    def get_compressed_rgb(self) -> Tuple[bytes, str]:
        with self._lock:
            if self._rgb is None:
                raise RuntimeError(f"no image received from {self.RGB_TOPIC}")
            if not self._fresh(
                self._rgb,
                time.monotonic(),
                self.RGB_MAX_AGE_S,
            ):
                raise RuntimeError(f"stale image from {self.RGB_TOPIC}")
            return bytes(self._rgb["data"]), str(self._rgb["format"])

    def health(self) -> Dict[str, Any]:
        with self._lock:
            return self._health_locked()

    def close(self) -> None:
        with self._lock:
            subscribers = list(self._subscribers)
            publishers = list(self._publishers)
            self._subscribers = []
            self._publishers = []
            self._control_pub = None
            self._goal_pub = None
            self._yaw_target_pub = None
            self._started = False
            self._state = None
            self._extended_state = None
            self._odom = None
            self._rgb = None
            self._battery = None
            self._planning_goal = None
            self._captain_target_pose = None
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
            self._condition.notify_all()

    def _extended_state_callback(self, message: Any) -> None:
        with self._lock:
            self._extended_state = {
                "vtol_state": int(message.vtol_state),
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
            self._condition.notify_all()

    def _rgb_callback(self, message: Any) -> None:
        with self._lock:
            self._rgb = {
                "data": bytes(message.data),
                "format": str(message.format or "jpeg"),
                "received_monotonic": time.monotonic(),
            }
            self._condition.notify_all()

    def _battery_callback(self, message: Any) -> None:
        percentage = float(message.percentage)
        voltage = float(message.voltage)
        with self._lock:
            self._battery = {
                "percentage": percentage if math.isfinite(percentage) else None,
                "voltage": voltage if math.isfinite(voltage) else None,
                "received_monotonic": time.monotonic(),
            }
            self._condition.notify_all()

    def _planning_goal_callback(self, message: Any) -> None:
        with self._condition:
            self._planning_goal = self._pose_stamped_data(message)
            self._condition.notify_all()

    def _captain_target_pose_callback(self, message: Any) -> None:
        with self._condition:
            self._captain_target_pose = self._pose_stamped_data(message)
            self._condition.notify_all()

    def _health_locked(self) -> Dict[str, Any]:
        now = time.monotonic()
        state = dict(self._state) if self._state is not None else {}
        extended = (
            dict(self._extended_state)
            if self._extended_state is not None
            else {}
        )
        battery = dict(self._battery) if self._battery is not None else {}

        state_ok = self._fresh(self._state, now, self.STATE_MAX_AGE_S)
        extended_state_ok = self._fresh(
            self._extended_state,
            now,
            self.STATE_MAX_AGE_S,
        )
        odom_ok = self._fresh(self._odom, now, self.ODOM_MAX_AGE_S)
        rgb_ok = self._fresh(self._rgb, now, self.RGB_MAX_AGE_S)
        battery_ok = self._fresh(self._battery, now, self.BATTERY_MAX_AGE_S)
        control_link_ok = self._publisher_connected(self._control_pub)
        goal_link_ok = self._publisher_connected(self._goal_pub)
        yaw_link_ok = self._publisher_connected(self._yaw_target_pub)
        connected = bool(state.get("connected", False))
        initialized = bool(
            self._started
            and state_ok
            and extended_state_ok
            and odom_ok
            and rgb_ok
            and control_link_ok
            and goal_link_ok
            and yaw_link_ok
            and connected
        )
        landed_state = extended.get("landed_state")
        airborne = bool(
            initialized
            and state.get("armed") is True
            and landed_state == self.LANDED_STATE_IN_AIR
        )
        mode = str(state.get("mode") or "")
        control_ready = bool(
            initialized
            and airborne
            and mode.upper() == "OFFBOARD"
        )
        missing = []
        for name, ready in (
            ("ros", self._started),
            ("mavros_state", state_ok),
            ("mavros_connected", connected),
            ("extended_state", extended_state_ok),
            ("odom", odom_ok),
            ("rgb", rgb_ok),
            ("captain_control_link", control_link_ok),
            ("captain_goal_link", goal_link_ok),
            ("yaw_target_link", yaw_link_ok),
        ):
            if not ready:
                missing.append(name)
        return {
            "ros_initialized": self._started,
            "initialized": initialized,
            "airborne": airborne,
            "offboard": mode.upper() == "OFFBOARD",
            "control_ready": control_ready,
            "connected": connected,
            "armed": bool(state.get("armed", False)),
            "mode": mode,
            "landed_state": landed_state,
            "state_ok": state_ok,
            "extended_state_ok": extended_state_ok,
            "odom_ok": odom_ok,
            "rgb_ok": rgb_ok,
            "control_link_ok": control_link_ok,
            "goal_link_ok": goal_link_ok,
            "yaw_link_ok": yaw_link_ok,
            "battery_ok": battery_ok,
            "battery_percentage": battery.get("percentage"),
            "battery_voltage": battery.get("voltage"),
            "missing": missing,
        }

    @staticmethod
    def _publisher_connected(publisher: Any) -> bool:
        if publisher is None:
            return False
        try:
            return int(publisher.get_num_connections()) > 0
        except Exception:
            return False

    @staticmethod
    def _pose_stamped_data(message: Any) -> Dict[str, Any]:
        pose = message.pose
        return {
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "z_m": float(pose.position.z),
            "frame_id": str(message.header.frame_id),
            "received_monotonic": time.monotonic(),
        }

    @staticmethod
    def _goal_matches(
        goal: Optional[Dict[str, Any]],
        x_m: float,
        y_m: float,
        z_m: float,
        after_monotonic: float,
        tolerance_m: float,
    ) -> bool:
        if goal is None:
            return False
        if float(goal.get("received_monotonic", 0.0)) < float(after_monotonic):
            return False
        error_m = math.sqrt(
            (float(goal["x_m"]) - float(x_m)) ** 2
            + (float(goal["y_m"]) - float(y_m)) ** 2
            + (float(goal["z_m"]) - float(z_m)) ** 2
        )
        return error_m <= max(0.0, float(tolerance_m))

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
