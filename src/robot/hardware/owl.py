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

from ..config_loader import (
    load_robot_config,
    required_number,
    required_section,
    required_string,
)


class OwlHardware:
    """Thin ROS1 adapter for the topics exposed by the OWL flight stack."""

    LANDED_STATE_ON_GROUND = 1
    LANDED_STATE_IN_AIR = 2

    def __init__(
        self,
        *,
        node_name: Optional[str] = None,
        config: Optional[dict] = None,
        config_path: Optional[str] = None,
    ) -> None:
        config = config or load_robot_config("owl", config_path)
        hardware_config = required_section(config, "hardware")
        topics = required_section(config, "topics")
        self.node_name = node_name or required_string(hardware_config, "node_name")
        self.odom_topic = required_string(topics, "odom")
        self.rgb_topic = required_string(topics, "rgb")
        self.state_topic = required_string(topics, "state")
        self.extended_state_topic = required_string(topics, "extended_state")
        self.battery_topic = required_string(topics, "battery")
        self.control_topic = required_string(topics, "control")
        self.goal_topic = required_string(topics, "goal")
        self.yaw_target_topic = required_string(topics, "yaw_target")
        self.planning_goal_topic = required_string(topics, "planning_goal")
        self.captain_target_pose_topic = required_string(
            topics, "captain_target_pose"
        )
        self.ready_timeout_s = required_number(
            hardware_config, "ready_timeout_s", minimum=1e-6
        )
        self.state_max_age_s = required_number(
            hardware_config, "state_max_age_s", minimum=1e-6
        )
        self.odom_max_age_s = required_number(
            hardware_config, "odom_max_age_s", minimum=1e-6
        )
        self.rgb_max_age_s = required_number(
            hardware_config, "rgb_max_age_s", minimum=1e-6
        )
        self.battery_max_age_s = required_number(
            hardware_config, "battery_max_age_s", minimum=1e-6
        )
        self.condition_wait_timeout_s = required_number(
            hardware_config, "condition_wait_timeout_s", minimum=1e-6
        )
        self.goal_ack_position_tolerance_m = required_number(
            hardware_config,
            "goal_ack_position_tolerance_m",
            minimum=0.0,
        )
        self.control_queue_size = required_number(
            hardware_config, "control_queue_size", integer=True, minimum=1
        )
        self.camera_queue_size = required_number(
            hardware_config, "camera_queue_size", integer=True, minimum=1
        )
        self.camera_buffer_bytes = required_number(
            hardware_config, "camera_buffer_bytes", integer=True, minimum=1
        )
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

    def connect(self, timeout_s: Optional[float] = None) -> Dict[str, Any]:
        """Initialize ROS and wait until telemetry and Captain links are ready."""
        timeout_s = self.ready_timeout_s if timeout_s is None else float(timeout_s)
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
                self._condition.wait(
                    timeout=min(self.condition_wait_timeout_s, remaining_s)
                )

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
            self.control_topic,
            Control,
            queue_size=self.control_queue_size,
        )
        self._goal_pub = rospy.Publisher(
            self.goal_topic,
            PoseStamped,
            queue_size=self.control_queue_size,
        )
        self._yaw_target_pub = rospy.Publisher(
            self.yaw_target_topic,
            YawTarget,
            queue_size=self.control_queue_size,
        )
        self._publishers = [
            self._control_pub,
            self._goal_pub,
            self._yaw_target_pub,
        ]
        self._subscribers = [
            rospy.Subscriber(
                self.odom_topic,
                Odometry,
                self._odom_callback,
                queue_size=self.control_queue_size,
            ),
            rospy.Subscriber(
                self.rgb_topic,
                CompressedImage,
                self._rgb_callback,
                queue_size=self.camera_queue_size,
                buff_size=self.camera_buffer_bytes,
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
                queue_size=self.control_queue_size,
            ),
            rospy.Subscriber(
                self.planning_goal_topic,
                PoseStamped,
                self._planning_goal_callback,
                queue_size=self.control_queue_size,
            ),
            rospy.Subscriber(
                self.captain_target_pose_topic,
                PoseStamped,
                self._captain_target_pose_callback,
                queue_size=self.control_queue_size,
            ),
        ]
        self._started = True

    def publish_control(self, cmd: int, drone_id: int) -> None:
        with self._lock:
            self._require_started()
            if not self._publisher_connected(self._control_pub):
                raise RuntimeError(
                    f"Captain is not subscribed to {self.control_topic}"
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
                    f"Captain is not subscribed to {self.goal_topic}"
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
                    f"{self.yaw_target_topic}"
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
    ) -> Dict[str, Any]:
        """Wait until Captain echoes a matching goal toward its planner."""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while True:
                for source, goal in (
                    (self.planning_goal_topic, self._planning_goal),
                    (self.captain_target_pose_topic, self._captain_target_pose),
                ):
                    if self._goal_matches(
                        goal,
                        x_m,
                        y_m,
                        z_m,
                        after_monotonic,
                        self.goal_ack_position_tolerance_m,
                    ):
                        result = dict(goal or {})
                        result["source"] = source
                        return result

                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0:
                    raise RuntimeError(
                        "Captain did not acknowledge goal on "
                        f"{self.planning_goal_topic} or "
                        f"{self.captain_target_pose_topic}"
                    )
                self._condition.wait(
                    timeout=min(self.condition_wait_timeout_s, remaining_s)
                )

    def get_pose_ros(self) -> Dict[str, Any]:
        with self._lock:
            if self._odom is None:
                raise RuntimeError(f"no odometry received from {self.odom_topic}")
            if not self._fresh(
                self._odom,
                time.monotonic(),
                self.odom_max_age_s,
            ):
                raise RuntimeError(f"stale odometry from {self.odom_topic}")
            return dict(self._odom)

    def get_compressed_rgb(self) -> Tuple[bytes, str]:
        with self._lock:
            if self._rgb is None:
                raise RuntimeError(f"no image received from {self.rgb_topic}")
            if not self._fresh(
                self._rgb,
                time.monotonic(),
                self.rgb_max_age_s,
            ):
                raise RuntimeError(f"stale image from {self.rgb_topic}")
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

        state_ok = self._fresh(self._state, now, self.state_max_age_s)
        extended_state_ok = self._fresh(
            self._extended_state,
            now,
            self.state_max_age_s,
        )
        odom_ok = self._fresh(self._odom, now, self.odom_max_age_s)
        rgb_ok = self._fresh(self._rgb, now, self.rgb_max_age_s)
        battery_ok = self._fresh(self._battery, now, self.battery_max_age_s)
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
