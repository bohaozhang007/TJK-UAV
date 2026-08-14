#!/usr/bin/env python3
"""Pure interactive EGO/MAVROS bridge for the I7 aircraft.

This node intentionally does not load a waypoint YAML.  Mission waypoints
belong to TJK-Agent.  The node owns only the real-time flight-control state:
manual OFFBOARD takeoff, one interactive goal, hold, takeover latching, abort,
and landing.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger, TriggerResponse


class NavState(str, Enum):
    IDLE = "IDLE"
    WAIT_OFFBOARD = "WAIT_OFFBOARD"
    TAKEOFF = "TAKEOFF"
    HOLD = "HOLD"
    NAVIGATING = "NAVIGATING"
    LANDING = "LANDING"
    MANUAL_TAKEOVER = "MANUAL_TAKEOVER"
    ERROR = "ERROR"


@dataclass
class PoseData:
    x: float
    y: float
    z: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    frame_id: str = "camera_init"
    received_monotonic: float = 0.0


@dataclass
class GoalData:
    x: float
    y: float
    z: float
    yaw: float
    frame_id: str
    received_monotonic: float
    uses_planner: bool
    planner_goal_published_monotonic: float = 0.0
    planner_output_ready: bool = False


class I7NavNode:
    ON_GROUND = 1
    IN_AIR = 2

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)

        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.goal_topic = rospy.get_param("~goal_topic", "/cxr_goal")
        self.planner_goal_topic = rospy.get_param(
            "~planner_goal_topic", "/move_base_simple/goal"
        )
        self.position_cmd_topic = rospy.get_param(
            "~position_cmd_topic", "/position_cmd"
        )
        self.setpoint_topic = rospy.get_param(
            "~setpoint_topic", "/mavros/setpoint_raw/local"
        )
        self.planner_heartbeat_topic = rospy.get_param(
            "~planner_heartbeat_topic", "/drone_0_traj_server/heartbeat"
        )

        self.control_hz = max(20.0, float(rospy.get_param("~control_hz", 30.0)))
        self.status_hz = max(1.0, float(rospy.get_param("~status_hz", 5.0)))
        self.takeoff_alt_m = float(rospy.get_param("~takeoff_alt_m", 1.0))
        self.max_takeoff_vel_m_s = max(
            0.1, float(rospy.get_param("~max_takeoff_vel_m_s", 0.8))
        )
        self.max_yaw_rate_rad_s = math.radians(
            max(1.0, float(rospy.get_param("~max_yaw_rate_deg_s", 60.0)))
        )
        self.min_height_m = float(rospy.get_param("~min_height_m", 0.5))
        self.goal_reached_distance_m = max(
            0.02, float(rospy.get_param("~goal_reached_distance_m", 0.15))
        )
        self.goal_yaw_tolerance_rad = math.radians(
            max(0.5, float(rospy.get_param("~goal_yaw_tolerance_deg", 5.0)))
        )
        self.stable_speed_m_s = max(
            0.01, float(rospy.get_param("~stable_speed_m_s", 0.20))
        )
        self.stable_samples_required = max(
            1, int(rospy.get_param("~stable_samples", 3))
        )
        self.takeoff_timeout_s = max(
            5.0, float(rospy.get_param("~takeoff_timeout_s", 120.0))
        )
        self.landing_timeout_s = max(
            10.0, float(rospy.get_param("~landing_timeout_s", 90.0))
        )
        self.odom_max_age_s = max(
            0.1, float(rospy.get_param("~odom_max_age_s", 1.0))
        )
        self.state_max_age_s = max(
            0.1, float(rospy.get_param("~state_max_age_s", 2.0))
        )
        self.position_cmd_max_age_s = max(
            0.05, float(rospy.get_param("~position_cmd_max_age_s", 0.5))
        )
        self.planner_heartbeat_max_age_s = max(
            0.2, float(rospy.get_param("~planner_heartbeat_max_age_s", 2.0))
        )
        self.direct_goal_max_distance_m = max(
            0.01, float(rospy.get_param("~direct_goal_max_distance_m", 0.15))
        )

        if self.takeoff_alt_m < self.min_height_m:
            raise ValueError("takeoff_alt_m must not be below min_height_m")

        self._state = NavState.IDLE
        self._state_changed_monotonic = time.monotonic()
        self._last_error: Optional[str] = None
        self._manual_takeover_latched = False
        self._control_session_active = False
        self._ground_z: Optional[float] = None
        self._odom: Optional[PoseData] = None
        self._hold: Optional[PoseData] = None
        self._goal: Optional[GoalData] = None
        self._position_cmd: Optional[PositionCommand] = None
        self._position_cmd_received_monotonic = 0.0
        self._planner_heartbeat_received_monotonic = 0.0
        self._mavros_state: Optional[dict[str, Any]] = None
        self._extended_state: Optional[dict[str, Any]] = None
        self._battery: Optional[dict[str, Any]] = None
        self._takeoff_command_z: Optional[float] = None
        self._commanded_yaw: Optional[float] = None
        self._goal_stable_samples = 0
        self._takeoff_stable_samples = 0
        self._last_control_monotonic = time.monotonic()
        self._last_status_monotonic = 0.0

        self._setpoint_pub = rospy.Publisher(
            self.setpoint_topic, PositionTarget, queue_size=20
        )
        self._planner_goal_pub = rospy.Publisher(
            self.planner_goal_topic, PoseStamped, queue_size=10
        )
        self._state_pub = rospy.Publisher(
            "~state", String, queue_size=10, latch=True
        )
        self._active_goal_pub = rospy.Publisher(
            "~active_goal", PoseStamped, queue_size=10, latch=True
        )

        self._subscribers = [
            rospy.Subscriber(
                self.odom_topic, Odometry, self._odom_callback, queue_size=20
            ),
            rospy.Subscriber(
                "/mavros/state", State, self._mavros_state_callback, queue_size=20
            ),
            rospy.Subscriber(
                "/mavros/extended_state",
                ExtendedState,
                self._extended_state_callback,
                queue_size=20,
            ),
            rospy.Subscriber(
                "/mavros/battery",
                BatteryState,
                self._battery_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                self.position_cmd_topic,
                PositionCommand,
                self._position_cmd_callback,
                queue_size=20,
            ),
            rospy.Subscriber(
                self.planner_heartbeat_topic,
                Empty,
                self._planner_heartbeat_callback,
                queue_size=20,
            ),
            rospy.Subscriber(
                self.goal_topic, PoseStamped, self._goal_callback, queue_size=10
            ),
        ]

        self._arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self._land_client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)

        self._services = [
            rospy.Service("~takeoff", Trigger, self._takeoff_service),
            rospy.Service("~land", Trigger, self._land_service),
            rospy.Service("~force_land", Trigger, self._force_land_service),
            rospy.Service("~abort", Trigger, self._abort_service),
            rospy.Service("~reinitialize", Trigger, self._reinitialize_service),
        ]

        self._control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_hz), self._control_timer_callback
        )
        self._publish_status(force=True)
        rospy.loginfo(
            "I7 interactive navigation ready: odom=%s goal=%s setpoint=%s",
            self.odom_topic,
            self.goal_topic,
            self.setpoint_topic,
        )

    def _odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        yaw = self._quaternion_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        value = PoseData(
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
            yaw=yaw,
            vx=float(twist.linear.x),
            vy=float(twist.linear.y),
            vz=float(twist.linear.z),
            frame_id=str(message.header.frame_id or "camera_init"),
            received_monotonic=time.monotonic(),
        )
        with self._condition:
            self._odom = value
            if self._commanded_yaw is None:
                self._commanded_yaw = value.yaw
            self._condition.notify_all()

    def _mavros_state_callback(self, message: State) -> None:
        now = time.monotonic()
        with self._condition:
            old_mode = str((self._mavros_state or {}).get("mode") or "")
            self._mavros_state = {
                "connected": bool(message.connected),
                "armed": bool(message.armed),
                "mode": str(message.mode),
                "received_monotonic": now,
            }
            new_mode = str(message.mode).upper()
            if (
                self._control_session_active
                and self._state != NavState.LANDING
                and old_mode.upper() == "OFFBOARD"
                and new_mode != "OFFBOARD"
                and bool(message.armed)
            ):
                self._latch_manual_takeover_locked(
                    f"flight mode changed from OFFBOARD to {message.mode!r}"
                )
            self._condition.notify_all()

    def _extended_state_callback(self, message: ExtendedState) -> None:
        with self._condition:
            self._extended_state = {
                "landed_state": int(message.landed_state),
                "vtol_state": int(message.vtol_state),
                "received_monotonic": time.monotonic(),
            }
            self._condition.notify_all()

    def _battery_callback(self, message: BatteryState) -> None:
        percentage = float(message.percentage)
        voltage = float(message.voltage)
        with self._condition:
            self._battery = {
                "percentage": percentage if math.isfinite(percentage) else None,
                "voltage": voltage if math.isfinite(voltage) else None,
                "received_monotonic": time.monotonic(),
            }

    def _position_cmd_callback(self, message: PositionCommand) -> None:
        with self._condition:
            self._position_cmd = message
            self._position_cmd_received_monotonic = time.monotonic()
            if (
                self._goal is not None
                and self._goal.uses_planner
                and not self._goal.planner_output_ready
                and self._goal.planner_goal_published_monotonic > 0.0
                and self._position_cmd_received_monotonic
                > self._goal.planner_goal_published_monotonic
            ):
                self._goal.planner_output_ready = True
                rospy.loginfo(
                    "EGO output accepted for active goal before MAVROS "
                    "trajectory forwarding"
                )
            self._condition.notify_all()

    def _planner_heartbeat_callback(self, _message: Empty) -> None:
        with self._condition:
            self._planner_heartbeat_received_monotonic = time.monotonic()
            self._condition.notify_all()

    def _goal_callback(self, message: PoseStamped) -> None:
        values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            message.pose.orientation.x,
            message.pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            rospy.logerr("Reject non-finite I7 goal: %s", values)
            return

        with self._condition:
            if self._manual_takeover_latched:
                rospy.logwarn("Reject I7 goal while manual takeover is latched")
                return
            if not self._control_ready_locked():
                rospy.logwarn("Reject I7 goal because OFFBOARD control is not ready")
                return
            if self._state not in {NavState.HOLD, NavState.NAVIGATING}:
                rospy.logwarn("Reject I7 goal in navigation state %s", self._state.value)
                return
            if self._ground_z is None:
                rospy.logerr("Reject I7 goal because ground height is unavailable")
                return
            if self._odom is None:
                rospy.logerr("Reject I7 goal because odometry is unavailable")
                return

            target_z = float(message.pose.position.z)
            min_z = self._ground_z + self.min_height_m
            if target_z < min_z:
                rospy.logerr(
                    "Reject I7 goal z=%.3f below minimum %.3f",
                    target_z,
                    min_z,
                )
                return

            requested_direct_control = float(message.pose.orientation.x) > 0.5
            target_distance_m = math.sqrt(
                (float(message.pose.position.x) - self._odom.x) ** 2
                + (float(message.pose.position.y) - self._odom.y) ** 2
                + (target_z - self._odom.z) ** 2
            )
            if (
                requested_direct_control
                and target_distance_m > self.direct_goal_max_distance_m
            ):
                self._last_error = (
                    "direct yaw goal contains translation "
                    f"{target_distance_m:.3f}m > {self.direct_goal_max_distance_m:.3f}m"
                )
                rospy.logerr("Reject I7 goal: %s", self._last_error)
                return

            uses_planner = not requested_direct_control
            if uses_planner and not self._planner_ok_locked():
                self._last_error = (
                    f"EGO planner heartbeat is missing or stale on "
                    f"{self.planner_heartbeat_topic}"
                )
                rospy.logerr("Reject I7 goal: %s", self._last_error)
                return

            goal = GoalData(
                x=float(message.pose.position.x),
                y=float(message.pose.position.y),
                z=target_z,
                yaw=self._normalize_angle(math.radians(float(message.pose.orientation.w))),
                frame_id=str(
                    message.header.frame_id
                    or (self._odom.frame_id if self._odom is not None else "camera_init")
                ),
                received_monotonic=time.monotonic(),
                uses_planner=uses_planner,
            )
            self._goal = goal
            # Do not allow a still-fresh command from the previous trajectory
            # to run before EGO publishes the command for this new goal.
            self._position_cmd = None
            self._position_cmd_received_monotonic = 0.0
            self._goal_stable_samples = 0
            self._last_error = None
            self._set_state_locked(NavState.NAVIGATING)
            self._condition.notify_all()

        if goal.uses_planner:
            planner_goal = self._goal_pose_message(goal, standard_quaternion=True)
            self._planner_goal_pub.publish(planner_goal)
            with self._condition:
                if self._goal is goal:
                    # Drop anything received between accepting the Robot goal
                    # and publishing its EGO goal. The next PositionCommand is
                    # therefore downstream of this planner request.
                    goal.planner_goal_published_monotonic = time.monotonic()
                    self._position_cmd = None
                    self._position_cmd_received_monotonic = 0.0
                    self._condition.notify_all()
        self._active_goal_pub.publish(self._goal_pose_message(goal))
        rospy.loginfo(
            "Accepted I7 %s goal: x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
            "planner" if goal.uses_planner else "direct-yaw",
            goal.x,
            goal.y,
            goal.z,
            math.degrees(goal.yaw),
        )

    def _control_timer_callback(self, _event: rospy.TimerEvent) -> None:
        now = time.monotonic()
        dt = max(0.0, min(0.2, now - self._last_control_monotonic))
        self._last_control_monotonic = now
        setpoint: Optional[PositionTarget] = None
        with self._condition:
            actively_controlling = self._state in {
                NavState.WAIT_OFFBOARD,
                NavState.TAKEOFF,
                NavState.HOLD,
                NavState.NAVIGATING,
            }
            telemetry_error = self._telemetry_error_locked(now)
            if actively_controlling and telemetry_error is not None:
                self._goal = None
                self._last_error = telemetry_error
                self._set_state_locked(NavState.ERROR)
                self._condition.notify_all()
                rospy.logerr("I7 stopped publishing setpoints: %s", telemetry_error)
            elif self._state == NavState.TAKEOFF:
                setpoint = self._takeoff_setpoint_locked(dt)
                self._update_takeoff_completion_locked()
            elif self._state == NavState.NAVIGATING:
                if (
                    self._goal is not None
                    and self._goal.uses_planner
                    and not self._planner_ok_locked(now)
                ):
                    self._last_error = "EGO planner heartbeat became stale"
                    self._goal = None
                    if self._odom is not None:
                        self._hold = self._copy_pose(self._odom)
                    self._set_state_locked(NavState.HOLD)
                    self._condition.notify_all()
                    rospy.logerr("I7 navigation stopped: %s", self._last_error)
                    setpoint = self._hold_setpoint_locked(dt)
                else:
                    setpoint = self._navigation_setpoint_locked(dt, now)
                    self._update_goal_completion_locked()
            elif self._state in {NavState.WAIT_OFFBOARD, NavState.HOLD}:
                setpoint = self._hold_setpoint_locked(dt)

        if setpoint is not None:
            self._setpoint_pub.publish(setpoint)
        if now - self._last_status_monotonic >= 1.0 / self.status_hz:
            self._publish_status()

    def _takeoff_setpoint_locked(self, dt: float) -> Optional[PositionTarget]:
        if self._hold is None or self._ground_z is None:
            return None
        target_z = self._ground_z + self.takeoff_alt_m
        current_command_z = (
            self._takeoff_command_z
            if self._takeoff_command_z is not None
            else self._hold.z
        )
        max_step = self.max_takeoff_vel_m_s * dt
        current_command_z = min(target_z, current_command_z + max_step)
        self._takeoff_command_z = current_command_z
        target = PoseData(
            x=self._hold.x,
            y=self._hold.y,
            z=current_command_z,
            yaw=self._hold.yaw,
            frame_id=self._hold.frame_id,
        )
        return self._pose_setpoint_locked(target, dt)

    def _navigation_setpoint_locked(
        self, dt: float, now: float
    ) -> Optional[PositionTarget]:
        goal = self._goal
        odom = self._odom
        command_fresh = (
            self._position_cmd is not None
            and now - self._position_cmd_received_monotonic
            <= self.position_cmd_max_age_s
        )
        if goal is None or odom is None:
            return self._hold_setpoint_locked(dt)
        if not goal.uses_planner:
            final_target = PoseData(
                x=goal.x,
                y=goal.y,
                z=goal.z,
                yaw=goal.yaw,
                frame_id=goal.frame_id,
            )
            return self._pose_setpoint_locked(final_target, dt)

        command = self._position_cmd
        if not goal.planner_output_ready or command is None:
            # EGO needs a short planning interval after receiving a new goal.
            # Only a live-pose hold setpoint is sent to MAVROS during this
            # interval; no motion trajectory is forwarded until a new EGO
            # PositionCommand has been accepted for the active goal.
            return self._pose_setpoint_locked(odom, dt)
        command_values = (
            command.position.x,
            command.position.y,
            command.position.z,
            command.velocity.x,
            command.velocity.y,
            command.velocity.z,
            command.acceleration.x,
            command.acceleration.y,
            command.acceleration.z,
        )
        if not all(math.isfinite(float(value)) for value in command_values):
            self._last_error = "planner emitted a non-finite position command"
            return self._pose_setpoint_locked(odom, dt)
        # EGO owns only the collision-free xyz trajectory.  Its traj_server
        # points yaw along the direction of travel, which makes a backwards
        # move rotate the aircraft by about 180 degrees.  Keep I7 yaw tied to
        # the Robot command, just as Owl's independent yaw controller does.
        requested_yaw = goal.yaw

        if not command_fresh:
            # This traj_server stops publishing when the polynomial duration
            # expires.  Keep its last (normally final) collision-free setpoint;
            # _hold still refers to the pose before this goal and would command
            # the aircraft to fly back to its starting point.
            last_planned_target = PoseData(
                x=float(command.position.x),
                y=float(command.position.y),
                z=float(command.position.z),
                yaw=requested_yaw,
                frame_id=str(command.header.frame_id or goal.frame_id),
            )
            return self._pose_setpoint_locked(last_planned_target, dt)

        yaw, _yaw_rate = self._limited_yaw_locked(requested_yaw, dt)

        message = PositionTarget()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = str(command.header.frame_id or goal.frame_id)
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Publish one yaw control variable.  Supplying yaw and yaw-rate
        # together can make PX4 fight two angular commands during translation.
        message.type_mask = PositionTarget.IGNORE_YAW_RATE
        message.position.x = float(command.position.x)
        message.position.y = float(command.position.y)
        message.position.z = float(command.position.z)
        message.velocity.x = float(command.velocity.x)
        message.velocity.y = float(command.velocity.y)
        message.velocity.z = float(command.velocity.z)
        message.acceleration_or_force.x = float(command.acceleration.x)
        message.acceleration_or_force.y = float(command.acceleration.y)
        message.acceleration_or_force.z = float(command.acceleration.z)
        message.yaw = yaw
        message.yaw_rate = 0.0
        return message

    def _hold_setpoint_locked(self, dt: float) -> Optional[PositionTarget]:
        target = self._hold or self._odom
        if target is None:
            return None
        return self._pose_setpoint_locked(target, dt)

    def _pose_setpoint_locked(
        self, target: PoseData, dt: float
    ) -> PositionTarget:
        yaw, _yaw_rate = self._limited_yaw_locked(target.yaw, dt)
        message = PositionTarget()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = target.frame_id
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = (
            PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        message.position.x = target.x
        message.position.y = target.y
        message.position.z = target.z
        message.yaw = yaw
        return message

    def _limited_yaw_locked(self, target_yaw: float, dt: float) -> tuple[float, float]:
        if self._commanded_yaw is None:
            self._commanded_yaw = (
                self._odom.yaw if self._odom is not None else target_yaw
            )
        error = self._normalize_angle(target_yaw - self._commanded_yaw)
        max_step = self.max_yaw_rate_rad_s * dt
        step = max(-max_step, min(max_step, error))
        self._commanded_yaw = self._normalize_angle(self._commanded_yaw + step)
        yaw_rate = 0.0 if dt <= 0.0 else step / dt
        return self._commanded_yaw, yaw_rate

    def _update_takeoff_completion_locked(self) -> None:
        if self._odom is None or self._ground_z is None:
            return
        target_z = self._ground_z + self.takeoff_alt_m
        height_ok = abs(self._odom.z - target_z) <= self.goal_reached_distance_m
        speed_ok = self._speed(self._odom) <= self.stable_speed_m_s
        airborne = int((self._extended_state or {}).get("landed_state", 0)) == self.IN_AIR
        if height_ok and speed_ok and airborne and self._control_ready_locked():
            self._takeoff_stable_samples += 1
        else:
            self._takeoff_stable_samples = 0
        if self._takeoff_stable_samples >= self.stable_samples_required:
            self._hold = PoseData(
                x=self._hold.x if self._hold is not None else self._odom.x,
                y=self._hold.y if self._hold is not None else self._odom.y,
                z=target_z,
                yaw=self._hold.yaw if self._hold is not None else self._odom.yaw,
                frame_id=self._odom.frame_id,
            )
            self._set_state_locked(NavState.HOLD)
            self._condition.notify_all()

    def _update_goal_completion_locked(self) -> None:
        if self._odom is None or self._goal is None:
            return
        if self._goal.uses_planner and not self._goal.planner_output_ready:
            self._goal_stable_samples = 0
            return
        position_ok = self._distance(self._odom, self._goal) <= self.goal_reached_distance_m
        speed_ok = self._speed(self._odom) <= self.stable_speed_m_s
        yaw_ok = abs(self._normalize_angle(self._goal.yaw - self._odom.yaw)) <= self.goal_yaw_tolerance_rad
        if position_ok and speed_ok and yaw_ok:
            self._goal_stable_samples += 1
        else:
            self._goal_stable_samples = 0
        if self._goal_stable_samples >= self.stable_samples_required:
            goal = self._goal
            self._hold = PoseData(
                x=goal.x,
                y=goal.y,
                z=goal.z,
                yaw=goal.yaw,
                frame_id=goal.frame_id,
            )
            self._goal = None
            self._set_state_locked(NavState.HOLD)
            self._condition.notify_all()

    def _takeoff_service(self, _request: Trigger.Request) -> TriggerResponse:
        try:
            return self._run_takeoff()
        except Exception as exc:
            rospy.logerr("I7 takeoff failed: %s", exc)
            with self._condition:
                self._last_error = str(exc)
                if self._control_ready_locked() and self._odom is not None:
                    self._hold = self._copy_pose(self._odom)
                    self._set_state_locked(NavState.HOLD)
                elif self._on_ground_disarmed_locked():
                    # A rejected pre-OFFBOARD takeoff request must remain safe to
                    # retry after the pilot switches back to POSITION.
                    self._set_state_locked(NavState.IDLE)
                elif not self._manual_takeover_latched:
                    self._set_state_locked(NavState.ERROR)
                self._condition.notify_all()
            return TriggerResponse(success=False, message=str(exc))

    def _run_takeoff(self) -> TriggerResponse:
        deadline = time.monotonic() + self.takeoff_timeout_s
        with self._condition:
            self._require_telemetry_locked()
            if self._manual_takeover_latched:
                raise RuntimeError(
                    "manual takeover is latched; run local console init first"
                )
            if self._odom is None:
                raise RuntimeError("odometry is unavailable")
            initial_mode = str((self._mavros_state or {}).get("mode") or "").upper()
            if initial_mode == "OFFBOARD":
                raise RuntimeError(
                    "takeoff must be requested before OFFBOARD; switch to POSITION, "
                    "call takeoff, then select OFFBOARD"
                )
            airborne = self._airborne_locked()

            if not airborne:
                landed_state = int(
                    (self._extended_state or {}).get("landed_state", 0)
                )
                if landed_state != self.ON_GROUND:
                    raise RuntimeError(
                        f"cannot take off with landed_state={landed_state}"
                    )
                self._ground_z = self._odom.z
            elif self._ground_z is None:
                self._ground_z = self._odom.z - self.takeoff_alt_m

            self._hold = self._copy_pose(self._odom)
            self._takeoff_command_z = self._odom.z
            self._takeoff_stable_samples = 0
            self._last_error = None
            self._set_state_locked(NavState.WAIT_OFFBOARD)
            self._condition.notify_all()

        rospy.loginfo("I7 takeoff is waiting for manual OFFBOARD selection")
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._condition:
                if self._manual_takeover_latched:
                    raise RuntimeError("manual takeover latched during takeoff")
                self._require_telemetry_locked()
                mode = str((self._mavros_state or {}).get("mode") or "").upper()
                armed = bool((self._mavros_state or {}).get("armed", False))
                airborne = self._airborne_locked()
                if mode != "OFFBOARD":
                    self._condition.wait(timeout=0.1)
                    continue
                self._control_session_active = True

            if airborne and armed:
                with self._condition:
                    assert self._odom is not None
                    self._hold = self._copy_pose(self._odom)
                    self._set_state_locked(NavState.HOLD)
                    self._condition.notify_all()
                return TriggerResponse(
                    success=True,
                    message="airborne OFFBOARD control resumed at current pose",
                )

            if not armed:
                rospy.wait_for_service("/mavros/cmd/arming", timeout=3.0)
                response = self._arm_client(True)
                if not bool(response.success):
                    raise RuntimeError(
                        f"PX4 arming rejected with result={response.result}"
                    )

            with self._condition:
                self._set_state_locked(NavState.TAKEOFF)
                self._condition.notify_all()
                while (
                    not rospy.is_shutdown()
                    and time.monotonic() < deadline
                    and self._state == NavState.TAKEOFF
                    and not self._manual_takeover_latched
                ):
                    self._condition.wait(timeout=0.1)
                if self._manual_takeover_latched:
                    raise RuntimeError("manual takeover latched during takeoff")
                if self._state == NavState.HOLD:
                    return TriggerResponse(
                        success=True, message="takeoff complete; holding"
                    )
                break

        with self._condition:
            if not bool((self._mavros_state or {}).get("armed", False)):
                self._set_state_locked(NavState.IDLE)
        raise RuntimeError("takeoff timed out")

    def _land_service(self, _request: Trigger.Request) -> TriggerResponse:
        return self._request_land(force=False)

    def _force_land_service(self, _request: Trigger.Request) -> TriggerResponse:
        return self._request_land(force=True)

    def _request_land(self, *, force: bool) -> TriggerResponse:
        try:
            with self._condition:
                self._require_telemetry_locked()
                if self._manual_takeover_latched and not force:
                    raise RuntimeError(
                        "manual takeover is latched; normal automatic landing is inhibited"
                    )
                if self._on_ground_disarmed_locked():
                    self._control_session_active = False
                    self._manual_takeover_latched = False
                    self._set_state_locked(NavState.IDLE)
                    return TriggerResponse(success=True, message="already landed")
                self._goal = None
                self._hold = self._copy_pose(self._odom) if self._odom else None
                self._set_state_locked(NavState.LANDING)
                self._condition.notify_all()

            rospy.wait_for_service("/mavros/cmd/land", timeout=3.0)
            response = self._land_client(0.0, 0.0, 0.0, 0.0, 0.0)
            if not bool(response.success):
                raise RuntimeError(
                    f"PX4 land command rejected with result={response.result}"
                )

            deadline = time.monotonic() + self.landing_timeout_s
            with self._condition:
                while not rospy.is_shutdown() and time.monotonic() < deadline:
                    if self._on_ground_disarmed_locked():
                        self._control_session_active = False
                        self._manual_takeover_latched = False
                        self._goal = None
                        self._set_state_locked(NavState.IDLE)
                        self._condition.notify_all()
                        return TriggerResponse(
                            success=True, message="landing complete; vehicle disarmed"
                        )
                    self._condition.wait(timeout=0.1)
            raise RuntimeError("landing timed out before ground/disarm confirmation")
        except Exception as exc:
            rospy.logerr("I7 landing failed: %s", exc)
            with self._condition:
                self._last_error = str(exc)
                if self._control_ready_locked() and self._odom is not None:
                    self._hold = self._copy_pose(self._odom)
                    self._set_state_locked(NavState.HOLD)
                elif not self._manual_takeover_latched:
                    self._set_state_locked(NavState.ERROR)
            return TriggerResponse(success=False, message=str(exc))

    def _abort_service(self, _request: Trigger.Request) -> TriggerResponse:
        with self._condition:
            if self._manual_takeover_latched:
                return TriggerResponse(
                    success=True,
                    message="manual takeover already owns control; navigation is stopped",
                )
            if self._odom is None:
                return TriggerResponse(success=False, message="odometry is unavailable")
            self._goal = None
            self._hold = self._copy_pose(self._odom)
            self._goal_stable_samples = 0
            self._set_state_locked(
                NavState.HOLD if self._control_ready_locked() else NavState.IDLE
            )
            self._condition.notify_all()
        return TriggerResponse(success=True, message="navigation aborted; holding")

    def _reinitialize_service(self, _request: Trigger.Request) -> TriggerResponse:
        with self._condition:
            self._require_telemetry_locked()
            mode = str((self._mavros_state or {}).get("mode") or "").upper()
            if mode == "OFFBOARD":
                return TriggerResponse(
                    success=False,
                    message="switch out of OFFBOARD before local reinitialization",
                )
            if self._odom is None:
                return TriggerResponse(success=False, message="odometry is unavailable")
            if not self._airborne_locked():
                self._ground_z = self._odom.z
            elif self._ground_z is None:
                self._ground_z = self._odom.z - self.takeoff_alt_m
            self._manual_takeover_latched = False
            self._control_session_active = False
            self._goal = None
            self._hold = self._copy_pose(self._odom)
            self._takeoff_command_z = None
            self._goal_stable_samples = 0
            self._takeoff_stable_samples = 0
            self._last_error = None
            self._commanded_yaw = self._odom.yaw
            self._set_state_locked(NavState.IDLE)
            self._condition.notify_all()
        return TriggerResponse(
            success=True,
            message="I7 navigation reinitialized; call takeoff before OFFBOARD",
        )

    def _latch_manual_takeover_locked(self, reason: str) -> None:
        self._manual_takeover_latched = True
        self._goal = None
        self._last_error = reason
        if self._odom is not None:
            self._hold = self._copy_pose(self._odom)
        self._set_state_locked(NavState.MANUAL_TAKEOVER)
        rospy.logwarn("I7 manual takeover latched: %s", reason)

    def _planner_ok_locked(self, now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else float(now)
        return bool(
            self._planner_heartbeat_received_monotonic > 0.0
            and current - self._planner_heartbeat_received_monotonic
            <= self.planner_heartbeat_max_age_s
        )

    def _publish_status(self, *, force: bool = False) -> None:
        now = time.monotonic()
        with self._lock:
            if not force and now - self._last_status_monotonic < 1.0 / self.status_hz:
                return
            self._last_status_monotonic = now
            state = dict(self._mavros_state or {})
            extended = dict(self._extended_state or {})
            battery = dict(self._battery or {})
            odom_fresh = self._fresh(self._odom, now, self.odom_max_age_s)
            state_fresh = self._fresh_mapping(
                self._mavros_state, now, self.state_max_age_s
            )
            extended_fresh = self._fresh_mapping(
                self._extended_state, now, self.state_max_age_s
            )
            payload = {
                "state": self._state.value,
                "manual_takeover_latched": self._manual_takeover_latched,
                "control_session_active": self._control_session_active,
                "control_ready": self._control_ready_locked(),
                "planner_ok": self._planner_ok_locked(now),
                "planner_heartbeat_topic": self.planner_heartbeat_topic,
                "odom_ok": odom_fresh,
                "state_ok": state_fresh,
                "extended_state_ok": extended_fresh,
                "connected": bool(state.get("connected", False)),
                "armed": bool(state.get("armed", False)),
                "mode": str(state.get("mode") or ""),
                "landed_state": extended.get("landed_state"),
                "airborne": self._airborne_locked(),
                "ground_z_m": self._ground_z,
                "min_height_m": self.min_height_m,
                "max_height_m": None,
                "takeoff_alt_m": self.takeoff_alt_m,
                "active_goal": self._goal_dict(self._goal),
                "last_error": self._last_error,
                "battery_percentage": battery.get("percentage"),
                "battery_voltage": battery.get("voltage"),
                "stamp": rospy.Time.now().to_sec(),
            }
        self._state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _require_telemetry_locked(self) -> None:
        now = time.monotonic()
        error = self._telemetry_error_locked(now)
        if error is not None:
            raise RuntimeError(error)

    def _telemetry_error_locked(self, now: Optional[float] = None) -> Optional[str]:
        current = time.monotonic() if now is None else float(now)
        missing = []
        if not self._fresh(self._odom, current, self.odom_max_age_s):
            missing.append(self.odom_topic)
        if not self._fresh_mapping(
            self._mavros_state, current, self.state_max_age_s
        ):
            missing.append("/mavros/state")
        if not self._fresh_mapping(
            self._extended_state, current, self.state_max_age_s
        ):
            missing.append("/mavros/extended_state")
        if not bool((self._mavros_state or {}).get("connected", False)):
            missing.append("mavros_connected")
        if missing:
            return "missing or stale flight telemetry: " + ", ".join(missing)
        return None

    def _control_ready_locked(self) -> bool:
        state = self._mavros_state or {}
        return bool(
            not self._manual_takeover_latched
            and self._telemetry_error_locked() is None
            and state.get("connected") is True
            and state.get("armed") is True
            and str(state.get("mode") or "").upper() == "OFFBOARD"
            and self._airborne_locked()
        )

    def _airborne_locked(self) -> bool:
        state = self._mavros_state or {}
        extended = self._extended_state or {}
        return bool(
            state.get("armed") is True
            and int(extended.get("landed_state", 0)) == self.IN_AIR
        )

    def _on_ground_disarmed_locked(self) -> bool:
        state = self._mavros_state or {}
        extended = self._extended_state or {}
        return bool(
            state.get("armed") is False
            and int(extended.get("landed_state", 0)) == self.ON_GROUND
        )

    def _set_state_locked(self, state: NavState) -> None:
        if self._state == state:
            return
        rospy.loginfo("I7 navigation state: %s -> %s", self._state.value, state.value)
        self._state = state
        self._state_changed_monotonic = time.monotonic()

    def _goal_pose_message(
        self, goal: GoalData, *, standard_quaternion: bool = False
    ) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = goal.frame_id
        message.pose.position.x = goal.x
        message.pose.position.y = goal.y
        message.pose.position.z = goal.z
        if standard_quaternion:
            message.pose.orientation.z = math.sin(goal.yaw / 2.0)
            message.pose.orientation.w = math.cos(goal.yaw / 2.0)
        else:
            message.pose.orientation.x = 0.0 if goal.uses_planner else 1.0
            message.pose.orientation.w = math.degrees(goal.yaw)
        return message

    @staticmethod
    def _copy_pose(pose: PoseData) -> PoseData:
        return PoseData(
            x=pose.x,
            y=pose.y,
            z=pose.z,
            yaw=pose.yaw,
            vx=pose.vx,
            vy=pose.vy,
            vz=pose.vz,
            frame_id=pose.frame_id,
            received_monotonic=pose.received_monotonic,
        )

    @staticmethod
    def _goal_dict(goal: Optional[GoalData]) -> Optional[dict[str, Any]]:
        if goal is None:
            return None
        return {
            "x_m": goal.x,
            "y_m": goal.y,
            "z_m": goal.z,
            "yaw_deg": math.degrees(goal.yaw),
            "uses_planner": goal.uses_planner,
            "planner_goal_published": (
                goal.planner_goal_published_monotonic > 0.0
            ),
            "planner_output_ready": goal.planner_output_ready,
        }

    @staticmethod
    def _distance(pose: PoseData, goal: GoalData) -> float:
        return math.sqrt(
            (pose.x - goal.x) ** 2
            + (pose.y - goal.y) ** 2
            + (pose.z - goal.z) ** 2
        )

    @staticmethod
    def _speed(pose: PoseData) -> float:
        return math.sqrt(pose.vx**2 + pose.vy**2 + pose.vz**2)

    @staticmethod
    def _fresh(value: Optional[PoseData], now: float, max_age_s: float) -> bool:
        return bool(
            value is not None
            and now - float(value.received_monotonic) <= float(max_age_s)
        )

    @staticmethod
    def _fresh_mapping(
        value: Optional[dict[str, Any]], now: float, max_age_s: float
    ) -> bool:
        return bool(
            value is not None
            and now - float(value.get("received_monotonic", 0.0))
            <= float(max_age_s)
        )

    @staticmethod
    def _quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _normalize_angle(value: float) -> float:
        return math.atan2(math.sin(float(value)), math.cos(float(value)))


def main() -> None:
    rospy.init_node("i7_nav", anonymous=False)
    I7NavNode()
    rospy.spin()


if __name__ == "__main__":
    main()
