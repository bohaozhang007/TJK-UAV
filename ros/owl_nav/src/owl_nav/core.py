"""ROS-independent flight state machine. Caller serializes calls with a short lock.

Only tick() produces setpoints. Ownership is a UUID namespace per planner process,
not a receive timestamp. All clocks for watchdogs are monotonic.
"""
import math
import uuid
from collections import deque
import numpy as np


class Rejected(ValueError):
    pass


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def finite(values):
    return all(isinstance(v, (float, int)) and not isinstance(v, bool)
               and math.isfinite(v) for v in values)


class Polynomial:
    def __init__(self, msg):
        self.start = msg.start_time.to_sec()
        self.id = msg.traj_id
        self.durations = np.asarray(msg.duration, dtype=float)
        self.coeffs = np.asarray([msg.coef_x, msg.coef_y, msg.coef_z], dtype=float)
        n = len(self.durations)
        if (msg.order != 5 or not n or self.coeffs.shape != (3, n * 6)
                or not np.isfinite(self.coeffs).all()
                or not np.isfinite(self.durations).all()
                or (self.durations <= 0).any() or not math.isfinite(self.start)):
            raise Rejected('invalid polynomial')
        self.duration = float(sum(self.durations))

    def sample(self, stamp):
        t = max(0., min(stamp - self.start, self.duration))
        i = 0
        while i < len(self.durations) - 1 and t > self.durations[i]:
            t -= self.durations[i]
            i += 1
        c = self.coeffs[:, i * 6:(i + 1) * 6]
        return tuple(np.array([np.polyval(np.polyder(axis, k), t) for axis in c])
                     for k in range(3))


class FlightCore:
    def __init__(self, config, *, global_z_enabled=False, vertical_tolerance_enabled=False):
        config = dict(config)
        config.pop('global_z_enabled', None)
        self.motion_options = dict(global_z_enabled=global_z_enabled, vertical_tolerance_enabled=vertical_tolerance_enabled)
        if any(type(v) is not bool for v in self.motion_options.values()):
            raise ValueError("Z options must be booleans")
        # Retired setting: old deployment YAML must not restore the Z-only gate.
        config.pop('vertical_tolerance_m', None)
        for key,value in config.items():
            if key in ('flight_enabled','failsafe_validated','sensors_validated','global_z_enabled'):
                if type(value) is not bool:
                    raise ValueError(key+' must be boolean')
            elif key == 'mavros_frame_profile':
                if value not in ('standard_enu','owl_vendor_world'):
                    raise ValueError('invalid mavros_frame_profile')
            elif key != 'world_frame' and (not finite([value]) or value <= 0):
                raise ValueError(key+' must be finite and positive')
        if config['control_hz'] < 20 or config['stable_samples'] < 2:
            raise ValueError('control/stability rates are too low')
        self.c = config
        self.epoch = uuid.uuid4().hex
        self.pose = None
        self.odom_at = -math.inf
        self.stamp = None
        self.frame = None
        self.state_at = -math.inf
        self.ext_at = -math.inf
        self.landed_state = 0
        self.connected = self.armed = self.airborne = False
        self.mode = ''
        self.manual = False
        self.initialized = False
        self.enabled = False
        self.session = None
        self.operator_token = None
        self.operator_session = None
        self.operator_land_requests = {}
        self.retired = set()
        self.lease_at = -math.inf
        self.tasks = {}
        self.active = None
        self.generation = None
        self.trajectory = None
        self.pending_trajectory = None
        self.last_traj_id = -1
        self.planner_at = -math.inf
        self.hold = None
        self.yaw = None
        self.pose_window = deque()
        self.position_span = self.heading_span = 0.
        self.stop_samples = 0
        self.stop_since = None
        self.stopped = False
        self.speed = math.inf
        self.yaw_rate = math.inf
        self.last_error = None
        self.last_tick = None
        self.landing = False

    def fresh(self, now):
        return (self.pose is not None and now - self.odom_at <= self.c['odom_timeout_s']
                and now - self.state_at <= self.c['state_timeout_s'] and self.connected)

    def planner_state(self, now):
        task = self.tasks.get(self.active)
        if task and self.generation is not None and task.get('planner_required'):
            if now-self.planner_at <= self.c['planner_timeout_s']:
                return 'ready'
            if task['status'] == 'planning' and now-task['started'] <= self.c['planning_timeout_s']:
                return 'starting'
            return 'lost'
        return 'not_required'

    def planner_heartbeat(self, generation, now):
        if generation != self.generation or generation is None:
            return False
        self.planner_at = now
        if self.active:
            self.tasks[self.active]['timing_s'].setdefault('first_heartbeat', now-self.tasks[self.active]['started'])
        return True

    def status(self, now):
        return dict(initialized=self.initialized, airborne=self.airborne,
                    control_ready=self.initialized and self.enabled and self.fresh(now)
                    and not self.manual and not self.landing and self.mode == 'OFFBOARD'
                    and self.session is not None and self.armed and self.airborne,
                    odom_ok=self.pose is not None and now-self.odom_at <= self.c['odom_timeout_s'],
                    planner_ok=self.planner_state(now) == 'ready',
                    planner_state=self.planner_state(now),
                    active_task_id=self.active,
                    control_owner=('operator' if self.session == self.operator_session else 'agent') if self.session else 'none',
                    operator_supervised=self.operator_token is not None,
                    hold_ready=self.enabled and self.fresh(now) and not self.manual
                    and not self.landing and self.mode == 'OFFBOARD' and self.armed,
                    landed_state=self.landed_state,
                    landed_state_fresh=now-self.ext_at <= self.c['state_timeout_s'],
                    localization_epoch=self.epoch, manual_takeover=self.manual,
                    error=self.last_error, stopped=self.stopped, landing=self.landing,
                    stop_diagnostics=dict(
                        method='pose_window',
                        position_span_m=self.position_span,
                        position_limit_m=self.c['stop_speed_m_s']*self.c['stable_duration_s'],
                        yaw_span_deg=math.degrees(self.heading_span),
                        yaw_span_limit_deg=math.degrees(self.c['stop_yaw_rate_rad_s']*self.c['stable_duration_s']),
                        speed_m_s=self.speed if math.isfinite(self.speed) else None,
                        yaw_rate_deg_s=math.degrees(self.yaw_rate) if math.isfinite(self.yaw_rate) else None,
                        speed_limit_m_s=self.c['stop_speed_m_s'],
                        yaw_rate_limit_deg_s=math.degrees(self.c['stop_yaw_rate_rad_s']),
                        stable_samples=self.stop_samples,required_samples=self.c['stable_samples'],
                        stable_duration_s=max(0.,self.odom_at-self.stop_since) if self.stop_since is not None else 0.,
                        required_duration_s=self.c['stable_duration_s']))

    def update_state(self, connected, armed, airborne, mode, now):
        incoming_ground = (connected and not armed and self.landed_state == 1
                           and now-self.ext_at <= self.c['state_timeout_s'])
        if self.landing and not incoming_ground and (mode not in ('OFFBOARD','AUTO.LAND')
                or (self.mode == 'AUTO.LAND' and mode != 'AUTO.LAND')):
            self.manual = True
            self.landing = False
            self.enabled = False
            self.fail('manual takeover during landing')
        if self.enabled and not self.landing and self.mode == 'OFFBOARD' and mode != 'OFFBOARD':
            self.manual = True
            self.enabled = False
            self.fail('manual takeover or PX4 mode change')
        if self.enabled and self.armed and not armed and not self.landing:
            self.fail('unexpected disarm')
            self.enabled = False
            self.manual = True
        self.connected, self.armed, self.airborne = connected, armed, airborne
        self.mode, self.state_at = mode, now
        if self.landing and mode == 'AUTO.LAND':
            self.enabled = False
        self.complete_landing(now)

    def update_extended_state(self, landed_state, now):
        self.landed_state, self.ext_at = landed_state, now
        self.airborne = landed_state == 2
        self.complete_landing(now)

    def output_diagnostics(self, now, stamp, output):
        """Read-only snapshot taken under the node lock for one published output."""
        task = self.tasks.get(self.active)
        return dict(monotonic_s=now, setpoint_stamp_ros_s=stamp,
                    localization_epoch=self.epoch, world_frame=self.frame,
                    task_id=self.active, task_status=task['status'] if task else None,
                    goal_z_m=float(task['goal'][2]) if task else None,
                    hold_z_m=float(self.hold[2]) if self.hold is not None else None,
                    output_z_world_m=float(output[0][2]),
                    output_vz_world_m_s=float(output[1][2]),
                    output_az_world_m_s2=float(output[2][2]),
                    measured_z_world_m=float(self.pose[2]) if self.pose is not None else None,
                    odom_stamp_ros_s=self.stamp,
                    odom_receipt_age_s=now-self.odom_at if math.isfinite(self.odom_at) else None,
                    mode=self.mode, position_tolerance_m=self.c['position_tolerance_m'])

    def ground_confirmed(self, now):
        return (self.landed_state == 1 and now-self.ext_at <= self.c['state_timeout_s']
                and now-self.state_at <= self.c['state_timeout_s'] and self.connected and not self.armed)

    def complete_landing(self, now):
        if self.landing and self.ground_confirmed(now):
            self.landing = False
            self.enabled = False
            self.initialized = False
            if self.active:
                self.finish('arrived', True)

    def reset(self, error='localization reset'):
        self.epoch = uuid.uuid4().hex
        self.initialized = False
        task = self.tasks.get(self.active)
        if self.landing and task and task['kind'] == 'land':
            # Explicit AUTO.LAND belongs to the FCU, including an in-flight
            # mode request. Revoke world-frame output, but keep monitoring the
            # accepted landing through fresh FCU ground/disarm confirmation.
            self.last_error = error
            task['localization_error'] = error
            self.invalidate()
        else:
            self.fail(error)
        # A hold point in the old world must never be reused.
        self.hold = None
        self.yaw = None
        self.enabled = False

    def odometry(self, xyz, yaw, velocity, yaw_rate, stamp, frame, now):
        if not finite([*xyz, yaw, *velocity, yaw_rate, stamp]) or not frame:
            self.reset('invalid odometry')
            self.odom_at = -math.inf
            return
        if self.stamp is not None:
            dt = stamp - self.stamp
            jump = np.linalg.norm(np.asarray(xyz)-self.pose[:3])
            if (frame != self.frame or dt < 0 or dt > self.c['odom_timeout_s']
                    or abs(wrap(yaw-self.pose[3])) > self.c['reset_yaw_rad'] + self.c['yaw_rate_rad_s']*max(0,dt)
                    or jump > self.c['reset_jump_m'] + self.c['max_speed_m_s'] * max(0,dt)):
                self.reset()
            elif dt == 0:
                return  # repeated cached samples never satisfy stability
        self.pose = np.array([*xyz, yaw], dtype=float)
        self.speed = float(np.linalg.norm(velocity))
        self.yaw_rate = abs(yaw_rate)
        self.stamp, self.frame, self.odom_at = stamp, frame, now
        if self.yaw is None:
            self.yaw = yaw
        self.update_stop_window(stamp,now)
        if not self.active:
            return
        t = self.tasks[self.active]
        t['diagnostics'] = dict(position_error_cm=float(np.linalg.norm(self.pose[:3]-np.array(t['goal'][:3]))*100),
            vertical_error_cm=float(abs(self.pose[2]-t['goal'][2])*100),
            yaw_error_deg=math.degrees(abs(wrap(yaw-t['goal'][3]))),speed_m_s=self.speed,
            yaw_rate_deg_s=math.degrees(self.yaw_rate),elapsed_s=now-t['started'])
        if t['status'] == 'stopping':
            if self.stopped and self.generation is None and self.enabled and self.mode == 'OFFBOARD':
                self.finish('cancelled', True)
            return
        if t['kind'] == 'land':
            return
        good = (self.enabled and self.mode == 'OFFBOARD' and self.armed and self.airborne and self.stopped and np.linalg.norm(self.pose[:3]-np.array(t['goal'][:3])) <= self.c['position_tolerance_m']
                and (not self.motion_options['vertical_tolerance_enabled']
                     or abs(self.pose[2]-t['goal'][2]) <= 0.08)
                and abs(wrap(yaw-t['goal'][3])) <= self.c['yaw_tolerance_rad'])
        if good:
            self.hold = np.array(t['goal'])
            self.finish('arrived', True)

    def update_stop_window(self, stamp, now):
        # One pose-based stability rule for arrival, cancellation and admission.
        # Keep the sample just before the window boundary; endpoint-only
        # differences could mistake a return swing for a stop.
        duration=self.c['stable_duration_s']
        self.pose_window.append((stamp,self.pose.copy()))
        while len(self.pose_window)>2 and self.pose_window[1][0]<=stamp-duration:
            self.pose_window.popleft()
        samples=np.array([p for _,p in self.pose_window])
        span=stamp-self.pose_window[0][0]
        self.position_span=float(np.linalg.norm(np.ptp(samples[:,:3],axis=0)))
        self.heading_span=float(np.ptp(np.unwrap(samples[:,3])))
        quiet=(self.position_span<=self.c['stop_speed_m_s']*duration
               and self.heading_span<=self.c['stop_yaw_rate_rad_s']*duration)
        self.stop_samples=len(samples) if quiet else 0
        self.stop_since=now-span if quiet else None
        self.stopped=quiet and len(samples)>=self.c['stable_samples'] and span>=duration

    def invalidate(self):
        self.generation = None
        self.trajectory = None
        self.pending_trajectory = None
        self.last_traj_id = -1
        self.pose_window.clear()
        self.position_span = self.heading_span = 0.
        self.stop_samples = 0
        self.stop_since = None
        self.stopped = False
        if self.pose is not None:
            self.hold = self.pose.copy()
            self.yaw = float(self.pose[3])

    def finish(self, status, stopped, error=None):
        if self.active:
            self.tasks[self.active].update(status=status, stopped=stopped)
            self.tasks[self.active]['timing_s']['terminal'] = max(self.odom_at,self.last_tick or self.odom_at)-self.tasks[self.active]['started']
            if error:
                self.tasks[self.active]['error'] = error
        self.active = None
        self.generation = None
        self.trajectory = None
        self.pending_trajectory = None

    def fail(self, error):
        self.last_error = error
        self.invalidate()
        self.finish('failed', False, error)

    def owner(self, session, now):
        self.watchdog(now)
        if not session or session != self.session:
            raise Rejected('invalid or expired session')

    def watchdog(self, now):
        if self.session and now-self.lease_at >= 5.0:
            self.retired.add(self.session)
            self.session = None
            self.fail('control lease expired')
        if self.enabled and not self.fresh(now):
            self.fail('telemetry lost; PX4 offboard-loss failsafe owns recovery')
            self.enabled = False

    def command(self, op, data, now):
        self.watchdog(now)
        if op == 'robot_restart':
            if self.session:
                self.retired.add(self.session)
            self.session = None
            self.operator_token = self.operator_session = None
            self.operator_land_requests.clear()
            self.fail('Robot server restarted')
            self.epoch = uuid.uuid4().hex
            self.initialized = False
            return {'ok': True}
        if op == 'acquire':
            sid = data['session_id']
            transfer = (self.operator_token is not None and self.session == self.operator_session
                        and not data.get('operator_token') and self.initialized and self.enabled
                        and self.fresh(now) and self.armed and self.airborne
                        and self.mode == 'OFFBOARD' and not self.manual
                        and self.last_error is None and data.get('flight_authorized'))
            if sid in self.retired or (self.session and self.session != sid and not transfer):
                raise Rejected('session unavailable')
            if self.operator_token is not None and not transfer:
                raise Rejected('operator supervision requires operator recovery')
            if self.active or self.landing or (self.airborne and not self.stopped):
                raise Rejected('vehicle is not stopped')
            if transfer:
                self.retired.add(self.session)
            if data.get('operator_token'):
                self.operator_token = data['operator_token']
                self.operator_session = sid
            self.session, self.lease_at = sid, now
            return {'ok': True}
        if op in ('operator_heartbeat','operator_land'):
            if not self.operator_token or data.get('operator_token') != self.operator_token:
                raise Rejected('invalid operator token')
            if op == 'operator_heartbeat':
                if self.session != self.operator_session:
                    # An operator must never keep a delegated Agent lease alive.
                    return {'ok': True, 'delegated': True}
                self.lease_at = now
                return {'ok': True}
            sid,tid = data['session_id'],data['task_id']
            if tid in self.operator_land_requests:
                result = self.operator_land_requests[tid]
                if result['session_id'] != sid:
                    raise Rejected('operator request identity mismatch')
                return result.copy()
            if tid in self.tasks:
                if self.tasks[tid]['session_id'] != sid:
                    raise Rejected('operator request identity mismatch')
                return {'ok': True,'session_id':sid,'task_id':tid}
            if self.manual or not self.fresh(now):
                raise Rejected('manual takeover or stale telemetry')
            if sid in self.retired:
                raise Rejected('retired operator session')
            if self.session and self.session != sid:
                self.retired.add(self.session)
            self.session = self.operator_session = sid
            self.lease_at = now
            if self.landing and self.active and self.tasks[self.active]['kind'] == 'land':
                # Adopt an already accepted AUTO.LAND without resending the mode request.
                self.tasks[self.active]['session_id'] = sid
                result = {'ok': True,'session_id':sid,'task_id':self.active}
            else:
                result = dict(self.command('land',data,now),session_id=sid)
            self.operator_land_requests[tid] = result
            return result.copy()
        self.owner(data.get('session_id'), now)
        if op == 'heartbeat':
            self.lease_at = now
            return {'ok': True}
        if op == 'release':
            if self.session == self.operator_session:
                self.operator_token = self.operator_session = None
                self.operator_land_requests.clear()
            self.retired.add(self.session)
            self.session = None
            self.fail('session released')
            return {'ok': True}
        if op == 'init':
            if self.active or self.landing:
                raise Rejected('cannot initialize during active flight task')
            if not self.fresh(now) or self.manual or not data.get('flight_authorized'):
                raise Rejected('preflight not ready or manual takeover latched')
            if self.operator_token and self.session != self.operator_session:
                if not self.initialized:
                    raise Rejected('operator initialization required')
                return {'ok': True, 'message': 'existing operator initialization retained'}
            self.initialized = True
            self.enabled = True
            self.hold = self.pose.copy()
            self.yaw = float(self.pose[3])
            self.last_tick = now
            self.last_error = None
            return {'ok': True, 'message': 'initialized; no arming performed'}
        if op == 'cancel':
            tid = data['task_id']
            if tid not in self.tasks or self.tasks[tid]['session_id'] != self.session:
                raise Rejected('unknown task')
            t = self.tasks[tid]
            if t['kind'] == 'land' and self.landing:
                raise Rejected('landing cannot be cancelled by navigation cancel; use pilot takeover')
            if t['status'] not in ('arrived','cancelled','failed','stopping'):
                self.invalidate()
                t.update(status='stopping', stopped=False)
            return {'ok': True, 'task_id': tid}
        retain_altitude = False
        if op == 'relative':
            relative = data.get('relative')
            if not isinstance(relative,list) or len(relative)!=4 or not finite(relative) or self.pose is None:
                raise Rejected('invalid relative movement')
            x,y,z,yaw = relative
            # A zero Z command keeps the established hold reference. Rebasing
            # it on measured altitude accumulates a persistent hover error.
            retain_altitude = z == 0 and self.hold is not None
            if self.motion_options['global_z_enabled'] and self.hold is not None:
                altitude = float(self.hold[2]) + z
            else:
                altitude = float(self.hold[2]) if retain_altitude else self.pose[2]+z
            c,s = math.cos(self.pose[3]),math.sin(self.pose[3])
            data = dict(data,goal=[self.pose[0]+c*x-s*y,self.pose[1]+s*x+c*y,
                                   altitude,wrap(self.pose[3]+yaw)])
            op = 'navigate'
        if op not in ('navigate','takeoff','land'):
            raise Rejected('unsupported command')
        if op == 'takeoff' and self.operator_token and self.session != self.operator_session:
            raise Rejected('takeoff reserved for operator')
        if self.manual or not self.fresh(now):
            raise Rejected('manual takeover or stale telemetry')
        if op == 'land':
            self.fail('preempted by landing')
            self.landing = not self.ground_confirmed(now)
            # Keep measured hold while mode service is pending, stop on AUTO.LAND State.
            self.enabled = self.enabled and self.landing and self.mode == 'OFFBOARD'
        else:
            if not self.initialized or self.active or self.landing:
                raise Rejected('not initialized or task busy')
            if op == 'navigate' and not self.stopped:
                raise Rejected('previous motion not confirmed stopped')
            if op == 'navigate' and (not self.enabled or not self.airborne or self.mode != 'OFFBOARD'):
                raise Rejected('flight not ready')
            if op == 'takeoff' and self.airborne:
                if not self.enabled or self.mode != 'OFFBOARD':
                    raise Rejected('airborne without bridge authority')
                return {'ok': True, 'message': 'already airborne'}
            if data.get('localization_epoch') != self.epoch:
                raise Rejected('localization epoch mismatch')
        tid = data['task_id']
        if op == 'takeoff':
            goal = self.pose.tolist()
            goal[2] += self.c['takeoff_height_m']
        else:
            goal = data.get('goal', self.pose.tolist())
        if len(goal)!=4 or not finite(goal) or (op!='land' and max(abs(v) for v in goal[:3]) > self.c['world_limit_m']):
            raise Rejected('invalid goal')
        if op == 'navigate' and goal[2] < 0:
            raise Rejected('navigation altitude below supported EGO ground')
        previous_altitude = float(self.hold[2]) if op == 'navigate' and self.hold is not None else None
        self.last_error = None
        self.invalidate()
        if previous_altitude is not None:
            # Navigation admission must not raise the waiting reference to a
            # biased measured height. Keep the established Z until EGO starts,
            # including absolute goals and explicit vertical requests. Cancel
            # and failure still capture measured hold through invalidate.
            self.hold[2] = previous_altitude
        self.active = tid
        self.tasks[tid] = dict(task_id=tid,session_id=self.session,status='accepted',stopped=False,
                               goal=goal,kind=op,started=now, timing_s={},
                               auto_start_pending=op == 'takeoff' and data.get('auto_arm',False),
                               planner_required=bool(op == 'navigate' and np.linalg.norm(np.array(goal[:3])-self.hold[:3]) >= 1e-5))
        if op == 'land':
            if not self.landing:
                self.finish('arrived',True)
        else:
            if op == 'takeoff':
                self.enabled = True
            self.generation = uuid.uuid4().hex
            self.planner_at = -math.inf
            self.tasks[tid]['generation'] = self.generation
            self.tasks[tid]['status'] = 'planning'
        return {'ok': True, 'task_id': tid}

    def trajectory_received(self, generation, trajectory, stamp, now):
        if (not self.active or not self.tasks[self.active].get('planner_required')
                or generation != self.generation or trajectory.id <= self.last_traj_id):
            return False
        if trajectory.start > stamp + .5 or trajectory.start + trajectory.duration < stamp:
            return False
        task = self.tasks[self.active]
        task['timing_s'].setdefault('first_trajectory', now-task['started'])
        if 'yaw_reference_end_stamp' not in task:
            turn_s = abs(wrap(task['goal'][3]-self.yaw))/self.c['yaw_rate_rad_s']
            # XYZ and the separately rate-limited yaw run concurrently. Budget
            # the turn once; subsequent replans must not restart this allowance.
            task['yaw_reference_end_stamp'] = max(stamp,trajectory.start)+turn_s
            task['timing_s']['yaw_reference_ready'] = now-task['started']+max(0.,trajectory.start-stamp)+turn_s
        self.pending_trajectory = trajectory
        self.last_traj_id = trajectory.id
        self.tasks[self.active]['status'] = 'executing'
        return True

    def tick(self, now, stamp):
        self.watchdog(now)
        dt = min(.1, max(0., now-(self.last_tick if self.last_tick is not None else now)))
        self.last_tick = now
        if not self.enabled or self.manual or self.hold is None:
            return None
        if self.landing:
            if self.mode == 'OFFBOARD':
                return (self.hold[:3].copy(),np.zeros(3),np.zeros(3),float(self.hold[3]))
            return None
        if self.pending_trajectory is not None and stamp >= self.pending_trajectory.start:
            self.trajectory = self.pending_trajectory
            self.pending_trajectory = None
        target = self.hold
        vel = acc = np.zeros(3)
        if self.active and self.generation:
            t = self.tasks[self.active]
            direct = t['kind'] == 'takeoff' or np.linalg.norm(np.array(t['goal'][:3])-self.hold[:3]) < 1e-5
            if now-t['started'] > self.c['task_timeout_s']:
                self.fail('task timed out')
            elif not direct and now-self.planner_at > self.c['planner_timeout_s'] and (t['status'] == 'executing' or now-t['started'] > self.c['planning_timeout_s']):
                self.fail('planner heartbeat lost')
            elif not direct and self.trajectory is None and now-t['started'] > self.c['planning_timeout_s']:
                self.fail('planning timed out')
            elif self.trajectory is not None:
                traj = self.trajectory
                deadline = max(traj.start+traj.duration,
                               t.get('yaw_reference_end_stamp',traj.start))
                deadline += self.c['trajectory_grace_s']+self.c['stable_duration_s']
                if stamp > deadline:
                    self.fail('trajectory expired before measured arrival')
                elif stamp >= traj.start:
                    pos, vel, acc = traj.sample(stamp)
                    if stamp >= traj.start+traj.duration:
                        # Hold the actual polynomial endpoint while yaw catches
                        # up; do not keep feeding its terminal derivatives.
                        vel, acc = np.zeros(3),np.zeros(3)
                    target = np.array([*pos, t['goal'][3]])
            if self.active and t['kind'] == 'takeoff':
                target = self.hold.copy()
                if self.armed and self.mode == 'OFFBOARD' and not t.get('auto_start_pending'):
                    progress=t.setdefault('takeoff_progress',dict(z=float(self.pose[2]),at=now))
                    if self.pose[2]>=progress['z']+.03:
                        progress.update(z=float(self.pose[2]),at=now)
                    lead=min(self.c.get('takeoff_max_lead_m',.2),self.c.get('max_tracking_error_m',.5)/2)
                    target[2] = min(t['goal'][2],target[2]+self.c['takeoff_speed_m_s']*dt,float(self.pose[2])+lead)
                    t['takeoff_reference']=dict(z_m=float(target[2]),measured_z_m=float(self.pose[2]),lead_m=float(target[2]-self.pose[2]))
                    if (t['goal'][2]-self.pose[2]>self.c['position_tolerance_m']
                            and now-progress['at']>self.c.get('takeoff_progress_timeout_s',10.)):
                        self.fail('takeoff has no measured upward progress')
                        target=self.hold.copy()
                    else:
                        self.hold = target.copy()
                target[3] = t['goal'][3]
            # Pure yaw does not need a zero-length EGO polynomial.
            if self.active and t['kind'] == 'navigate' and np.linalg.norm(np.array(t['goal'][:3])-self.hold[:3]) < 1e-5:
                target = np.array(t['goal'])
                t['status'] = 'executing'
        if self.last_error and not self.active:
            target, vel, acc = self.hold, np.zeros(3), np.zeros(3)
        if (not np.isfinite(target).all() or not np.isfinite(vel).all() or not np.isfinite(acc).all()
                or np.max(np.abs(target[:3])) > self.c['world_limit_m']
                or np.linalg.norm(target[:3]-self.pose[:3]) > self.c.get('max_tracking_error_m',.5)
                or np.linalg.norm(vel) > self.c['max_speed_m_s']
                or np.linalg.norm(acc) > self.c['max_acceleration_m_s2']):
            if self.active:
                safe=lambda value:float(value) if np.isfinite(value) else None
                self.tasks[self.active]['execution_error']=dict(reference=[safe(v) for v in target],measured=self.pose.tolist(),
                    tracking_error_m=safe(np.linalg.norm(target[:3]-self.pose[:3])),
                    tracking_limit_m=self.c.get('max_tracking_error_m',.5),
                    velocity_m_s=safe(np.linalg.norm(vel)),acceleration_m_s2=safe(np.linalg.norm(acc)))
            self.fail('trajectory exceeds execution/tracking limits')
            target,vel,acc = self.hold,np.zeros(3),np.zeros(3)
        self.yaw = wrap(self.yaw + np.clip(wrap(target[3]-self.yaw),
                          -self.c['yaw_rate_rad_s']*dt,self.c['yaw_rate_rad_s']*dt))
        return (target[:3].copy(), vel, acc, self.yaw)
