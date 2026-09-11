#!/usr/bin/env python3
"""50 Hz onboard bridge. Starts passive; flight requires explicit validated config."""
import copy
import json
import math
import threading
import time
import uuid
import numpy as np
import rospy
import rosgraph
import yaml
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Empty, Int32
from owl_nav.srv import Command, CommandResponse
from owl_nav.core import FlightCore, Polynomial, Rejected
from owl_nav.planner import PlannerProcess
from owl_nav.software_takeoff import prepare
from owl_nav.frames import Frames, Alignment
from owl_nav.cloud import validate as validate_cloud


class Node:
    def __init__(self):
        with open(rospy.get_param('~config')) as f:
            self.c = yaml.safe_load(f)
        self.lock = threading.RLock()
        self.core = FlightCore(self.c['control'],
                              global_z_enabled=self.c.get('global_z_enabled', False),
                              vertical_tolerance_enabled=self.c.get('vertical_tolerance_enabled', False))
        self.frames = Frames(self.c['control'].get('mavros_frame_profile','standard_enu'))
        with open(self.c['planner']['parameters']) as f:
            planner_params=yaml.safe_load(f)
        self.ceiling = float(planner_params['grid_map/virtual_ceil'])-float(planner_params['grid_map/obstacles_inflation'])
        if not math.isfinite(self.ceiling) or self.ceiling<=0:
            raise ValueError('invalid planner ceiling')
        self.alignment = Alignment()
        self.frame_config_ok = not self.frames.vendor
        self.stop = threading.Event()
        self.cloud_at = -math.inf
        self.cloud_stamp = -math.inf
        self.ext_at = -math.inf
        self.landed = None
        self.conflict_at = -math.inf
        self.conflicts = ['preflight not checked']
        self.idle_generation = uuid.uuid4().hex
        self.planner_generation = None
        self.status_sequence = 0
        self.bridge_id = uuid.uuid4().hex
        self.last_odom = None
        self.last_world_velocity = None
        self.setpoint_pub = None
        self.pending_fcu = None
        self.fcu_busy = False
        self.fcu_uncertain = False
        self.stream_since = None
        self.last_output_at = -math.inf
        self.last_control_sample = None
        topics = self.c['topics']
        self.status_pub = rospy.Publisher(topics['bridge_status'],String,queue_size=1)
        self.odom_pub = rospy.Publisher('/owl_ego/planner_odom',Odometry,queue_size=5)
        self.cloud_pub = rospy.Publisher('/owl_ego/validated_cloud',PointCloud2,queue_size=1)
        self.subs = [rospy.Subscriber(topics['odom'],Odometry,self.odom,queue_size=5),
            rospy.Subscriber(topics['state'],State,self.state,queue_size=5),
            rospy.Subscriber(topics['extended_state'],ExtendedState,self.ext,queue_size=5),
            rospy.Subscriber(topics['cloud'],PointCloud2,self.cloud,queue_size=1),
            rospy.Subscriber(topics['localization_reset'],Empty,self.reset,queue_size=1),
            rospy.Subscriber(topics['vision_pose_reset'],Int32,self.reset,queue_size=1)]
        if self.frames.vendor:
            self.subs += [rospy.Subscriber('/mavros/local_position/pose',PoseStamped,lambda m:self.reference('map',m),queue_size=5),
                          rospy.Subscriber('/mavros/vision_pose/pose',PoseStamped,lambda m:self.reference('lio',m),queue_size=5)]
        self.service = rospy.Service(topics['command'],Command,self.command)
        # Wall-clock loops remain live when simulated ROS time pauses.
        self.workers = []
        for target in (self.control_loop,self.planner_loop,self.audit_loop,self.fcu_loop):
            worker = threading.Thread(target=target,daemon=True)
            self.workers.append(worker)
            worker.start()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.stop.set()
        for worker in self.workers:
            if worker is not threading.current_thread():
                worker.join(timeout=3)

    def odom(self,m):
        from tf.transformations import quaternion_matrix, euler_from_quaternion
        if not 0 <= rospy.Time.now().to_sec()-m.header.stamp.to_sec() <= self.c['control']['odom_timeout_s']:
            return
        p,q = m.pose.pose.position,m.pose.pose.orientation
        quat = [q.x,q.y,q.z,q.w]
        v,w = m.twist.twist.linear,m.twist.twist.angular
        if (not np.isfinite([*quat,p.x,p.y,p.z,v.x,v.y,v.z,w.x,w.y,w.z]).all()
                or abs(np.linalg.norm(quat)-1) > .02):
            with self.lock:
                self.core.reset('invalid attitude')
                self.core.odom_at = -math.inf
            return
        # Standard Odometry twist is body-relative; vendor OWL publishes world twist.
        rotation=quaternion_matrix(quat)[:3,:3]
        world_v = self.frames.world_velocity(rotation,[v.x,v.y,v.z])
        try:
            yaw_rate=self.frames.yaw_rate(rotation,[w.x,w.y,w.z])
        except ValueError as e:
            with self.lock:self.core.reset(str(e))
            return
        planner_odom = copy.deepcopy(m)
        planner_odom.twist.twist.linear.x,planner_odom.twist.twist.linear.y,planner_odom.twist.twist.linear.z = world_v
        with self.lock:
            if m.child_frame_id != 'base_link' or m.header.frame_id != self.c['control']['world_frame']:
                self.core.reset('unexpected odometry frames')
                return
            self.core.odometry([p.x,p.y,p.z],euler_from_quaternion(quat)[2],world_v,yaw_rate,
                               m.header.stamp.to_sec(),m.header.frame_id,time.monotonic())
            self.last_odom = m
            self.last_world_velocity = world_v.copy()
            if self.frames.vendor:
                try:
                    self.alignment.check(m.header.stamp.to_sec(),[p.x,p.y,p.z],self.core.pose[3],time.monotonic())
                except ValueError as e:
                    self.core.reset(str(e))
                    return
        self.odom_pub.publish(planner_odom)

    def reference(self,kind,m):
        from tf.transformations import euler_from_quaternion
        p,q=m.pose.position,m.pose.orientation
        quat=[q.x,q.y,q.z,q.w]
        with self.lock:
            try:
                expected='map' if kind=='map' else self.c['control']['world_frame']
                if m.header.frame_id!=expected or not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1)>.02:
                    raise ValueError('invalid '+kind+' frame/attitude')
                self.alignment.add(kind,m.header.stamp.to_sec(),[p.x,p.y,p.z],euler_from_quaternion(quat)[2])
                if self.last_odom is not None and self.core.pose is not None:
                    self.alignment.check(self.core.stamp,self.core.pose[:3],self.core.pose[3],time.monotonic())
            except ValueError as e:
                self.alignment.error=str(e)
                self.alignment.good_at[kind]=-math.inf
                self.core.reset(str(e))

    def state(self,m):
        with self.lock:
            self.core.update_state(m.connected,m.armed,self.landed == ExtendedState.LANDED_STATE_IN_AIR,
                                   m.mode,time.monotonic())

    def ext(self,m):
        with self.lock:
            self.ext_at = time.monotonic()
            self.landed = m.landed_state
            self.core.update_extended_state(m.landed_state,self.ext_at)

    def reset(self,m):
        with self.lock:
            self.core.reset('explicit localization reset')
            self.cloud_at=self.cloud_stamp=-math.inf

    def cloud(self,m):
        stamp = m.header.stamp.to_sec()
        now = rospy.Time.now().to_sec()
        try:
            validate_cloud(m)
        except ValueError:
            with self.lock:
                self.cloud_at=-math.inf
            return
        with self.lock:
            valid = (stamp>self.cloud_stamp and self.last_odom is not None and m.header.frame_id == self.core.frame
                     and 0 <= now-stamp <= self.c['control']['cloud_timeout_s']
                     and abs(stamp-self.core.stamp) <= self.c['control']['cloud_sync_max_s']
                     and m.width*m.height > 0 and {'x','y','z'} <= {f.name for f in m.fields})
            if valid:
                self.cloud_at = time.monotonic()
                self.cloud_stamp = stamp
        if valid:
            self.cloud_pub.publish(m)

    def authorized(self,now):
        c = self.c['control']
        return (c['flight_enabled'] and c['failsafe_validated'] and c['sensors_validated']
                and self.frame_config_ok and (not self.frames.vendor or self.alignment.ready(now))
                and not self.conflicts and now-self.conflict_at < 2
                and now-self.cloud_at < c['cloud_timeout_s']
                and now-self.ext_at < c['state_timeout_s'])

    def command(self,req):
        try:
            data = json.loads(req.json)
            if not isinstance(data,dict) or not math.isfinite(data.get('deadline',math.nan)):
                raise Rejected('invalid command deadline')
            with self.lock:
                if rospy.Time.now().to_sec() > data['deadline']:
                    raise Rejected('command deadline expired')
                now = time.monotonic()
                op = data.pop('op')
                if op in ('init','takeoff','navigate','relative'):
                    if not self.authorized(now):
                        raise Rejected('flight disabled, competing controllers, or unvalidated/stale sensors/planner')
                data['flight_authorized'] = self.authorized(now)
                if op == 'takeoff' and (self.fcu_busy or self.fcu_uncertain):
                    raise Rejected('FCU operation busy or previous outcome uncertain')
                if op == 'takeoff' and data.get('auto_arm') and not self.core.airborne and self.core.armed:
                    raise Rejected('software takeoff requires disarmed ground state')
                if op == 'takeoff' and (self.pending_fcu or self.landed != ExtendedState.LANDED_STATE_ON_GROUND) and not self.core.airborne:
                    raise Rejected('not confirmed on ground or FCU service busy')
                if op in ('navigate','relative','takeoff') and self.core.pose is not None:
                    z = (data['goal'][2] if op=='navigate' else self.core.pose[2]+
                         (data['relative'][2] if op=='relative' else self.c['control']['takeoff_height_m']))
                    if not (op=='takeoff' and self.core.airborne) and not 0 <= z < self.ceiling:
                        raise Rejected('target altitude outside planner ground/ceiling bounds')
                was_landing = self.core.landing
                result = self.core.command(op,data,now)
                if op == 'takeoff' and data.get('auto_arm') and self.core.active == data['task_id']:
                    self.pending_fcu = ('takeoff',data['task_id'],now)
                if op == 'land' and self.core.landing:
                    self.pending_fcu = ('land',data['task_id'],now)
                if op == 'operator_land' and not was_landing and self.core.landing and self.core.active == result['task_id']:
                    self.pending_fcu = ('land',result['task_id'],now)
                # Publish ownership immediately so session acquisition is observable.
                result['_snapshot'] = self.publish_status(now)
                return CommandResponse(json=json.dumps(result))
        except Exception as e:
            return CommandResponse(json=json.dumps(dict(ok=False,error=str(e),code=409)))

    def publish_status(self,now):
        c = self.core
        h = c.status(now)
        h['control_ready'] = h['control_ready'] and self.authorized(now)
        h['hold_ready'] = h['hold_ready'] and self.authorized(now)
        h['conflicting_publishers'] = self.conflicts
        h['mavros_frame_profile'] = self.frames.profile
        h['frame_alignment_ok'] = self.frame_config_ok and (not self.frames.vendor or self.alignment.ready(now))
        h['frame_alignment_error'] = self.alignment.error if self.frames.vendor else None
        self.status_sequence += 1
        data = dict(health=h,session_id=c.session,bridge_id=self.bridge_id,sequence=self.status_sequence,
                    pose=c.pose.tolist() if c.pose is not None else None,tasks=copy.deepcopy(c.tasks))
        data['control_sample'] = copy.deepcopy(self.last_control_sample)
        data['control_sample_age_s'] = (now-self.last_control_sample['monotonic_s']
                                       if self.last_control_sample is not None else None)
        self.status_pub.publish(String(data=json.dumps(data,allow_nan=False)))
        return data

    def control_loop(self):
        dt = 1/self.c['control']['control_hz']
        last_status = 0
        while not self.stop.wait(dt):
            try:
                with self.lock:
                    now = time.monotonic()
                    c = self.core
                    if c.enabled and (not self.frame_config_ok or (self.frames.vendor and not self.alignment.ready(now))):
                        c.reset('frame alignment unavailable')
                    if c.enabled and (self.conflicts or now-self.conflict_at>2):
                        c.fail('competing controller or control-authority audit stale')
                        c.enabled = False
                        c.manual = True
                    if c.enabled and now-self.ext_at > self.c['control']['state_timeout_s']:
                        c.fail('extended state lost')
                        c.enabled = False
                    if c.active and not c.landing and now-self.cloud_at > self.c['control']['cloud_timeout_s']:
                        c.fail('obstacle cloud lost')
                    output = c.tick(now,rospy.Time.now().to_sec())
                    if output is not None:
                        if self.setpoint_pub is None:
                            self.setpoint_pub = rospy.Publisher(self.c['topics']['setpoint'],PositionTarget,queue_size=1)
                        p,v,a,y = self.frames.setpoint(*output)
                        m = PositionTarget()
                        m.header.stamp = rospy.Time.now()
                        m.header.frame_id = 'map' if self.frames.vendor else self.c['control']['world_frame']
                        # Adapt vendor world to map before MAVROS performs ENU->NED.
                        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                        m.type_mask = PositionTarget.IGNORE_YAW_RATE
                        m.position.x,m.position.y,m.position.z = p
                        m.velocity.x,m.velocity.y,m.velocity.z = v
                        m.acceleration_or_force.x,m.acceleration_or_force.y,m.acceleration_or_force.z = a
                        m.yaw = y
                        self.setpoint_pub.publish(m)
                        sample = c.output_diagnostics(now,m.header.stamp.to_sec(),output)
                        sample.update(mavros_profile=self.frames.profile,
                                      mavros_frame_id=m.header.frame_id,
                                      mavros_coordinate_frame=int(m.coordinate_frame),
                                      mavros_type_mask=int(m.type_mask),
                                      published_z_m=float(m.position.z),
                                      published_vz_m_s=float(m.velocity.z),
                                      published_az_m_s2=float(m.acceleration_or_force.z),
                                      measured_vz_world_m_s=(float(self.last_world_velocity[2])
                                          if self.last_world_velocity is not None else None),
                                      conflicting_publishers=list(self.conflicts),
                                      publisher_audit_age_s=now-self.conflict_at)
                        self.last_control_sample = sample
                        if now-self.last_output_at > .15:
                            self.stream_since = now
                        self.last_output_at = now
                    else:
                        self.stream_since = None
                    if now-last_status >= .1:
                        self.publish_status(now)
                        last_status = now
            except Exception as e:
                with self.lock:
                    self.core.fail('control loop error: '+str(e))
                    self.core.enabled = False
                rospy.logerr_throttle(1,str(e))

    def trajectory(self,g,m):
        try:
            traj = Polynomial(m)
            with self.lock:
                self.core.trajectory_received(g,traj,rospy.Time.now().to_sec(),time.monotonic())
        except Exception as e:
            with self.lock:
                if g == self.core.generation:
                    self.core.fail('invalid EGO trajectory: '+str(e))

    def heartbeat(self,g):
        with self.lock:
            self.core.planner_heartbeat(g,time.monotonic())

    def planner_loop(self):
        process = None
        try:
            while not self.stop.wait(.05):
                with self.lock:
                    g = self.core.generation or self.idle_generation
                    task = self.core.tasks.get(self.core.active)
                    goal = list(task['goal']) if task and self.core.generation and task.get('planner_required') else None
                    if goal is None:
                        g = self.idle_generation
                try:
                    if not process or process.generation != g:
                        if process:
                            process.close()
                            process = None
                        with self.lock:
                            self.planner_generation = g
                        process = PlannerProcess(self.c,g,goal,self.trajectory,self.heartbeat)
                    process.poll()
                    with self.lock:
                        if self.core.generation == g and self.core.active:
                            task = self.core.tasks[self.core.active]
                            task['timing_s'].update({k:v-task['started'] for k,v in process.milestones.copy().items()})
                except Exception as e:
                    with self.lock:
                        if self.core.generation == g:
                            self.core.fail('planner failure: '+str(e))
                    rospy.logwarn_throttle(5,str(e))
                    self.stop.wait(.5)
        finally:
            if process:
                process.close()

    def audit_loop(self):
        while not self.stop.is_set():
            try:
                pubs,subs,services = rosgraph.Master(rospy.get_name()).getSystemState()
                bad = []
                prefixes = ('/mavros/setpoint_','/mavros/actuator_control','/mavros/rc/override')
                excluded = {'/mavros/setpoint_raw/target_local','/mavros/setpoint_raw/target_global',
                            '/mavros/setpoint_raw/target_attitude','/mavros/setpoint_trajectory/desired'}
                for topic,nodes in pubs:
                    if topic.startswith(prefixes) and topic not in excluded:
                        bad.extend(n for n in nodes if n != rospy.get_name())
                all_nodes = {n for t,nodes in pubs for n in nodes}
                bad.extend(n for n in all_nodes if n.rsplit('/',1)[-1] in ('captain','mavros_controller'))
                frame_config_ok = (not self.frames.vendor or all(abs(float(rospy.get_param('/mavros/'+key,0.)))<1e-9 for key in ('x','y','z','R','P','Y')))
                with self.lock:
                    self.frame_config_ok = frame_config_ok
                    self.conflicts,self.conflict_at = sorted(set(bad)),time.monotonic()
            except Exception:
                pass  # a stale audit revokes authority in the independent loop
            self.stop.wait(.5)

    def bounded_fcu(self, service, cls, **kwargs):
        # Requests cannot be recalled once sent to MAVROS. Unknown outcome
        # latches this bridge against another software arm attempt.
        done = threading.Event()
        result = {}
        def call():
            try:
                result['response'] = rospy.ServiceProxy(service,cls)(**kwargs)
            except Exception as e:
                result['error'] = e
            finally:
                done.set()
        threading.Thread(target=call,daemon=True).start()
        if not done.wait(1.5):
            with self.lock:
                self.fcu_uncertain = True
            raise RuntimeError('FCU service timed out; outcome uncertain')
        if 'error' in result:
            with self.lock:
                self.fcu_uncertain = True
            raise result['error']
        response = result['response']
        if not getattr(response,'mode_sent',getattr(response,'success',False)):
            raise RuntimeError('FCU rejected '+service)

    def software_takeoff(self, tid):
        with self.lock:
            sid,epoch = self.core.session,self.core.epoch
        def guard():
            with self.lock:
                c = self.core
                now = time.monotonic()
                c.watchdog(now)
                task = c.tasks.get(tid,{})
                if (self.stop.is_set() or c.active != tid or c.session != sid or c.epoch != epoch
                        or c.manual or not c.enabled or not c.fresh(now) or not self.authorized(now)
                        or task.get('status') not in ('planning','executing')
                        or not task.get('auto_start_pending') or self.fcu_uncertain):
                    raise RuntimeError('software takeoff ownership/authority invalidated')
                return dict(mode=c.mode,armed=c.armed)
        def warm():
            with self.lock:
                now = time.monotonic()
                return (self.stream_since is not None and now-self.stream_since>=1.2
                        and now-self.last_output_at<.15 and self.setpoint_pub is not None
                        and self.setpoint_pub.get_num_connections()>0)
        def mode_request():
            rospy.wait_for_service('/mavros/set_mode',timeout=1)
            guard()
            self.bounded_fcu('/mavros/set_mode',SetMode,custom_mode='OFFBOARD')
        def arm_request():
            rospy.wait_for_service('/mavros/cmd/arming',timeout=1)
            state = guard()
            with self.lock:
                if state['mode']!='OFFBOARD' or self.landed != ExtendedState.LANDED_STATE_ON_GROUND:
                    raise RuntimeError('not confirmed OFFBOARD/on ground before arm')
            self.bounded_fcu('/mavros/cmd/arming',CommandBool,value=True)
        prepare(guard,mode_request,arm_request,warm)
        with self.lock:
            guard()
            self.core.tasks[tid]['auto_start_pending'] = False

    def fcu_loop(self):
        while not self.stop.wait(.05):
            with self.lock:
                item = self.pending_fcu
                if not item:
                    continue
                op,tid,started = item
                self.pending_fcu = None
                if self.core.active != tid or self.core.manual:
                    continue
                self.fcu_busy = True
            try:
                if op == 'takeoff':
                    self.software_takeoff(tid)
                else:
                    rospy.wait_for_service('/mavros/set_mode',timeout=1)
                    with self.lock:
                        if self.core.active != tid or self.core.manual:
                            continue
                    self.bounded_fcu('/mavros/set_mode',SetMode,custom_mode='AUTO.LAND')
            except Exception as e:
                with self.lock:
                    owns_task = self.core.active == tid
                    if owns_task:
                        self.core.fail('FCU service failed: '+str(e))
                    if op == 'takeoff' and not self.core.landing and (owns_task or self.fcu_uncertain):
                        # No ascent/retry after an uncertain or cancelled arm request.
                        self.core.enabled = False
                        self.core.manual = True
            finally:
                with self.lock:
                    self.fcu_busy = False


if __name__ == '__main__':
    rospy.init_node('owl_ego_bridge')
    Node()
    rospy.spin()
