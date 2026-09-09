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
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Empty, Int32
from owl_nav.srv import Command, CommandResponse
from owl_nav.core import FlightCore, Polynomial, Rejected
from owl_nav.planner import PlannerProcess


class Node:
    def __init__(self):
        with open(rospy.get_param('~config')) as f:
            self.c = yaml.safe_load(f)
        self.lock = threading.RLock()
        self.core = FlightCore(self.c['control'])
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
        self.setpoint_pub = None
        self.pending_fcu = None
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
        if not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1) > .02:
            with self.lock:
                self.core.fail('invalid attitude')
            return
        # nav_msgs/Odometry twist is in child_frame_id; EGO expects world velocity.
        world_v = quaternion_matrix(quat)[:3,:3]@np.array([v.x,v.y,v.z])
        planner_odom = copy.deepcopy(m)
        planner_odom.twist.twist.linear.x,planner_odom.twist.twist.linear.y,planner_odom.twist.twist.linear.z = world_v
        with self.lock:
            if m.child_frame_id != 'base_link' or m.header.frame_id != self.c['control']['world_frame']:
                self.core.reset('unexpected odometry frames')
                return
            self.core.odometry([p.x,p.y,p.z],euler_from_quaternion(quat)[2],world_v,w.z,
                               m.header.stamp.to_sec(),m.header.frame_id,time.monotonic())
            self.last_odom = m
        self.odom_pub.publish(planner_odom)

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

    def cloud(self,m):
        stamp = m.header.stamp.to_sec()
        now = rospy.Time.now().to_sec()
        with self.lock:
            valid = (self.last_odom is not None and m.header.frame_id == self.core.frame
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
                if op == 'takeoff' and (self.pending_fcu or self.landed != ExtendedState.LANDED_STATE_ON_GROUND) and not self.core.airborne:
                    raise Rejected('not confirmed on ground or FCU service busy')
                if op == 'navigate' and data['goal'][2] < 0:
                    raise Rejected('navigation altitude below supported EGO ground')
                result = self.core.command(op,data,now)
                if op == 'land' and self.core.landing:
                    self.pending_fcu = ('land',data['task_id'],now)
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
        self.status_sequence += 1
        data = dict(health=h,session_id=c.session,bridge_id=self.bridge_id,sequence=self.status_sequence,
                    pose=c.pose.tolist() if c.pose is not None else None,tasks=copy.deepcopy(c.tasks))
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
                    if c.enabled and (self.conflicts or now-self.conflict_at>2):
                        c.fail('competing controller or control-authority audit stale')
                        c.enabled = False
                        c.manual = True
                    if c.enabled and now-self.ext_at > self.c['control']['state_timeout_s']:
                        c.fail('extended state lost')
                        c.enabled = False
                    if c.active and now-self.cloud_at > self.c['control']['cloud_timeout_s']:
                        c.fail('obstacle cloud lost')
                    output = c.tick(now,rospy.Time.now().to_sec())
                    if output is not None:
                        if self.setpoint_pub is None:
                            self.setpoint_pub = rospy.Publisher(self.c['topics']['setpoint'],PositionTarget,queue_size=1)
                        p,v,a,y = output
                        m = PositionTarget()
                        m.header.stamp = rospy.Time.now()
                        m.header.frame_id = self.c['control']['world_frame']
                        # MAVROS receives ENU and converts once to PX4 NED.
                        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                        m.type_mask = PositionTarget.IGNORE_YAW_RATE
                        m.position.x,m.position.y,m.position.z = p
                        m.velocity.x,m.velocity.y,m.velocity.z = v
                        m.acceleration_or_force.x,m.acceleration_or_force.y,m.acceleration_or_force.z = a
                        m.yaw = y
                        self.setpoint_pub.publish(m)
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
                    goal = list(task['goal']) if task and self.core.generation and task['kind']=='navigate' else None
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
                with self.lock:
                    self.conflicts,self.conflict_at = sorted(set(bad)),time.monotonic()
            except Exception:
                pass  # a stale audit revokes authority in the independent loop
            self.stop.wait(.5)

    def fcu_loop(self):
        # Pilot selects OFFBOARD and arms. This worker never arms or re-enters
        # OFFBOARD, so a delayed service reply cannot revive a cancelled takeoff.
        while not self.stop.wait(.05):
            with self.lock:
                item = self.pending_fcu
                if not item:
                    continue
                op,tid,started = item
                self.pending_fcu = None
                if self.core.active != tid or self.core.manual:
                    continue
            try:
                rospy.wait_for_service('/mavros/set_mode',timeout=1)
                with self.lock:
                    if self.core.active != tid or self.core.manual:
                        continue
                response = rospy.ServiceProxy('/mavros/set_mode',SetMode)(custom_mode='AUTO.LAND')
                if not response.mode_sent:
                    raise RuntimeError('PX4 rejected AUTO.LAND request')
            except Exception as e:
                with self.lock:
                    if self.core.active == tid:
                        self.core.fail('FCU service failed: '+str(e))


if __name__ == '__main__':
    rospy.init_node('owl_ego_bridge')
    Node()
    rospy.spin()
