"""Disposable upstream EGO processes; each callback closes over immutable ownership.

No traj_server, no mandatory_stop resume, and no vendor/global planning topics.
Process creation/teardown runs outside the flight control lock.
"""
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import time
import threading
import tempfile
import yaml


class PlannerProcess:
    def __init__(self, config, generation, goal, on_trajectory, on_heartbeat):
        import rospy
        from traj_utils.msg import PolyTraj, DataDisp
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from quadrotor_msgs.msg import GoalSet
        from std_msgs.msg import Empty
        self.callback_lock=threading.Lock()
        self.closed=False
        self.ros = rospy
        self.generation = generation
        self.ns = '/owl_ego/planners/g_'+generation
        self.goal = goal
        self.sent = False
        self.birth = time.monotonic()
        self.milestones = {'process_setup_started': self.birth}
        self.handles = []
        self.process = None
        p = config['planner']
        binary = p['executable']
        with open(binary,'rb') as f:
            digest = hashlib.sha256(f.read()).hexdigest()
        if not p['sha256'] or digest != p['sha256']:
            raise RuntimeError('planner executable SHA256 mismatch')
        with open(p['parameters']) as f:
            params = yaml.safe_load(f)
        for key,value in params.items():
            rospy.set_param(self.ns+'/ego/'+key,value)
        self.fsm_ready = False
        self.map_observed = False
        self.odom_pub = rospy.Publisher(self.ns+'/odom',Odometry,queue_size=5)
        self.data_sub = rospy.Subscriber(self.ns+'/fsm_initialized',DataDisp,self.initialized,queue_size=1)
        self.goal_pub = rospy.Publisher(self.ns+'/goal',GoalSet,queue_size=1)
        self.handles = [self.goal_pub,self.odom_pub,self.data_sub,
            rospy.Subscriber('/owl_ego/planner_odom',Odometry,self.odometry,queue_size=5),
            rospy.Subscriber(self.ns+'/map_observed',PointCloud2,self.map_received,queue_size=1),
            rospy.Subscriber(self.ns+'/trajectory',PolyTraj,lambda m:on_trajectory(generation,m),queue_size=2),
            rospy.Subscriber(self.ns+'/heartbeat',Empty,lambda m:on_heartbeat(generation),queue_size=2)]
        remaps = {'~odom_world':self.ns+'/odom','~grid_map/odom':self.ns+'/odom',
                  '~planning/data_display':self.ns+'/fsm_initialized',
                  '~grid_map/occupancy_inflate':self.ns+'/map_observed',
                  '~grid_map/cloud':'/owl_ego/validated_cloud','/goal':self.ns+'/goal',
                  '~planning/trajectory':self.ns+'/trajectory','~planning/heartbeat':self.ns+'/heartbeat',
                  '~planning/broadcast_traj_send':self.ns+'/broadcast_out',
                  '~planning/broadcast_traj_recv':self.ns+'/broadcast_in',
                  '~mandatory_stop':self.ns+'/stop','/traj_start_trigger':self.ns+'/trigger',
                  '/ground_height_measurement':self.ns+'/ground',
                  '/vins_estimator/extrinsic':self.ns+'/disabled_extrinsic'}
        log_dir = p.get('log_dir')
        self.temporary_logs=None
        if not log_dir:
            self.temporary_logs=tempfile.TemporaryDirectory(prefix='owl_ego_planner_')
            log_dir=self.temporary_logs.name
        Path(log_dir).mkdir(parents=True,exist_ok=True)
        log_path=Path(log_dir)/(generation+'.log')
        self.log = open(log_path,'w')
        self.state_log = open(log_path,'r')
        self.state_tail=''
        self.process = subprocess.Popen([binary,'__ns:='+self.ns,'__name:=ego',
                          *[a+':='+b for a,b in remaps.items()]],start_new_session=True,
                          stdout=self.log or subprocess.DEVNULL,stderr=subprocess.STDOUT)
        self.milestones['process_spawned'] = time.monotonic()

    def odometry(self, message):
        # This pinned EGO emits DataDisp once when INIT advances to WAIT_TARGET.
        # Establish its subscriber BEFORE allowing any odom into the process,
        # otherwise that one-shot acknowledgment can be lost. Goal connections
        # or an arbitrary startup sleep do not prove the FSM has processed odom.
        with self.callback_lock:
            if self.closed:return
            if self.data_sub.get_num_connections() and self.odom_pub.get_num_connections():
                self.milestones.setdefault('first_odom_forwarded',time.monotonic())
                self.odom_pub.publish(message)

    def initialized(self, message):
        if 'first_odom_forwarded' in self.milestones:
            self.fsm_ready = True
            self.milestones.setdefault('fsm_initialized',time.monotonic())

    def map_received(self, message):
        # Upstream visualization is heading-filtered: empty output is valid,
        # even with obstacles behind the vehicle. This is a map-output timing
        # milestone, NOT proof of cloud processing or complete map readiness.
        if self.fsm_ready:
            self.map_observed = True
            self.milestones.setdefault('first_map_output',time.monotonic())

    def poll(self):
        from quadrotor_msgs.msg import GoalSet
        if self.process.poll() is not None:
            raise RuntimeError('EGO process exited: '+str(self.process.returncode))
        # DataDisp is one-shot and can be lost during ROS connection setup.
        # The pinned, hash-verified process also flushes its exact INIT transition
        # to its own private stdout log. This is state evidence, not elapsed time.
        if self.state_log is not None and not self.fsm_ready:
            self.state_tail=(self.state_tail+self.state_log.read(16384))[-32768:]
            if ('first_odom_forwarded' in self.milestones
                    and '[FSM]Drone:0, from INIT to WAIT_TARGET' in self.state_tail):
                self.fsm_ready=True
                self.milestones.setdefault('fsm_initialized',time.monotonic())
                self.milestones.setdefault('fsm_log_confirmation',time.monotonic())
        if (self.goal is not None and not self.sent and self.fsm_ready and self.map_observed
                and self.goal_pub.get_num_connections()):
            self.goal_pub.publish(GoalSet(drone_id=0,goal=self.goal[:3]))
            self.sent = True
            self.milestones['goal_sent'] = time.monotonic()

    def close(self):
        with self.callback_lock:
            self.closed=True
            for h in self.handles:
                h.unregister()
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGKILL)
                self.process.wait(timeout=1)
        if self.log:
            self.log.close()
        self.state_log.close()
        if self.temporary_logs:self.temporary_logs.cleanup()
        try:
            self.ros.delete_param(self.ns)
        except KeyError:
            pass
