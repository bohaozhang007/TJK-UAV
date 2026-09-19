"""One persistent EGO map with separate preview replies and fenced execution output."""
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import time
import threading
import tempfile
import yaml
from app.timing import measure


class PlannerProcess:
    def __init__(self, config, identity, on_trajectory, on_heartbeat):
        import rospy
        from traj_utils.msg import DataDisp
        from owl_nav_v22.msg import PlannerTrajectory
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        self.callback_lock=threading.Lock()
        self.closed=False
        self.ros = rospy
        self.identity = identity
        self.generation = None
        self.sequence = 0
        self.failed = None
        self.ns = '/owl_ego_v22/planners/g_'+identity
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
        self.handles = [self.odom_pub,self.data_sub,
            rospy.Subscriber('/owl_ego_v22/planner_odom',Odometry,self.odometry,queue_size=5),
            rospy.Subscriber(self.ns+'/map_observed',PointCloud2,self.map_received,queue_size=1),
            rospy.Subscriber(self.ns+'/trajectory',PlannerTrajectory,lambda m:on_trajectory(identity,m),queue_size=2),
            rospy.Subscriber(self.ns+'/heartbeat',String,lambda m:on_heartbeat(identity,m.data),queue_size=2)]
        remaps = {'~odom_world':self.ns+'/odom','~grid_map/odom':self.ns+'/odom',
                  '~planning/data_display':self.ns+'/fsm_initialized',
                  '~grid_map/occupancy_inflate':self.ns+'/map_observed',
                  '~grid_map/cloud':'/owl_ego_v22/validated_cloud','/goal':self.ns+'/goal',
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
        log_path=Path(log_dir)/(identity+'.log')
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
        if self.failed:
            raise RuntimeError(self.failed)
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

    def command(self, mode, generation='', goal=None, timeout=1.):
        from geometry_msgs.msg import Point
        from owl_nav_v22.srv import Plan, PlanRequest
        self.poll()
        self.sequence += 1
        req = PlanRequest(sequence=self.sequence, mode=mode, generation=generation,
                          deadline=self.ros.Time.now()+self.ros.Duration(timeout),
                          goal=Point(*(goal[:3] if goal is not None else [0., 0., 0.])))
        result, event = {}, threading.Event()
        def call():
            try:
                service = self.ns+'/ego/planning/command'
                self.ros.wait_for_service(service, timeout=min(.5, timeout))
                result['value'] = self.ros.ServiceProxy(service, Plan)(req)
            except Exception as exc:
                result['error'] = exc
            finally:
                event.set()
        threading.Thread(target=call, daemon=True).start()
        if not event.wait(timeout):
            self.failed = 'EGO command timed out; restart bridge before further planning'
            raise RuntimeError(self.failed)
        if 'error' in result:
            self.failed = 'EGO command outcome uncertain: '+str(result['error'])
            raise RuntimeError(self.failed)
        response = result['value']
        if not response.success:
            raise RuntimeError(response.error)
        return response

    def select(self, generation, goal):
        if generation == self.generation:
            return
        self.command('idle')
        self.generation = None
        if generation is not None:
            self.command('execute', generation, goal)
            self.generation = generation
            self.milestones['goal_sent'] = time.monotonic()

    def preview(self, goal, timeout, timings=None):
        with measure(timings, 'planner_idle'):
            self.select(None, None)
        with measure(timings, 'ego_service'):
            return self.command('preview', goal=goal, timeout=timeout).trajectory

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
