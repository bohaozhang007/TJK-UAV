"""Disposable upstream EGO processes; each callback closes over immutable ownership.

No traj_server, no mandatory_stop resume, and no vendor/global planning topics.
Process creation/teardown runs outside the flight control lock.
"""
import hashlib
import os
import signal
import subprocess
import time
import yaml


class PlannerProcess:
    def __init__(self, config, generation, goal, on_trajectory, on_heartbeat):
        import rospy
        from traj_utils.msg import PolyTraj
        from quadrotor_msgs.msg import GoalSet
        from std_msgs.msg import Empty
        self.ros = rospy
        self.generation = generation
        self.ns = '/owl_ego/planners/g_'+generation
        self.goal = goal
        self.sent = False
        self.birth = time.monotonic()
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
        self.goal_pub = rospy.Publisher(self.ns+'/goal',GoalSet,queue_size=1)
        self.handles = [self.goal_pub,
            rospy.Subscriber(self.ns+'/trajectory',PolyTraj,lambda m:on_trajectory(generation,m),queue_size=2),
            rospy.Subscriber(self.ns+'/heartbeat',Empty,lambda m:on_heartbeat(generation),queue_size=2)]
        remaps = {'~odom_world':'/owl_ego/planner_odom','~grid_map/odom':'/owl_ego/planner_odom',
                  '~grid_map/cloud':'/owl_ego/validated_cloud','/goal':self.ns+'/goal',
                  '~planning/trajectory':self.ns+'/trajectory','~planning/heartbeat':self.ns+'/heartbeat',
                  '~planning/broadcast_traj_send':self.ns+'/broadcast_out',
                  '~planning/broadcast_traj_recv':self.ns+'/broadcast_in',
                  '~mandatory_stop':self.ns+'/stop','/traj_start_trigger':self.ns+'/trigger',
                  '/ground_height_measurement':self.ns+'/ground',
                  '/vins_estimator/extrinsic':self.ns+'/disabled_extrinsic'}
        self.process = subprocess.Popen([binary,'__ns:='+self.ns,'__name:=ego',
                          *[a+':='+b for a,b in remaps.items()]],start_new_session=True,
                          stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)

    def poll(self):
        from quadrotor_msgs.msg import GoalSet
        if self.process.poll() is not None:
            raise RuntimeError('EGO process exited: '+str(self.process.returncode))
        if self.goal is not None and not self.sent and self.goal_pub.get_num_connections() and time.monotonic()-self.birth > .5:
            self.goal_pub.publish(GoalSet(drone_id=0,goal=self.goal[:3]))
            self.sent = True

    def close(self):
        for h in self.handles:
            h.unregister()
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGTERM)
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGKILL)
                self.process.wait(timeout=1)
        try:
            self.ros.delete_param(self.ns)
        except KeyError:
            pass
