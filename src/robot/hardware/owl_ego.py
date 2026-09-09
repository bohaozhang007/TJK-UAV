"""Passive ROS sensor cache and bounded command transport. No Captain I/O."""
from __future__ import annotations
from collections import deque
import copy
import json
import math
import threading
import time
import uuid
import numpy as np



class OwlEgoHardware:
    def __init__(self, config):
        self.c = config
        self.lock = threading.RLock()
        self.history = deque(maxlen=600)
        self.images = deque(maxlen=12)
        self.camera_changed = threading.Condition(self.lock)
        self.tf_edges = {}
        self.image = self.info = self.bridge = None
        self.bridge_at = -math.inf
        self.epoch = uuid.uuid4().hex
        self.handles = []

    def start(self):
        import rospy
        import tf2_ros
        from cv_bridge import CvBridge
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image, CameraInfo
        from std_msgs.msg import String
        from owl_nav.srv import Command
        from tf2_msgs.msg import TFMessage
        self.ros = rospy
        if not rospy.core.is_initialized():
            rospy.init_node('owl_ego_robot',disable_signals=True)
        self.cv = CvBridge()
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(15))
        self.listener = tf2_ros.TransformListener(self.tf)
        self.proxy = rospy.ServiceProxy(self.c['topics']['command'],Command)
        for key,cls,cb in [('odom',Odometry,self._odom),('rgb',Image,self._image),
                           ('camera_info',CameraInfo,self._info),('bridge_status',String,self._status)]:
            self.handles.append(rospy.Subscriber(self.c['topics'][key],cls,cb,queue_size=5,
                                                buff_size=16777216))
        self.handles.extend([
            rospy.Subscriber('/tf',TFMessage,lambda m:self._tf_samples(m,False),queue_size=50),
            rospy.Subscriber('/tf_static',TFMessage,lambda m:self._tf_samples(m,True),queue_size=10)])
        rospy.wait_for_service(self.c['topics']['command'],timeout=3)
        result = self.command('robot_restart',{})
        if not result.get('ok'):
            raise RuntimeError(result.get('error','bridge restart fence rejected'))

    def _tf_samples(self,message,static):
        with self.lock:
            for tr in message.transforms:
                key = (tr.header.frame_id,tr.child_frame_id)
                if static:
                    self.tf_edges[key] = None
                else:
                    if self.tf_edges.get(key) is None:
                        self.tf_edges[key] = deque(maxlen=600)
                    self.tf_edges[key].append(tr.header.stamp.to_sec())

    def _odom(self,m):
        p,q = m.pose.pose.position,m.pose.pose.orientation
        value = dict(stamp=m.header.stamp.to_sec(),frame=m.header.frame_id,body=m.child_frame_id,
                     xyz=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w])
        with self.lock:
            if self.history and (value['stamp'] <= self.history[-1]['stamp']
                    or value['frame'] != self.history[-1]['frame']
                    or np.linalg.norm(np.array(value['xyz'])-self.history[-1]['xyz']) > self.c['control']['reset_jump_m']):
                self.history.clear()
            self.history.append(value)
            self.camera_changed.notify_all()

    def _image(self,m):
        with self.lock:
            self.image = m
            self.images.append(m)
            self.camera_changed.notify_all()

    def _info(self,m):
        with self.lock:
            self.info = m
            self.camera_changed.notify_all()

    def _status(self,m):
        try:
            data = json.loads(m.data)
            self._store_status(data)
        except (ValueError,KeyError):
            return

    def _store_status(self,data):
        with self.lock:
            if (self.bridge and self.bridge.get('bridge_id') == data.get('bridge_id')
                    and self.bridge.get('sequence',-1) >= data.get('sequence',0)):
                return
            epoch = data['health']['localization_epoch']
            if self.bridge and epoch != self.epoch:
                self.history.clear()
                self.image = None
                self.images.clear()
                self.camera_changed.notify_all()
            self.epoch = epoch
            self.bridge, self.bridge_at = data,time.monotonic()


    def snapshot(self):
        with self.lock:
            if self.bridge is None or time.monotonic()-self.bridge_at > self.c['hardware']['bridge_timeout_s']:
                raise RuntimeError('bridge status unavailable/stale')
            return copy.deepcopy(self.bridge)

    def command(self, op, data):
        # Deadline is checked again inside the bridge before any side effect.
        payload = dict(op=op,**data,deadline=self.ros.Time.now().to_sec()+1.0)
        event = threading.Event()
        result = {}
        def call():
            try:
                result['value'] = json.loads(self.proxy(json.dumps(payload,allow_nan=False)).json)
            except Exception as e:
                result['error'] = e
            finally:
                event.set()
        threading.Thread(target=call,daemon=True).start()
        if not event.wait(1.5):
            raise RuntimeError('bridge command timeout; outcome uncertain, lease remains enforced')
        if 'error' in result:
            raise RuntimeError('bridge command failed: '+str(result['error']))
        value = result['value']
        if '_snapshot' in value:
            self._store_status(value.pop('_snapshot'))
        return value

    def camera_snapshot(self):
        # Wait releases the sensor lock. Other HTTP requests and flight loops
        # stay independent. Select an atomic, fresh exposure, never current pose.
        from ..controllers.owl_ego_observation import ObservationUnavailable, ObservationEpochChanged
        deadline = time.monotonic()+.08
        with self.camera_changed:
            epoch = self.epoch
            while True:
                if self.epoch != epoch:
                    raise ObservationEpochChanged('localization changed while awaiting observation')
                candidates = list(self.images) or ([self.image] if self.image is not None else [])
                history = list(self.history)
                now = self.now_s()
                for m in reversed(candidates):
                    stamp = m.header.stamp.to_sec()
                    if not 0 <= now-stamp <= self.c['hardware']['rgb_max_age_s']:
                        continue
                    before = [v for v in history if v['stamp'] <= stamp]
                    after = [v for v in history if v['stamp'] >= stamp]
                    if (before and after and max(stamp-before[-1]['stamp'],after[0]['stamp']-stamp)
                            <= self.c['hardware']['sync_max_s']):
                        return (m,self.info,history,epoch,
                            {k:list(v) if v is not None else None for k,v in self.tf_edges.items()})
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise ObservationUnavailable('no fresh exposure bracketed by odometry within 80 ms wait')
                self.camera_changed.wait(remaining)

    def current_epoch(self):
        with self.lock:
            return self.epoch

    def now_s(self):
        return self.ros.Time.now().to_sec()

    def camera_transform(self, body, optical, stamp):
        return self.tf.lookup_transform(body,optical,stamp,self.ros.Duration(.02))

    def rgb_array(self, image):
        return self.cv.imgmsg_to_cv2(image,'bgr8')

    def close(self):
        for h in self.handles:
            h.unregister()
