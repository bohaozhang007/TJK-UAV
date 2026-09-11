"""Protocol v1 ownership, idempotency and coordinate conversion for OWL EGO."""
from __future__ import annotations
import copy
import json
import math
import threading
import time
import uuid
from ..config_loader import load_robot_config


class ApiError(Exception):
    def __init__(self, message, code=409):
        super().__init__(message)
        self.code = code


def public_pose(p):
    return dict(x=p[0]*100, y=-p[1]*100, z=p[2]*100,
                yaw=(-math.degrees(p[3])+180)%360-180)


def enu_pose(p, limit):
    if not isinstance(p, dict) or set(p) != {'x','y','z','yaw'}:
        raise ApiError('pose requires x,y,z,yaw', 400)
    vals = [p[k] for k in ('x','y','z','yaw')]
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in vals):
        raise ApiError('pose must contain finite numbers',400)
    if max(abs(v) for v in vals[:3]) > limit*100 or not -180 <= vals[3] < 180:
        raise ApiError('pose out of range',400)
    return [vals[0]/100, -vals[1]/100, vals[2]/100, -math.radians(vals[3])]


class OwlEgoController:
    backend = 'owl_ego'
    def __init__(self, image_dir=None, config_path=None, *, hardware=None, config=None):
        self.config = config or load_robot_config('owl_ego',config_path)
        self.motion_options = {key: self.config.get(key,False)
                               for key in ("global_z_enabled","vertical_tolerance_enabled")}
        if any(type(v) is not bool for v in self.motion_options.values()):
            raise ValueError("Z options must be booleans")
        if hardware is None:
            from ..hardware.owl_ego import OwlEgoHardware
            hardware = OwlEgoHardware(self.config)
        self.hw = hardware
        self.lock = threading.RLock()
        self.requests = {}
        self.session = None
        self.session_requests = {}
        self.operator_token = None
        self.operator_session = None
        self.operator_requests = {}
        self.frame_size = None
        self.hw.start()  # passive observations before session/init

    def _command(self, op, **data):
        result = self.hw.command(op,data)
        if not result.get('ok'):
            raise ApiError(result.get('error','bridge rejected command'),result.get('code',409))
        return result

    def _owner(self, data):
        if not self.session or data.get('session_id') != self.session:
            raise ApiError('invalid session')
        snap = self.hw.snapshot()
        if snap.get('session_id') != self.session:
            raise ApiError('expired or released session')

    def _idempotent(self, path, data, operation):
        rid = data.get('request_id')
        try:
            uuid.UUID(rid)
        except (ValueError,TypeError,AttributeError):
            raise ApiError('request_id must be UUID',400)
        key = (data.get('session_id'),rid)
        canonical = json.dumps([path,data],sort_keys=True,allow_nan=False)
        with self.lock:
            self._owner(data)
            entry = self.requests.get(key)
            if entry and entry['body'] != canonical:
                raise ApiError('request_id reused with different body')
            if entry is None:
                entry = dict(body=canonical,event=threading.Event())
                self.requests[key] = entry
                creator = True
            else:
                creator = False
        if creator:
            try:
                entry['result'] = operation()
            except Exception as e:
                entry['error'] = e
            finally:
                entry['event'].set()
        elif not entry['event'].wait(175 if not path.startswith('/v21/') else 2):
            raise ApiError('original request still in progress',503)
        if 'error' in entry:
            raise entry['error']
        return copy.deepcopy(entry['result'])

    def handle_http(self, method, path, data, *, local_operator=False):
        gets = {'/v21/capabilities','/v21/observation','/v21/navigation/status',
                '/health','/get_pose','/motion_tolerances','/v21/motion_log'}
        posts = {'/v21/session','/v21/heartbeat','/v21/session/release',
                 '/v21/navigation','/v21/navigation/cancel','/init','/takeoff',
                 '/move_relative_xyz_yaw','/land','/v21/operator/land','/v21/operator/stop'}
        if path not in gets | posts:
            raise ApiError('unsupported owl_ego endpoint',404)
        if (method == 'GET') != (path in gets):
            raise ApiError('method not allowed',405)
        if method == 'GET':
            if path == '/v21/motion_log':
                if not local_operator:
                    raise ApiError('motion log is local only',403)
                snap = self.hw.snapshot()
                keys = ('task_id','kind','status','goal','started_wall_s','finished_wall_s',
                        'before_pose','after_pose','before_epoch','after_epoch','relative_command','source','error')
                return dict(ok=True, bridge_id=snap.get('bridge_id'), tasks=[
                    {k:t[k] for k in keys if k in t} for t in snap.get('tasks',{}).values()])
            if path == '/v21/capabilities':
                return dict(ok=True,backend='owl_ego',protocol_version=1,async_navigation=True,
                            cancel_and_hold=True,synchronized_observation=True,
                            control_lease=True,relative_xyz_yaw=True,software_takeoff=True,operator_override=True,operator_stop=True)
            if path == '/v21/observation':
                result = self.observation()
                with self.lock:
                    if self.session and self.frame_size and self.frame_size != result['image_size']:
                        raise ApiError('image dimensions changed during session',503)
                    if self.session:
                        self.frame_size = result['image_size']
                return result
            if path == '/get_pose':
                return self.get_pose()
            if path == '/motion_tolerances':
                return dict(ok=True,motion_tolerances=self.get_motion_tolerances())
            if path == '/health':
                return dict(ok=True,health=self.health())
            snap = self.hw.snapshot()
            tid = data.get('task_id')
            if tid not in snap.get('tasks',{}):
                raise ApiError('unknown task_id',404)
            task = snap['tasks'][tid]
            result = {k:v for k,v in dict(ok=True,**task).items()
                      if k in ('ok','task_id','status','stopped','error','generation','timing_s','diagnostics','execution_error','takeoff_reference','localization_error')}
            # Immutable accepted world goal; land has no fixed position target.
            if task.get('kind') != 'land' and task.get('goal') is not None:
                result['target'] = public_pose(task['goal'])
            return result
        if path in ('/v21/operator/land','/v21/operator/stop'):
            return self._operator_land(data,local_operator,op='operator_'+path.rsplit('/',1)[-1])
        if path == '/v21/session':
            operator = data.get('operator',False)
            if type(operator) is not bool:
                raise ApiError('operator must be boolean',400)
            if operator and not local_operator:
                raise ApiError('operator access requires loopback connection',403)
            rid = data.get('request_id')
            try:
                uuid.UUID(rid)
            except (ValueError,TypeError,AttributeError):
                raise ApiError('request_id must be UUID',400)
            with self.lock:
                if rid in self.session_requests:
                    original, result = self.session_requests[rid]
                    if original != data:
                        raise ApiError('request_id reused with different body')
                    if self.hw.snapshot().get('session_id') != result['session_id']:
                        raise ApiError('session has expired; request cannot revive it')
                    return result.copy()
                sid = uuid.uuid4().hex
                token = uuid.uuid4().hex if operator else None
                self._command('acquire',session_id=sid,**({'operator_token':token} if operator else {}))
                self.session = sid
                self.frame_size = None
                result = dict(ok=True,session_id=sid)
                if operator:
                    self.operator_token,self.operator_session = token,sid
                    result['operator_token'] = token
                self.session_requests[rid] = (copy.deepcopy(data),result)
                event = threading.Event()
                event.set()
                self.requests[(sid,rid)] = dict(body=json.dumps([path,data],sort_keys=True),result=result,event=event)
                return result
        if path in ('/v21/heartbeat','/v21/session/release'):
            if path.endswith('heartbeat') and self.operator_token and data.get('session_id') == self.operator_session:
                return self._command('operator_heartbeat',operator_token=self.operator_token)
            self._owner(data)
            op = 'heartbeat' if path.endswith('heartbeat') else 'release'
            # Use the request's owner, never a new owner installed by a concurrent override.
            result = self._command(op,session_id=data['session_id'])
            if op == 'release' and data['session_id'] == self.operator_session:
                self.operator_token = self.operator_session = None
            return result
        return self._idempotent(path,data,lambda:self._mutate(path,data))

    def _operator_land(self,data,local_operator,op='operator_land'):
        if not local_operator:
            raise ApiError('operator access requires loopback connection',403)
        rid = data.get('request_id')
        try: uuid.UUID(rid)
        except (ValueError,TypeError,AttributeError):
            raise ApiError('request_id must be UUID',400)
        with self.lock:
            if not self.operator_token or data.get('operator_token') != self.operator_token:
                raise ApiError('invalid operator token',403)
            key = (self.operator_token,rid)
            body = json.dumps(dict(data,operation=op),sort_keys=True,allow_nan=False)
            entry = self.operator_requests.get(key)
            if entry and entry['body'] != body:
                raise ApiError('request_id reused with different body')
            if entry is None:
                entry = dict(body=body,session_id=uuid.uuid4().hex,task_id='nav-'+uuid.uuid4().hex)
                self.operator_requests[key] = entry
            if 'result' not in entry:
                result = self._command(op,operator_token=self.operator_token,
                    session_id=entry['session_id'],task_id=entry['task_id'])
                if self.hw.snapshot().get('session_id') == result['session_id']:
                    self.session = self.operator_session = result['session_id']
                entry['result'] = result
            return copy.deepcopy(entry['result'])

    def _mutate(self, path, data):
        sid = data['session_id']
        if path == '/init':
            # Bridge performs live conflict/sensor checks and injects authorization.
            return self._command('init',session_id=sid)
        if path == '/v21/navigation/cancel':
            return self._command('cancel',session_id=sid,task_id=data.get('task_id'))
        snap = self.hw.snapshot()
        tid = 'nav-'+uuid.uuid4().hex
        epoch = snap['health']['localization_epoch']
        if path == '/v21/navigation':
            goal = enu_pose(data.get('pose'),self.config['control']['world_limit_m'])
            return self._command('navigate',session_id=sid,task_id=tid,goal=goal,
                                 localization_epoch=data.get('localization_epoch'))
        timeout = self.config['controller']['motion_timeout_s']
        if path == '/move_relative_xyz_yaw':
            vals = [data.get(k) for k in ('x','y','z','yaw')]
            if any(type(v) is not int for v in vals):
                raise ApiError('relative x,y,z,yaw must be integer cm/deg',400)
            x,y,z,yaw = vals
            relative = [x/100,-y/100,z/100,-math.radians(yaw)]
            timeout = data.get('timeout_s',timeout)
            if isinstance(timeout,bool) or not isinstance(timeout,(int,float)) or not math.isfinite(timeout) or not 0 < timeout <= 170:
                raise ApiError('invalid timeout_s',400)
            result = self._command('relative',session_id=sid,task_id=tid,relative=relative,localization_epoch=epoch)
        elif path == '/takeoff':
            goal = list(snap['pose'])
            goal[2] += self.config['control']['takeoff_height_m']
            timeout = self.config['controller']['takeoff_timeout_s']
            auto_arm = data.get('auto_arm',False)
            if type(auto_arm) is not bool:
                raise ApiError('auto_arm must be boolean',400)
            result = self._command('takeoff',session_id=sid,task_id=tid,goal=goal,localization_epoch=epoch,auto_arm=auto_arm)
        else:
            timeout = self.config['controller']['landing_timeout_s']
            result = self._command('land',session_id=sid,task_id=tid)
        if 'task_id' not in result:
            return result
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            # No flight-duration controller lock: telemetry/lease/land can preempt.
            task = self.hw.snapshot().get('tasks',{}).get(tid)
            if task and task['status'] in ('arrived','failed','cancelled'):
                if task['status'] == 'arrived' and task['stopped']:
                    return dict(ok=True,message='motion completed',task_id=tid)
                detail=task.get('error',task['status'])
                if task.get('execution_error'):
                    detail+='; '+json.dumps(task['execution_error'],allow_nan=False)
                raise ApiError(detail)
            time.sleep(.04)
        if path != '/land':
            self._command('cancel',session_id=sid,task_id=tid)
        raise ApiError('motion timed out; stop requested' if path != '/land' else 'landing confirmation timed out',504)

    def observation(self):
        from .owl_ego_observation import build_observation, InvalidObservation
        try:
            return build_observation(self.hw)
        except ValueError as e:
            if hasattr(e,'error_code'):
                raise
            raise InvalidObservation(str(e)) from e

    def health(self):
        try:
            h = self.hw.snapshot()['health'].copy()
        except Exception as e:
            h = dict(initialized=False,airborne=False,control_ready=False,odom_ok=False,
                     planner_ok=False,localization_epoch=self.hw.epoch,error=str(e))
        try:
            self.observation()
            h['rgb_ok'] = True
        except Exception as e:
            h['rgb_ok'] = False
            h['observation_error_code'] = getattr(e,'error_code','invalid_observation')
            h['observation_retryable'] = getattr(e,'retryable',False)
        return h

    def get_pose(self):
        s = self.hw.snapshot()
        if not s['health']['odom_ok'] or s.get('pose') is None:
            raise ApiError('odometry unavailable',503)
        return dict(ok=True,pose=public_pose(s['pose']),localization_epoch=s['health']['localization_epoch'])

    def get_motion_tolerances(self):
        c = self.config['control']
        return dict(position_tolerance_cm=c['position_tolerance_m']*100,
                    global_z_enabled=getattr(self,'motion_options',{}).get('global_z_enabled',False),
                    vertical_tolerance_enabled=getattr(self,'motion_options',{}).get('vertical_tolerance_enabled',False),
                    yaw_tolerance_deg=math.degrees(c['yaw_tolerance_rad']),
                    position_error_metric='euclidean_3d',source='owl_ego')

    def close(self):
        if self.session:
            try:
                self._command('release',session_id=self.session)
            except Exception:
                pass  # independent bridge lease remains the shutdown backstop
        self.hw.close()
        return dict(ok=True,message='server closed; onboard lease/hold remains active')
