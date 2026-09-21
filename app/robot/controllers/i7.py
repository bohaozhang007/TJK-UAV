"""i7 camera framing alongside the shared v22 ownership/navigation protocol."""
import base64
from collections import OrderedDict
import copy
import math
import threading
import time
import uuid

import cv2
import numpy as np

from app.robot.config_loader import load_robot_config
from app.robot.i7.calibration import validate
from app.robot.hardware.i7 import I7Hardware
from app.robot.hardware.k40t import K40TClient
from app.robot.hardware.tracker_client import validate_image_box
from .owl_ego import OwlEgoController, ApiError, public_pose
from .owl_ego_observation import ObservationUnavailable
from .i7_framing import autofocus_box, restore_camera


class CameraControlLost(RuntimeError):
    pass


class CameraNotStopped(CameraControlLost):
    pass


class I7Controller(OwlEgoController):
    backend = 'i7'

    def __init__(self, config_path=None, *, hardware=None, config=None, camera=None):
        config = config or load_robot_config('i7', config_path)
        validate(config)
        c = config['camera']
        self.camera = camera or K40TClient(c['host'], c['port'], c['timeout_s'])
        self.camera_lock = threading.Lock()
        self.exposures = OrderedDict()
        self.photo_jobs = {}
        self.photo_active = None
        self.camera_shutdown = threading.Event()
        self.baseline = dict(yaw_deg=c['baseline_yaw_deg'], pitch_deg=c['baseline_pitch_deg'], zoom=c['baseline_zoom'])
        super().__init__(hardware=hardware or I7Hardware(config), config=config)

    def require_baseline(self):
        from app.robot.i7.mapping import require_running
        require_running(self.config, self.hw.ros)
        pose, zoom = self.camera.get_gimbal(), self.camera.get_zoom()
        if (any(not math.isfinite(pose[k]) or abs(pose[k]-self.baseline[k]) > 2.0 for k in ('yaw_deg', 'pitch_deg'))
                or not math.isfinite(float(zoom['zoom'])) or abs(zoom['zoom']-self.baseline['zoom']) > .05
                or zoom.get('zooming')):
            raise ApiError('K40T baseline mismatch: actual='+str(dict(pose, **zoom))
                           +'; expected='+str(self.baseline)
                           +'; tolerance: yaw/pitch=2 deg, zoom=0.05x')
        return dict(pose, zoom=zoom['zoom'])

    def health(self):
        # Camera UDP/HTTP latency must never block the control heartbeat monitor.
        h = self.hw.snapshot()['health'].copy()
        with self.hw.lock:
            image = self.hw.image
            h['rgb_ok'] = image is not None and 0 <= self.hw.now_s()-image.header.stamp.to_sec() <= self.config['hardware']['rgb_max_age_s']
        h['autofocus_active'] = self.photo_active
        return h

    def observation(self, *, include_image=True):
        if not self.camera_lock.acquire(blocking=False):
            raise ApiError('camera framing is active')
        started = time.monotonic()
        deadline = started+10.
        restarts = 0
        try:
            snap = self.hw.snapshot()
            sid, epoch = snap.get('session_id'), snap['health']['localization_epoch']
            pose = public_pose(snap['pose'])
            while True:
                try:
                    result = self._stationary_observation(sid, epoch, pose, deadline,
                                                          include_image=include_image)
                except CameraNotStopped as exc:
                    if time.monotonic() >= deadline:
                        raise ObservationUnavailable('stationary observation timed out after 10 s: '+str(exc)) from exc
                    restarts += 1
                    time.sleep(.05)
                    continue
                result['assembly_elapsed_s'] = time.monotonic()-started
                result['capture_timing']['stationary_restarts'] = restarts
                return result
        finally:
            self.camera_lock.release()

    def _stationary_observation(self, sid, epoch, pose, deadline, *, include_image):
        started = time.monotonic()
        finished, lost = threading.Event(), threading.Event()
        interrupted = []
        monitor_thread = None
        try:
            def guard():
                if lost.is_set():
                    raise interrupted[0]
                self.photo_guard(sid, epoch, pose)
                if time.monotonic() >= deadline:
                    raise ObservationUnavailable('stationary observation timed out after 10 s')
            def monitor():
                while not finished.wait(.05):
                    try:
                        guard()
                    except Exception as exc:
                        interrupted.append(exc)
                        lost.set()
                        return
            guard()
            monitor_thread = threading.Thread(target=monitor, daemon=True)
            monitor_thread.start()
            self.require_baseline()
            guard()
            settle = self.config['camera']['observation_settle_s']
            after = self.hw.now_s()+settle
            self.hw.frame_after(time.monotonic()+settle, settle+3., guard)
            self.require_baseline()
            guard()
            result = super().observation(include_image=include_image)
            guard()
            if result['timestamp_s'] < after:
                raise ObservationUnavailable('camera frame precedes stationary settling interval')
            result['age_s'] = self.hw.now_s()-result['timestamp_s']
            result['assembly_elapsed_s'] = time.monotonic()-started
            result['capture_timing'] = dict(source='stationary_receipt', stationary_checked=True,
                settle_s=settle, exposure_delay_s=None, pose_reference='receipt_time')
            result['geometry_assumptions']['limitations'] = [
                *result['geometry_assumptions'].get('limitations', []),
                'Stationary receipt-time pose; camera exposure latency is unmeasured.']
            if not 0 <= result['age_s'] <= self.config['hardware']['rgb_max_age_s']:
                raise ObservationUnavailable('received RTSP frame exceeds freshness tolerance')
            if include_image:
                with self.hw.lock:
                    image = next((m for m in self.hw.images if abs(m.header.stamp.to_sec()-result['timestamp_s']) < 1e-8), None)
                    if image is None:
                        raise ApiError('exposure expired from camera cache')
                    raw = self.hw.rgb_array(image).copy()
                with self.lock:
                    self.exposures[result['frame_id']] = (copy.deepcopy(result), raw)
                    while len(self.exposures) > 4:
                        self.exposures.popitem(last=False)
            guard()
            return result
        finally:
            finished.set()
            if monitor_thread is not None:
                monitor_thread.join(timeout=.2)

    def handle_http(self, method, path, data, *, local_operator=False):
        if path == '/v21/capabilities' and method == 'GET':
            return dict(super().handle_http(method, path, data, local_operator=local_operator),
                        autofocus=True, manual_offboard_takeoff=self.hw.snapshot()['health'].get('manual_offboard_takeoff') is True)
        if path == '/takeoff' and method == 'POST':
            if self.hw.snapshot()['health'].get('manual_offboard_takeoff') is not True:
                raise ApiError('restart the i7 bridge to enable manual OFFBOARD takeoff')
        if path == '/v22/autofocus' and method == 'POST':
            return self._idempotent(path, data, lambda: self.start_photo(data))
        if path == '/v22/autofocus/status' and method == 'GET':
            self._owner(data)
            with self.lock:
                job = self.photo_jobs.get(data.get('task_id'))
                if not job or job['session_id'] != data.get('session_id'):
                    raise ApiError('unknown photo task', 404)
                return dict(ok=True, **copy.deepcopy(job))
        if method == 'POST' and path in ('/v21/navigation', '/move_relative_xyz_yaw', '/init', '/takeoff'):
            if not self.camera_lock.acquire(blocking=False):
                raise ApiError('camera operation is active')
            try:
                self.require_baseline()
                return super().handle_http(method, path, data, local_operator=local_operator)
            finally:
                self.camera_lock.release()
        return super().handle_http(method, path, data, local_operator=local_operator)

    def photo_guard(self, sid, epoch, pose):
        if self.camera_shutdown.is_set():
            raise CameraControlLost('camera server shutting down')
        snap = self.hw.snapshot()
        h = snap['health']
        if snap.get('session_id') != sid or h['localization_epoch'] != epoch or h.get('manual_takeover'):
            raise CameraControlLost('camera ownership or localization changed')
        missing = [k for k in ('initialized', 'airborne', 'stopped', 'control_ready', 'hold_ready', 'odom_ok') if not h.get(k)]
        if any(k != 'stopped' for k in missing) or h.get('active_task_id') is not None or h.get('error'):
            raise CameraControlLost('camera operation requires stopped flight hold: '
                +str(dict(unavailable=missing, active_task_id=h.get('active_task_id'),
                          stop_diagnostics=h.get('stop_diagnostics'), flight_error=h.get('error'))))
        actual = snap['pose']
        target = [pose['x']/100, -pose['y']/100, pose['z']/100, -math.radians(pose['yaw'])]
        if (not np.isfinite(actual).all()
                or np.linalg.norm(np.asarray(actual[:3])-target[:3]) > self.config['control']['position_tolerance_m']
                or abs((actual[3]-target[3]+math.pi)%(2*math.pi)-math.pi) > self.config['control']['yaw_tolerance_rad']):
            raise CameraControlLost('aircraft moved from the detection exposure')
        if 'stopped' in missing:
            raise CameraNotStopped('waiting for stopped flight hold: '+str(h.get('stop_diagnostics')))

    def start_photo(self, data):
        import ipaddress
        # The Windows tracker shares the detector host; no arbitrary service URL.
        host = str(ipaddress.ip_address(data['tracker_host']))
        with self.lock:
            if data.get('frame_id') not in self.exposures:
                raise ApiError('unknown detection exposure')
            observation, raw = self.exposures[data['frame_id']]
            if data.get('localization_epoch') != observation['localization_epoch']:
                raise ApiError('photo exposure epoch mismatch')
            bounds = validate_image_box(raw, data['box'])
            self.photo_guard(data['session_id'], data['localization_epoch'], observation['pose'])
            if not self.camera_lock.acquire(blocking=False):
                raise ApiError('camera operation is active')
            task = 'photo-'+uuid.uuid4().hex
            job = dict(task_id=task, session_id=data['session_id'], status='running')
            completed = [key for key, value in self.photo_jobs.items() if value['status'] != 'running']
            for key in completed[:-7]:
                del self.photo_jobs[key]
            self.photo_jobs[task] = job
            self.photo_active = task
            worker = threading.Thread(target=self.run_photo,
                args=(job, observation, raw, bounds, host), daemon=True)
            try:
                worker.start()
            except Exception:
                self.photo_active = None
                del self.photo_jobs[task]
                self.camera_lock.release()
                raise
            return dict(ok=True, task_id=task, timeout_s=self.config['autofocus']['timeout_s']+120.)

    def raw_box(self, box):
        # Detector boxes use rectified pixels. Tracker starts on the cached raw exposure.
        c = self.config['hardware']['calibration']
        k, d = np.asarray(c['K'], float), np.asarray(c['D'], float)
        xs, ys = np.meshgrid(np.linspace(box[0], box[2], 20), np.linspace(box[1], box[3], 20))
        pixels = np.column_stack((xs.ravel(), ys.ravel(), np.ones(xs.size)))
        rays = pixels @ np.linalg.inv(k).T
        distorted = cv2.projectPoints(rays, np.zeros(3), np.zeros(3), k, d)[0].reshape(-1, 2)
        w, h = c['image_size']
        lo, hi = distorted.min(axis=0), distorted.max(axis=0)
        return [float(np.clip(lo[0], 0, w)), float(np.clip(lo[1], 0, h)),
                float(np.clip(hi[0], 0, w)), float(np.clip(hi[1], 0, h))]

    def run_photo(self, job, observation, raw, bounds, host):
        sid, epoch = job['session_id'], observation['localization_epoch']
        lost = threading.Event()
        finished = threading.Event()
        def guard():
            if lost.is_set():
                raise CameraControlLost('camera operation interrupted by flight state change')
            self.photo_guard(sid, epoch, observation['pose'])
        def monitor():
            while not finished.wait(.1):
                try:
                    guard()
                except Exception:
                    lost.set()
                    return
        controller = self
        class Camera:
            def get_gimbal(self):
                guard()
                return controller.camera.get_gimbal()
            def get_zoom(self):
                guard()
                return controller.camera.get_zoom()
            def zoom(self, ratio):
                guard()
                return controller.camera.set_zoom(ratio)
            def gimbal_yaw(self, angle):
                guard()
                return controller.camera.gimbal_yaw(angle)
            def gimbal_pitch(self, angle):
                guard()
                return controller.camera.gimbal_pitch(angle)
        camera = Camera()
        photo = {}
        result = dict(status='failed', error='camera worker interrupted')
        self.camera.guard = guard
        def save(frame):
            guard()
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok:
                raise RuntimeError('photo encoding failed')
            photo['image_base64'] = base64.b64encode(encoded).decode('ascii')
        threading.Thread(target=monitor, daemon=True).start()
        try:
            guard()
            capture = lambda after, timeout: self.hw.frame_after(after, timeout, guard)
            try:
                self.require_baseline()
                details = autofocus_box(camera, raw, self.raw_box(bounds), capture,
                    tracker_url=f'http://[{host}]:8791' if ':' in host else f'http://{host}:8791',
                    **self.config['autofocus'], on_photo=save, guard=guard)
            except CameraControlLost:
                raise
            except Exception as exc:
                details = dict(ok=False, message=str(exc))
            guard()
            restoration = restore_camera(camera, self.baseline)
            guard()
            if not restoration['ok']:
                raise RuntimeError('camera baseline restoration failed: '+ '; '.join(restoration['errors']))
            self.require_baseline()
            frame = capture(time.monotonic()+self.config['camera']['settle_s'], 3.)
            focused = bool(details['ok'] and photo)
            if not focused:
                save(frame)
            result = dict(status='completed', focused=focused, details=details,
                          restoration=restoration, **photo)
        except Exception as exc:
            result = dict(status='failed', error=str(exc))
            # CameraControlLost forbids further camera commands, including restoration.
            if not isinstance(exc, CameraControlLost) and not lost.is_set():
                try:
                    guard()
                    result['restoration'] = restore_camera(camera, self.baseline)
                except Exception as restore_error:
                    result['restore_error'] = str(restore_error)
        finally:
            finished.set()
            self.camera.guard = lambda: None
            with self.lock:
                job.update(result)
                self.photo_active = None
                self.camera_lock.release()

    def close(self):
        self.camera_shutdown.set()
        return super().close()
