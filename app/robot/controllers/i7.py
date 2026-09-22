"""i7 camera framing alongside the shared v22 ownership/navigation protocol."""
import base64
from collections import OrderedDict
import copy
import math
import logging
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


class CameraBaselineMismatch(ApiError):
    pass


STOP_RECOVERY_TIMEOUT_S = 5.
STOP_RECOVERY_TOTAL_S = 10.


class I7Controller(OwlEgoController):
    backend = 'i7'

    def __init__(self, config_path=None, *, hardware=None, config=None, camera=None):
        config = config or load_robot_config('i7', config_path)
        validate(config)
        c = config['camera']
        self.camera = camera or K40TClient(c['host'], c['port'], c['timeout_s'],
            angle_tolerance_deg=math.degrees(config['control']['yaw_tolerance_rad']))
        self.camera_lock = threading.Lock()
        self.exposures = OrderedDict()
        self.exposure_clouds = OrderedDict()
        self.photo_jobs = {}
        self.photo_active = None
        self.camera_shutdown = threading.Event()
        self.baseline = dict(yaw_deg=c['baseline_yaw_deg'], pitch_deg=c['baseline_pitch_deg'], zoom=c['baseline_zoom'])
        super().__init__(hardware=hardware or I7Hardware(config), config=config)

    def require_baseline(self):
        angle_tolerance = math.degrees(self.config['control']['yaw_tolerance_rad'])
        from app.robot.i7.mapping import require_running
        require_running(self.config, self.hw.ros)
        pose, zoom = self.camera.get_gimbal(), self.camera.get_zoom()
        if (any(not math.isfinite(pose[k]) or abs(pose[k]-self.baseline[k]) > angle_tolerance for k in ('yaw_deg', 'pitch_deg'))
                or not math.isfinite(float(zoom['zoom'])) or abs(zoom['zoom']-self.baseline['zoom']) > .05
                or zoom.get('zooming')):
            error = CameraBaselineMismatch('K40T baseline mismatch: actual='+str(dict(pose, **zoom))
                           +'; expected='+str(self.baseline)
                           +f'; tolerance: yaw/pitch={angle_tolerance:g} deg, zoom=0.05x')
            error.camera_diagnostics = dict(actual=dict(pose, **zoom), expected=dict(self.baseline),
                                            angle_tolerance_deg=angle_tolerance, zoom_tolerance=.05)
            raise error
        return dict(pose, zoom=zoom['zoom'], zooming=zoom.get('zooming'),
                    focal_length_mm=zoom.get('focal_length_mm'))

    def prepare_baseline(self, guard):
        guard()
        try:
            return self.require_baseline(), None
        except CameraBaselineMismatch:
            previous_guard = self.camera.guard
            self.camera.guard = guard
            try:
                restoration = restore_camera(self.camera, self.baseline, set_zoom=self.camera.set_zoom,
                    angle_tolerance_deg=math.degrees(self.config['control']['yaw_tolerance_rad']))
                guard()
                if not restoration['ok']:
                    error = ApiError('camera baseline restoration failed: '+str(restoration['errors']))
                    error.camera_diagnostics = restoration
                    raise error
                return self.require_baseline(), restoration
            finally:
                self.camera.guard = previous_guard

    def camera_preflight(self):
        checks = []
        for index in range(2):
            started = time.monotonic()
            try:
                state = self.require_baseline()
            except Exception as exc:
                checks.append(dict(sample=index+1, elapsed_s=time.monotonic()-started,
                                   error=str(exc), error_type=type(exc).__name__,
                                   details=getattr(exc, 'camera_diagnostics', None)))
                exc.camera_diagnostics = dict(ready=False, checks=checks)
                raise
            checks.append(dict(sample=index+1, elapsed_s=time.monotonic()-started, state=state))
            print('[I7] Camera preflight: '+str(checks[-1]), flush=True)
        return dict(ready=True, checks=checks)

    def health(self):
        # Camera UDP/HTTP latency must never block the control heartbeat monitor.
        h = self.hw.snapshot()['health'].copy()
        with self.hw.lock:
            image = self.hw.image
            h['rgb_ok'] = image is not None
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
                        error = ObservationUnavailable('stationary observation timed out after 10 s: '+str(exc))
                        error.flight_diagnostics = getattr(exc, 'flight_diagnostics', None)
                        error.observation_diagnostics = getattr(exc, 'observation_diagnostics', {})
                        raise error from exc
                    restarts += 1
                    time.sleep(.05)
                    continue
                result['assembly_elapsed_s'] = time.monotonic()-started
                result['capture_timing']['stationary_restarts'] = restarts
                return result
        except Exception as exc:
            detail = dict(getattr(exc, 'observation_diagnostics', {}))
            detail.update(elapsed_s=time.monotonic()-started, stationary_restarts=restarts,
                          timeout_s=10., reference='observation_start_pose')
            exc.observation_diagnostics = detail
            raise
        finally:
            self.camera_lock.release()

    def _stationary_observation(self, sid, epoch, pose, deadline, *, include_image):
        started = time.monotonic()
        finished, lost = threading.Event(), threading.Event()
        interrupted = []
        monitor_thread = None
        stage = 'initial_hold_check'
        try:
            def guard():
                if lost.is_set():
                    raise interrupted[0]
                try:
                    self.photo_guard(sid, epoch, pose)
                except Exception as exc:
                    exc.observation_diagnostics = dict(stage=stage,
                        attempt_elapsed_s=time.monotonic()-started, reference_pose=copy.deepcopy(pose))
                    raise
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
            stage = 'baseline_before_settle'
            _, restoration = self.prepare_baseline(guard)
            guard()
            settle = self.config['camera']['observation_settle_s']
            after = self.hw.now_s()+settle
            stage = 'settle_and_wait_frame'
            self.hw.frame_after(time.monotonic()+settle, settle+3., guard)
            stage = 'baseline_after_settle'
            camera_baseline = self.require_baseline()
            guard()
            stage = 'assemble_observation'
            retained_images = []
            result = super().observation(include_image=include_image, retain_image=retained_images.append)
            result['camera_baseline'] = camera_baseline
            guard()
            stage = 'validate_frame_timing'
            if result['timestamp_s'] < after:
                raise ObservationUnavailable('camera frame precedes stationary settling interval')
            result['age_s'] = self.hw.now_s()-result['timestamp_s']
            result['assembly_elapsed_s'] = time.monotonic()-started
            result['capture_timing'] = dict(source='stationary_receipt', stationary_checked=True,
                settle_s=settle, exposure_delay_s=None, pose_reference='receipt_time',
                baseline_restoration=restoration)
            result['geometry_assumptions']['limitations'] = [
                *result['geometry_assumptions'].get('limitations', []),
                'Stationary receipt-time pose; camera exposure latency is unmeasured.']
            if include_image:
                stage = 'cache_exposure'
                raw = retained_images[0]
                with self.lock:
                    self.exposures[result['frame_id']] = (copy.deepcopy(result), raw)
                    self.exposure_clouds[result['frame_id']] = self.hw.exposure_cloud(result['timestamp_s'], epoch)
                    while len(self.exposures) > 4:
                        expired, _ = self.exposures.popitem(last=False)
                        self.exposure_clouds.pop(expired, None)
            guard()
            return result
        except Exception as exc:
            if not hasattr(exc, 'observation_diagnostics'):
                exc.observation_diagnostics = dict(stage=stage,
                    attempt_elapsed_s=time.monotonic()-started, reference_pose=copy.deepcopy(pose))
            raise
        finally:
            finished.set()
            if monitor_thread is not None:
                monitor_thread.join(timeout=.2)

    def handle_http(self, method, path, data, *, local_operator=False):
        if path == '/v22/diagnostic/cloud' and method == 'POST':
            return self.diagnostic_cloud(data)
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
                preflight = self.camera_preflight() if path == '/init' else None
                if preflight is None:
                    self.require_baseline()
                result = super().handle_http(method, path, data, local_operator=local_operator)
                if preflight is not None:
                    result['camera_preflight'] = preflight
                return result
            finally:
                self.camera_lock.release()
        return super().handle_http(method, path, data, local_operator=local_operator)

    def diagnostic_cloud(self, data):
        from sensor_msgs import point_cloud2
        self._owner(data)
        epoch = data.get('localization_epoch')
        self.queries.guard(data['session_id'], epoch)
        with self.lock:
            exposure = self.exposures.get(data.get('frame_id'))
            cloud = self.exposure_clouds.get(data.get('frame_id'))
        if exposure is None or exposure[0]['localization_epoch'] != epoch:
            raise ApiError('unknown diagnostic exposure')
        if cloud is None:
            raise ApiError('no point cloud near exposure')
        stamp = cloud.header.stamp.to_sec()
        delta = stamp-exposure[0]['timestamp_s']
        if cloud.header.frame_id != self.config['control']['world_frame'] or abs(delta) > .2:
            raise ApiError('point cloud frame mismatch or exposure time difference exceeds 200 ms')
        if cloud.width*cloud.height > 200000:
            raise ApiError('diagnostic point cloud exceeds 200000 point budget')
        points = np.asarray(list(point_cloud2.read_points(cloud, field_names=('x','y','z'),
                                                         skip_nans=True)), dtype='<f4').reshape(-1,3)
        points = points[np.isfinite(points).all(axis=1)]
        self.queries.guard(data['session_id'], epoch)
        return dict(ok=True, frame_id=data['frame_id'], localization_epoch=epoch,
            points_base64=base64.b64encode(points.tobytes()).decode('ascii'),
            metadata=dict(topic=self.config['topics']['cloud'], frame=cloud.header.frame_id,
                stamp_s=stamp, image_delta_s=delta, point_count=len(points),
                source='registered_cloud_single_frame', position_unit='m', encoding='float32_le_xyz'))

    def photo_guard(self, sid, epoch, pose=None):
        snap = None
        try:
            if self.camera_shutdown.is_set():
                raise CameraControlLost('camera server shutting down')
            snap = self.hw.snapshot()
            self.check_photo_state(snap, sid, epoch, pose)
        except Exception as exc:
            detail = dict(time_s=time.time(), exception_type=type(exc).__name__, message=str(exc),
                          expected_epoch=epoch, exposure_pose=copy.deepcopy(pose),
                          server_shutting_down=self.camera_shutdown.is_set())
            if snap is not None:
                detail.update(health=copy.deepcopy(snap['health']),
                              owner_matches=snap.get('session_id') == sid,
                              reference_pose_check_enabled=pose is not None)
                actual = np.asarray(snap['pose'], float)
                detail['actual_pose_world_m_rad'] = [float(v) if math.isfinite(v) else None for v in actual]
                if np.isfinite(actual).all():
                    detail['actual_pose'] = public_pose(actual)
                if pose is not None and np.isfinite(actual).all():
                    detail.update(position_tolerance_cm=self.config['control']['position_tolerance_m']*100,
                                  yaw_tolerance_deg=math.degrees(self.config['control']['yaw_tolerance_rad']))
                    target = np.array([pose['x']/100, -pose['y']/100, pose['z']/100])
                    detail['position_error_cm'] = float(np.linalg.norm(actual[:3]-target)*100)
                    detail['yaw_error_deg'] = abs((detail['actual_pose']['yaw']-pose['yaw']+180)%360-180)
                    detail['position_delta_cm'] = {k: detail['actual_pose'][k]-pose[k] for k in ('x', 'y', 'z')}
                    detail['yaw_delta_deg'] = (detail['actual_pose']['yaw']-pose['yaw']+180)%360-180
                    detail['exceeded_limits'] = []
                    if detail['position_error_cm'] > detail['position_tolerance_cm']:
                        detail['exceeded_limits'].append('position')
                    if detail['yaw_error_deg'] > detail['yaw_tolerance_deg']:
                        detail['exceeded_limits'].append('yaw')
            exc.flight_diagnostics = detail
            raise

    def check_photo_state(self, snap, sid, epoch, pose=None):
        if self.camera_shutdown.is_set():
            raise CameraControlLost('camera server shutting down')
        h = snap['health']
        if snap.get('session_id') != sid or h['localization_epoch'] != epoch or h.get('manual_takeover'):
            raise CameraControlLost('camera ownership or localization changed')
        missing = [k for k in ('initialized', 'airborne', 'stopped', 'control_ready', 'hold_ready', 'odom_ok') if not h.get(k)]
        if any(k != 'stopped' for k in missing) or h.get('active_task_id') is not None or h.get('error'):
            raise CameraControlLost('camera operation requires stopped flight hold: '
                +str(dict(unavailable=missing, active_task_id=h.get('active_task_id'),
                          stop_diagnostics=h.get('stop_diagnostics'), flight_error=h.get('error'))))
        actual = snap['pose']
        if not np.isfinite(actual).all():
            raise CameraControlLost('invalid aircraft pose')
        # Only stationary observation assembly binds the aircraft to a reference
        # pose. Autofocus tracks live images without the old exposure constraint.
        if pose is not None:
            target = [pose['x']/100, -pose['y']/100, pose['z']/100, -math.radians(pose['yaw'])]
            if (np.linalg.norm(np.asarray(actual[:3])-target[:3]) > self.config['control']['position_tolerance_m']
                    or abs((actual[3]-target[3]+math.pi)%(2*math.pi)-math.pi) > self.config['control']['yaw_tolerance_rad']):
                raise CameraControlLost('aircraft moved from the observation reference pose')
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
            cached = data.get('image_cache')
            if cached is not None:
                if (not isinstance(cached,dict) or cached.get('frame_id') != data['frame_id']
                        or not isinstance(cached.get('token'),str) or len(cached['token']) != 32
                        or any(c not in '0123456789abcdef' for c in cached['token'])):
                    raise ApiError('invalid tracker image cache reference')
                observation = dict(observation, detector_image_cache=dict(cached))
            if data.get('localization_epoch') != observation['localization_epoch']:
                raise ApiError('photo exposure epoch mismatch')
            bounds = validate_image_box(raw, data['box'])
            try:
                self.photo_guard(data['session_id'], data['localization_epoch'])
            except CameraNotStopped:
                pass  # The worker waits within its bounded recovery budget.
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

    def run_photo(self, job, observation, raw, bounds, host):
        sid, epoch = job['session_id'], observation['localization_epoch']
        lost = threading.Event()
        finished = threading.Event()
        interruption = []
        recovery_lock = threading.Lock()
        recoveries = []
        recovery = dict(started=None, total_s=0.)
        def check():
            with recovery_lock:
                if lost.is_set():
                    original = interruption[0]
                    error = CameraControlLost('camera operation interrupted: '+type(original).__name__+': '+str(original))
                    error.flight_diagnostics = getattr(original, 'flight_diagnostics', None)
                    raise error from original
                try:
                    self.photo_guard(sid, epoch)
                except CameraNotStopped as exc:
                    now = time.monotonic()
                    if recovery['started'] is None:
                        recovery['started'] = now
                        recoveries.append(dict(started_s=time.time(), status='waiting',
                            flight_diagnostics=getattr(exc, 'flight_diagnostics', None)))
                        logging.warning('Autofocus %s paused: %s', job['task_id'], exc)
                    elapsed = now-recovery['started']
                    recoveries[-1].update(elapsed_s=elapsed,
                        last_flight_diagnostics=getattr(exc, 'flight_diagnostics', None))
                    if elapsed >= STOP_RECOVERY_TIMEOUT_S or recovery['total_s']+elapsed >= STOP_RECOVERY_TOTAL_S:
                        recoveries[-1]['status'] = 'timeout'
                        error = CameraControlLost('stopped hold did not recover within autofocus wait budget')
                        error.flight_diagnostics = getattr(exc, 'flight_diagnostics', None)
                        interruption.append(error)
                        lost.set()
                        raise error from exc
                    return False
                except Exception as exc:
                    interruption.append(exc)
                    lost.set()
                    raise
                if recovery['started'] is not None:
                    elapsed = time.monotonic()-recovery['started']
                    recovery['total_s'] += elapsed
                    recovery['started'] = None
                    recoveries[-1].update(status='recovered', elapsed_s=elapsed)
                    logging.warning('Autofocus %s stopped hold recovered after %.3f s', job['task_id'], elapsed)
                return True
        def guard():
            while not check():
                if finished.wait(.05):
                    raise CameraControlLost('camera task finished during stopped hold wait')
        def monitor():
            while not finished.wait(.1):
                try:
                    check()
                except Exception:
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
        details = None
        self.camera.guard = guard
        def save(frame):
            guard()
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok:
                raise RuntimeError('photo encoding failed')
            photo['image_base64'] = base64.b64encode(encoded).decode('ascii')
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        try:
            guard()
            capture = lambda after, timeout: self.hw.frame_after(after, timeout, guard)
            try:
                self.require_baseline()
                details = autofocus_box(camera, raw, bounds, capture,
                    image_cache=observation.get('detector_image_cache'),
                    tracker_url=f'http://[{host}]:8791' if ':' in host else f'http://{host}:8791',
                    **self.config['autofocus'], on_photo=save, guard=guard)
            except CameraControlLost:
                raise
            except Exception as exc:
                details = dict(ok=False, message=str(exc))
            guard()
            restoration = restore_camera(camera, self.baseline,
                        angle_tolerance_deg=math.degrees(self.config['control']['yaw_tolerance_rad']))
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
            original = interruption[0] if interruption else exc
            result = dict(status='failed', error=str(exc), error_type=type(original).__name__,
                          original_error=str(original),
                          details=details,
                          flight_diagnostics=getattr(original, 'flight_diagnostics', None))
            # CameraControlLost forbids further camera commands, including restoration.
            if not isinstance(exc, CameraControlLost) and not lost.is_set():
                try:
                    guard()
                    result['restoration'] = restore_camera(camera, self.baseline,
                        angle_tolerance_deg=math.degrees(self.config['control']['yaw_tolerance_rad']))
                except Exception as restore_error:
                    result['restore_error'] = str(restore_error)
        finally:
            finished.set()
            monitor_thread.join(timeout=.2)
            with recovery_lock:
                if recovery['started'] is not None and recoveries[-1]['status'] == 'waiting':
                    recoveries[-1].update(status='interrupted', elapsed_s=time.monotonic()-recovery['started'])
                result['stop_recoveries'] = copy.deepcopy(recoveries)
                result['stop_recovery_limits'] = dict(per_wait_s=STOP_RECOVERY_TIMEOUT_S, total_s=STOP_RECOVERY_TOTAL_S)
            self.camera.guard = lambda: None
            with self.lock:
                job.update(result)
                self.photo_active = None
                self.camera_lock.release()

    def close(self):
        self.camera_shutdown.set()
        return super().close()
