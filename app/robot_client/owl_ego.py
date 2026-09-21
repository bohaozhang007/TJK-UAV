"""OWL implementation of the public mission interface; no model or DA3 dependency."""
import base64
import io
import json
import math
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

import numpy as np
from PIL import Image

from app.timing import measure
from .base import ControlLost, MissionError, Observation, TargetNotLocalizable, NavigationPlanningFailed
from app.robot.mapping import GridMap, TargetSurfaceUnavailable


# Fixed local Robot deployment and control protocol timings.
ROBOT_URL = 'http://127.0.0.1:8765'
ALLOW_APPROXIMATE_GEOMETRY = True
ATTACH_TIMEOUT_S = 120.0
STOP_TIMEOUT_S = 8.0
PREVIEW_TIMEOUT_S = 12.0
NAVIGATION_TIMEOUT_S = 180.0


class HttpError(MissionError):
    def __init__(self, status, result):
        super().__init__(result.get('error', str(result)))
        self.status, self.result = status, result


class OwlEgoClient:
    observation_max_age_s = .5
    observation_sync_max_s = .05
    def __init__(self, *, min_target_voxels, target_depth_gap_m):
        self.min_target_voxels = min_target_voxels
        self.target_depth_gap_m = target_depth_gap_m
        self.url = ROBOT_URL
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.session = self.epoch = None
        self.error = None
        self.lease_deadline = 0.
        self.done = threading.Event()
        self.thread = None
        self.landing = False
        self.navigation_id = None

    def rpc(self, method, path, data=None, timeout=1., timings=None):
        data = dict(data or {})
        if method == 'POST' and self.session:
            data.setdefault('session_id', self.session)
        if method == 'POST' and path != '/v21/heartbeat':
            data.setdefault('request_id', str(uuid.uuid4()))
        body = json.dumps(data, allow_nan=False).encode() if method == 'POST' else None
        if method == 'GET' and data:
            path += '?' + urllib.parse.urlencode(data)
        request = urllib.request.Request(self.url + path, body, {'Content-Type': 'application/json'}, method=method)
        try:
            with measure(timings, 'http_round_trip'):
                with self.http.open(request, timeout=timeout) as response:
                    result = json.load(response)
        except urllib.error.HTTPError as exc:
            with exc:
                result = json.load(exc)
            if timings is not None:
                timings['server'] = result.get('timings', {})
            raise HttpError(exc.code, result) from exc
        if timings is not None:
            timings['server'] = result.get('timings', {})
        if result.get('ok') is not True:
            raise MissionError(str(result))
        return result

    def check_lease(self):
        if self.error:
            raise ControlLost(self.error)
        if not self.session or time.monotonic() >= self.lease_deadline:
            self.error = 'control lease expired'
            raise ControlLost(self.error)

    def heartbeat(self):
        while not self.done.is_set():
            try:
                self.check_lease()
                sent = time.monotonic()
                self.rpc('POST', '/v21/heartbeat', timeout=min(.7, self.lease_deadline-sent))
                self.check_lease()  # A late response cannot revive an expired lease.
                self.lease_deadline = sent + 5.
                self.done.wait(.7)
            except HttpError as exc:
                self.error = 'control ownership lost: ' + str(exc)
                return
            except ControlLost:
                return
            except Exception:
                if time.monotonic() >= self.lease_deadline:
                    self.error = 'heartbeat recovery exhausted the original lease'
                    return
                self.done.wait(.1)

    def attach(self):
        caps = self.rpc('GET', '/v21/capabilities')
        if getattr(self, 'autofocus_enabled', False) and caps.get('autofocus') is not True:
            raise MissionError('Robot does not support autofocus; set photography.autofocus=false')
        for key in ('async_navigation', 'cancel_and_hold', 'synchronized_observation', 'control_lease',
                    'sensor_geometry', 'map_query', 'preview_only', 'stop_at_goal'):
            if caps.get(key) is not True:
                raise MissionError('missing Robot capability: ' + key)
        self.rpc('GET', '/sensor_geometry')
        deadline = time.monotonic() + ATTACH_TIMEOUT_S
        while True:
            h = self.rpc('GET', '/health')['health']
            if (h.get('airborne') and h.get('stopped') and not h.get('active_task_id')
                    and h.get('control_ready') and h.get('hold_ready') and h.get('control_owner') == 'operator'):
                break
            if time.monotonic() >= deadline:
                raise MissionError('console takeoff/stable operator hold not ready')
            time.sleep(.2)
        self.epoch = h['localization_epoch']
        sent = time.monotonic()
        result = self.rpc('POST', '/v21/session')
        self.session = result['session_id']
        self.lease_deadline = sent + 5.
        self.thread = threading.Thread(target=self.heartbeat, daemon=True)
        self.thread.start()
        self.rpc('POST', '/init')  # Acknowledges existing console initialization; never takes off.
        self.tolerances = self.rpc('GET', '/motion_tolerances')['motion_tolerances']
        self.wait_stopped()
        return self.pose()

    def health(self):
        self.check_lease()
        h = self.rpc('GET', '/health')['health']
        self.check_lease()
        if (h.get('control_owner') != 'agent' or h.get('manual_takeover')
                or h.get('localization_epoch') != self.epoch):
            self.error = 'operator takeover or localization epoch changed'
            raise ControlLost(self.error)
        if not all(h.get(k) for k in ('initialized', 'airborne', 'control_ready', 'hold_ready', 'odom_ok')):
            raise MissionError('flight state unavailable: ' + str(h))
        return h

    def pose(self):
        self.health()
        result = self.rpc('GET', '/get_pose')
        if result.get('localization_epoch') != self.epoch:
            self.error = 'pose localization epoch changed'
            raise ControlLost(self.error)
        pose = result['pose']
        if not all(math.isfinite(float(pose[k])) for k in ('x', 'y', 'z', 'yaw')):
            raise MissionError('invalid pose')
        return pose

    def wait_stopped(self):
        deadline = time.monotonic() + STOP_TIMEOUT_S
        while time.monotonic() < deadline:
            h = self.health()
            if h.get('stopped') and h.get('active_task_id') is None:
                return
            time.sleep(.1)
        raise MissionError('actual stop not confirmed')

    def observe(self, timings=None):
        with measure(timings, 'wait_stopped'):
            self.wait_stopped()
        sent = time.monotonic()
        data = self.rpc('GET', '/v21/observation', timeout=1., timings=timings)
        if data.get('localization_epoch') != self.epoch:
            self.error = 'observation localization epoch changed'
            raise ControlLost(self.error)
        with measure(timings, 'image_decode'):
            with Image.open(io.BytesIO(base64.b64decode(data['rgb_image_base64'], validate=True))) as im:
                rgb = np.array(im.convert('RGB'))
        quality = data.get('calibration_quality')
        if (data.get('rectified') is not True or quality not in ('calibrated', 'approximate')
                or quality == 'approximate' and not ALLOW_APPROXIMATE_GEOMETRY):
            raise MissionError('camera geometry is not accepted')
        if quality == 'approximate':
            g = data.get('geometry_assumptions', {})
            if (g.get('intrinsics') != 'configured_calibration' or g.get('extrinsics') != 'sensor_geometry'
                    or g.get('distortion') != 'corrected' or not g.get('profile_id')):
                raise MissionError('unsupported approximate camera geometry')
        k = np.asarray(data['intrinsics'], float)
        t = np.asarray(data['world_from_camera_optical_cm'], float)
        if (k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
                or not np.allclose(k[2], [0, 0, 1]) or abs(np.linalg.det(k)) < 1e-9
                or t.shape != (4, 4) or not np.isfinite(t).all()
                or not np.allclose(t[3], [0, 0, 0, 1])
                or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-4)
                or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-4)
                or data['image_size'] != [rgb.shape[1], rgb.shape[0]]):
            raise MissionError('invalid observation geometry')
        with measure(timings, 'health_after'):
            self.health()
        with measure(timings, 'freshness_check'):
            elapsed = time.monotonic() - sent
            server_age = float(data['age_s'])
            assembly = float(data['assembly_elapsed_s'])
            sync = float(data['sync_error_s'])
            if not all(math.isfinite(v) for v in (server_age, assembly, sync)) or not 0 <= assembly <= elapsed:
                raise MissionError('invalid observation timing metadata')
            # Request/response overhead remains a conservative age allowance.
            age = server_age + elapsed - assembly
            if timings is not None:
                timings['observation'] = dict(frame_id=data['frame_id'], server_age_s=server_age,
                    server_assembly_elapsed_s=assembly, client_elapsed_s=elapsed,
                    age_upper_bound_s=age, sync_error_s=sync, max_age_s=self.observation_max_age_s, max_sync_s=self.observation_sync_max_s)
            if server_age < 0 or not 0 <= age <= self.observation_max_age_s:
                raise MissionError(f'observation stale: age_upper_bound_s={age:.6f}, limit_s={self.observation_max_age_s:.6f}, '
                                   f'server_age_s={server_age:.6f}, assembly_s={assembly:.6f}, client_elapsed_s={elapsed:.6f}')
            if not 0 <= sync <= self.observation_sync_max_s:
                raise MissionError(f'observation unsynchronized: sync_error_s={sync:.6f}, limit_s={self.observation_sync_max_s:.6f}')
        return Observation(data['frame_id'], data['pose'], self.epoch, rgb, data['rgb_image_base64'], k, t, data['timestamp_s'],
                           {key:data[key] for key in ('geometry_assumptions', 'camera_baseline', 'capture_timing',
                               'sync_error_s', 'odom_bracket_span_s') if key in data})

    def autofocus_photo(self, observation, box, timings=None):
        raise MissionError('Robot does not support camera autofocus')

    def grid(self, timings=None):
        with measure(timings, 'wait_stopped'):
            self.wait_stopped()
        data = self.rpc('POST', '/v22/map', {'localization_epoch': self.epoch}, timeout=3., timings=timings)
        if data['localization_epoch'] != self.epoch:
            raise ControlLost('map localization epoch changed')
        self.health()
        with measure(timings, 'map_decode'):
            return GridMap(data['map'])

    def locate_target(self, observation, mask, timings=None, diagnostic_prefix=None):
        if observation.epoch != self.epoch:
            raise ControlLost('old exposure epoch')
        current = self.pose()
        delta = np.linalg.norm([current[k] - observation.pose[k] for k in ('x', 'y', 'z')])
        yaw = abs((current['yaw'] - observation.pose['yaw'] + 180) % 360 - 180)
        if delta > self.tolerances['position_tolerance_cm'] or yaw > self.tolerances['yaw_tolerance_deg']:
            raise MissionError('vehicle moved while awaiting detection')
        grid = self.grid(timings)
        diagnostics = {} if diagnostic_prefix is not None else None
        with measure(timings, 'mask_projection_and_localization'):
            try:
                return grid.locate(observation, mask, min_voxels=self.min_target_voxels,
                                   depth_gap_m=self.target_depth_gap_m, diagnostics=diagnostics)
            except TargetSurfaceUnavailable as exc:
                if diagnostics is not None:
                    diagnostics['error'] = str(exc)
                raise TargetNotLocalizable(str(exc)) from exc
            finally:
                if diagnostics is not None:
                    from .localization_diagnostics import save_projection
                    try:
                        summary = save_projection(diagnostic_prefix, observation, mask, diagnostics)
                        if timings is not None:
                            timings['localization_diagnostics'] = summary
                    except Exception as exc:
                        if timings is not None:
                            timings['localization_diagnostics'] = dict(write_error=str(exc))

    def points_are_free(self, poses, timings=None):
        grid = self.grid(timings)
        clearance = self.tolerances['position_tolerance_cm']
        if timings is not None:
            timings['point_clearance_cm'] = clearance
        with measure(timings, 'point_collision_check'):
            return [grid.free(pose, clearance_cm=clearance) for pose in poses]

    def point_is_free(self, pose, timings=None):
        return self.points_are_free([pose], timings=timings)[0]

    def preview_path(self, goal, require_arrival=False, timings=None):
        with measure(timings, 'wait_stopped'):
            self.wait_stopped()
        result = self.rpc('POST', '/v22/preview', {'pose': goal, 'localization_epoch': self.epoch,
                           'require_arrival': require_arrival}, timeout=PREVIEW_TIMEOUT_S + 2., timings=timings)
        self.health()
        if result.get('localization_epoch') != self.epoch:
            raise ControlLost('preview localization epoch changed')
        points = np.asarray(result['points_cm'], float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
            raise MissionError('invalid preview path')
        return points

    def navigate(self, goal, timings=None):
        self.wait_stopped()
        # Never replay a navigation after a transport failure or uncertain acceptance.
        result = self.rpc('POST', '/v21/navigation', {'pose': goal, 'localization_epoch': self.epoch})
        task = self.navigation_id = result['task_id']
        deadline = time.monotonic() + NAVIGATION_TIMEOUT_S
        while time.monotonic() < deadline:
            self.health()
            state = self.rpc('GET', '/v21/navigation/status', {'task_id': task})
            if timings is not None:
                timings['navigation'] = dict(task_id=task, status=state['status'],
                    error_code=state.get('error_code'), timing_s=state.get('timing_s', {}),
                    planning_diagnostics=state.get('planning_diagnostics', {}))
            if state['status'] == 'arrived' and state.get('stopped'):
                self.wait_stopped()
                p = self.pose()
                distance = np.linalg.norm([p[k]-goal[k] for k in ('x', 'y', 'z')])
                yaw = abs((p['yaw']-goal['yaw']+180) % 360-180)
                if distance > self.tolerances['position_tolerance_cm'] or yaw > self.tolerances['yaw_tolerance_deg']:
                    raise MissionError('arrival feedback outside tolerance')
                self.navigation_id = None
                return
            if state['status'] in ('failed', 'cancelled'):
                if state['status'] == 'failed' and state.get('error_code') == 'planning_failed':
                    with measure(timings, 'failed_plan_hold'):
                        self.wait_stopped()
                    self.navigation_id = None
                    raise NavigationPlanningFailed(state['error'])
                raise MissionError('navigation ' + str(state))
            time.sleep(.1)
        raise MissionError('navigation timeout')

    def stop(self):
        self.check_lease()
        h = self.rpc('GET', '/health')['health']
        if h.get('control_owner') != 'agent' or h.get('localization_epoch') != self.epoch:
            self.error = 'cannot stop after takeover or epoch change'
            raise ControlLost(self.error)
        task = h.get('active_task_id')
        if task:
            self.rpc('POST', '/v21/navigation/cancel', {'task_id': task})
        self.wait_stopped()
        self.navigation_id = None

    def land(self):
        self.health()
        self.landing = True
        result = self.rpc('POST', '/v22/land')
        task = result['task_id']
        # Follow the accepted task read-only even if console takes over its landing.
        deadline = time.monotonic() + 95.
        while time.monotonic() < deadline:
            state = self.rpc('GET', '/v21/navigation/status', {'task_id': task})
            if state['status'] == 'arrived' and state.get('stopped'):
                return
            if state['status'] in ('failed', 'cancelled'):
                raise MissionError('landing failed: ' + str(state))
            time.sleep(.2)
        raise MissionError('landing not confirmed')

    def close(self):
        # Release only our original session. Never acquire a replacement session.
        try:
            if self.session and not self.error:
                self.rpc('POST', '/v21/session/release')
        finally:
            self.done.set()
            if self.thread:
                self.thread.join(timeout=2.)
            self.session = None
