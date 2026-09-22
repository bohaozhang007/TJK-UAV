"""Stop-and-detect mission. All physical operations use the public Robot interface."""
import argparse
import base64
import enum
from http.client import HTTPException
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace
import uuid
import urllib.error
import urllib.request

import numpy as np
import yaml
from PIL import Image, ImageDraw

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from app.timing import measure
from app.robot_client.base import ControlLost, MissionError, Robot, TargetNotLocalizable, NavigationPlanningFailed
from app.robot_client.factory import create_robot
from app.robot_client.artifact_writer import ArtifactWriter, save_photo
from app.robot_client.depth import DepthClient, DEPTH_TIMEOUT_S


# Fixed service settings and bounded retry/loop limits.
DETECTOR_PORT = 8790
DETECTOR_TIMEOUT_S = 20.0
DETECTOR_HEALTH_TIMEOUT_S = 5.0
READ_ATTEMPTS = 2
READ_RETRY_DELAY_S = 0.5
MAX_STOPS_PER_WAYPOINT = 100


class State(enum.Enum):
    WAIT_OPERATOR = 'wait_operator'
    PATROL_PLAN = 'patrol_plan'
    PATROL_MOVE = 'patrol_move'
    DETECT = 'detect'
    LOCALIZE = 'localize'
    ORBIT_PLAN = 'orbit_plan'
    ORBIT_MOVE = 'orbit_move'
    PHOTO = 'photo'
    AUTOFOCUS = 'autofocus'
    RETURN_CAPTURE = 'return_capture'
    RETURN_HOME = 'return_home'
    LANDING = 'landing'
    COMPLETE = 'complete'
    ERROR_HOLD = 'error_hold'
    ERROR_LANDING = 'error_landing'
    FAILED = 'failed'
    CONTROL_LOST = 'control_lost'


NEXT = {
    State.WAIT_OPERATOR: {State.PATROL_PLAN},
    State.PATROL_PLAN: {State.PATROL_MOVE, State.RETURN_HOME},
    State.PATROL_MOVE: {State.DETECT},
    State.DETECT: {State.LOCALIZE},
    State.LOCALIZE: {State.AUTOFOCUS, State.ORBIT_PLAN, State.PATROL_PLAN, State.RETURN_HOME},
    State.AUTOFOCUS: {State.AUTOFOCUS, State.PATROL_PLAN, State.RETURN_HOME},
    State.ORBIT_PLAN: {State.ORBIT_MOVE, State.RETURN_CAPTURE},
    State.ORBIT_MOVE: {State.PHOTO, State.ORBIT_PLAN, State.RETURN_CAPTURE},
    State.PHOTO: {State.ORBIT_MOVE, State.ORBIT_PLAN, State.RETURN_CAPTURE},
    State.RETURN_CAPTURE: {State.ORBIT_PLAN, State.PATROL_PLAN, State.RETURN_HOME},
    State.RETURN_HOME: {State.LANDING},
    State.LANDING: {State.COMPLETE},
    State.ERROR_LANDING: {State.FAILED},
}


def wrap(yaw):
    return (yaw + 180) % 360 - 180


def stopping_point(path, goal, interval_cm, tolerance_cm):
    """Select by arc length; EGO will replan a zero-terminal-velocity stop here."""
    remaining = interval_cm
    for a, b in zip(path[:-1], path[1:]):
        distance = float(np.linalg.norm(b-a))
        if distance < 1e-6:
            continue
        if remaining <= distance:
            point = a + (b-a) * (remaining/distance)
            end = np.array([goal[k] for k in ('x', 'y', 'z')])
            arrived = np.linalg.norm(point-end) <= tolerance_cm
            if arrived:
                return dict(goal), True
            yaw = goal['yaw'] if arrived else math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
            return dict(zip(('x', 'y', 'z'), point.tolist()), yaw=wrap(yaw)), bool(arrived)
        remaining -= distance
    end = np.array([goal[k] for k in ('x', 'y', 'z')])
    if np.linalg.norm(path[-1]-end) <= tolerance_cm:
        return dict(goal), True
    raise MissionError('preview ends before the next detection stop and before the waypoint')


class Detector:
    def __init__(self, url):
        self.url = url.rstrip('/') + '/detect'
        self.health_url = url.rstrip('/') + '/health'
        self.timeout = DETECTOR_TIMEOUT_S
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def save_fallback_photo(self, path, rgb):
        """Detect and annotate the saved photo without persisting box coordinates."""
        path = Path(path)
        if rgb is not None:
            save_photo(path, rgb)
        annotated_path = path.with_name(path.stem + '_boxes.jpg')
        with Image.open(path) as source:
            image = source.convert('RGB')
        obs = SimpleNamespace(frame_id='fallback-photo:' + uuid.uuid4().hex,
                              rgb=np.array(image), metadata={},
                              image_base64=base64.b64encode(path.read_bytes()).decode('ascii'))
        self.health()
        detections = self.detect(obs)
        draw = ImageDraw.Draw(image)
        for index, detection in enumerate(detections, 1):
            box = detection['box']
            draw.rectangle(box, outline='red', width=3)
            draw.text((box[0], max(0, box[1]-14)),
                      f"{index}: {detection['confidence']:.3f}", fill='red')
        image.save(annotated_path, quality=95)
        return dict(annotated_file=annotated_path.name,
                    detection_status='detected' if detections else 'no_detections',
                    detection_count=len(detections))

    def health(self):
        try:
            with self.http.open(self.health_url, timeout=DETECTOR_HEALTH_TIMEOUT_S) as response:
                data = json.load(response)
                if response.status != 200 or not isinstance(data, dict) or data.get('ok') is not True:
                    raise ValueError('expected HTTP 200 with {"ok": true}')
        except (OSError, ValueError, HTTPException) as exc:
            raise MissionError(f'detector health check failed at {self.health_url}: {exc}') from exc

    def detect(self, obs, timings=None):
        with measure(timings, 'request_encode'):
            body = dict(image=obs.image_base64, frame_id=obs.frame_id)
            request = urllib.request.Request(self.url, json.dumps(body).encode(), {'Content-Type': 'application/json'})
        with measure(timings, 'http_round_trip'):
            try:
                with self.http.open(request, timeout=self.timeout) as response:
                    data = json.load(response)
            except urllib.error.HTTPError as exc:
                with exc:
                    data = json.load(exc)
                if timings is not None:
                    timings['server'] = data.get('timings', {})
                raise
        if timings is not None:
            timings['server'] = data.get('timings', {})
        with measure(timings, 'validate_and_decode_masks'):
            results = self.decode(data, obs)
            obs.metadata.pop('detector_image_cache', None)
            cached = data.get('image_cache')
            if (results and isinstance(cached,dict) and cached.get('frame_id') == obs.frame_id
                    and isinstance(cached.get('token'),str) and len(cached['token']) == 32
                    and all(c in '0123456789abcdef' for c in cached['token'])):
                obs.metadata['detector_image_cache'] = dict(cached)
            return results

    def decode(self, data, obs):
        if (data.get('ok') is not True or data.get('frame_id') != obs.frame_id
                or data.get('image_size') != [obs.rgb.shape[1], obs.rgb.shape[0]]):
            raise MissionError('detector response does not match exposure')
        items = data.get('detections')
        if not isinstance(items, list) or len(items) > 3:
            raise MissionError('invalid detector result count')
        results = []
        for item in items:
            score, mask = item['confidence'], item['mask']
            if not isinstance(score, (int, float)) or isinstance(score, bool) or not .5 < score <= 1:
                raise MissionError('invalid detector confidence')
            counts = mask.get('counts')
            if (mask.get('encoding') != 'rle' or mask.get('order') != 'C'
                    or mask.get('size') != list(obs.rgb.shape[:2]) or not isinstance(counts, list)
                    or not counts or any(type(n) is not int or n < 0 for n in counts)
                    or sum(counts) != obs.rgb.shape[0]*obs.rgb.shape[1]):
                raise MissionError('invalid mask encoding')
            decoded = np.repeat(np.arange(len(counts)) % 2 == 1, counts).reshape(mask['size'])
            if not decoded.any():
                raise MissionError('empty detector mask')
            box = np.asarray(item.get('box'), dtype=float)
            h, w = obs.rgb.shape[:2]
            if (box.shape != (4,) or not np.isfinite(box).all()
                    or not 0 <= box[0] < box[2] <= w or not 0 <= box[1] < box[3] <= h):
                raise MissionError('invalid detector box')
            results.append(dict(confidence=score, mask=decoded, box=box.tolist()))
        return sorted(results, key=lambda d: d['confidence'], reverse=True)


class Mission:
    def __init__(self, robot: Robot, detector, config, output, depth_client=None):
        self.robot, self.detector, self.c = robot, detector, config
        self.depth_client = depth_client
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.state = State.WAIT_OPERATOR
        self.targets = []
        self.current_target = None
        self.origin = None
        self.attached = False
        self.operation_id = 0
        self.detection_point_id = 0
        self.artifacts = ArtifactWriter()
        self.events = (self.output/'events.jsonl').open('a', encoding='utf-8', buffering=1)

    def event(self, kind, **fields):
        data = dict(time=time.time(), state=self.state.value, event=kind, **fields)
        self.events.write(json.dumps(data, allow_nan=False) + '\n')
        print(json.dumps(data, ensure_ascii=False), flush=True)

    def transition(self, state):
        if (state != self.state and state not in NEXT.get(self.state, set())
                and state not in (State.ERROR_HOLD, State.ERROR_LANDING, State.CONTROL_LOST)):
            raise RuntimeError(f'invalid state transition {self.state.value} -> {state.value}')
        self.state = state
        self.event('state')

    def guarded_wait(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.robot.health()
            time.sleep(min(.1, max(0., deadline-time.monotonic())))

    def retry_read(self, name, operation, **context):
        # Only observation/detection/preview operations; never replay navigation.
        self.operation_id += 1
        operation_id = self.operation_id
        if self.current_target is not None:
            context['target_id'] = self.current_target['id']
        for attempt in range(1, READ_ATTEMPTS + 1):
            timings = {}
            started = time.monotonic()
            fields = dict(operation=name, operation_id=operation_id, attempt=attempt, **context)
            self.event('operation_started', **fields)
            try:
                with measure(timings, 'health'):
                    self.robot.health()
                with measure(timings, 'call'):
                    result = operation(timings)
            except BaseException as exc:
                self.event('operation_finished', **fields, status='error',
                           elapsed_s=time.monotonic()-started, timings=timings,
                           error=str(exc) or type(exc).__name__)
                if isinstance(exc, ControlLost) or not isinstance(exc, Exception) or 'call' not in timings:
                    raise
                self.event('read_retry', **fields, error=str(exc), will_retry=attempt < READ_ATTEMPTS)
                if attempt == READ_ATTEMPTS:
                    if isinstance(exc, TargetNotLocalizable):
                        raise
                    raise MissionError(f'{name}: {exc}') from exc
                try:
                    with measure(timings, 'retry_wait'):
                        self.guarded_wait(READ_RETRY_DELAY_S)
                finally:
                    self.event('retry_wait', **fields, **timings['retry_wait'])
            else:
                self.event('operation_finished', **fields, status='ok',
                           elapsed_s=time.monotonic()-started, timings=timings)
                return result

    def attach(self):
        self.origin = self.robot.attach()
        self.attached = True
        self.event('attached', origin=self.origin)

    def mission_waypoint(self, waypoint):
        a = math.radians(self.origin['yaw'])
        x, y = waypoint['x_cm'], waypoint['y_cm']
        goal = dict(x=self.origin['x']+math.cos(a)*x-math.sin(a)*y,
                    y=self.origin['y']+math.sin(a)*x+math.cos(a)*y,
                    z=self.origin['z']+waypoint['z_cm'], yaw=wrap(self.origin['yaw']+waypoint['yaw_deg']))
        if goal['z'] < self.c['safety']['safe_z_cm']:
            raise MissionError('waypoint below safety height')
        return goal

    def plan_stop(self, goal):
        self.transition(State.PATROL_PLAN)
        path = self.retry_read('preview', lambda t: self.robot.preview_path(goal, timings=t), pose=goal)
        point, final = stopping_point(path, goal, self.c['patrol']['detection_stop_interval_m']*100,
                                      self.c['patrol']['waypoint_tolerance_cm'])
        return point, final

    def fly_to(self, pose, state):
        if pose['z'] < self.c['safety']['safe_z_cm']:
            raise MissionError('navigation goal below safety height')
        self.transition(state)
        self.event('navigate', pose=pose)
        started = time.monotonic()
        timings = {}
        try:
            self.robot.navigate(pose, timings=timings)
            self.robot.wait_stopped()
        except BaseException as exc:
            self.event('navigation_finished', pose=pose, status='error',
                       elapsed_s=time.monotonic()-started, timings=timings,
                       error=str(exc) or type(exc).__name__)
            raise
        self.event('navigation_finished', pose=pose, status='ok',
                   elapsed_s=time.monotonic()-started, timings=timings)

    def capture_observation(self):
        return self.retry_read('observation', lambda t: self.robot.observe(timings=t))

    def detect(self, observation):
        self.transition(State.DETECT)
        def request(timings):
            return self.wait_perception(lambda t: self.detector.detect(observation, timings=t),
                                        DETECTOR_TIMEOUT_S, timings)
        return self.retry_read('detection', request, frame_id=observation.frame_id,
                               detection_point_id=self.detection_point_id)

    def wait_perception(self, operation, timeout, timings):
        outcome = queue.Queue(maxsize=1)
        def worker():
            details = {}
            try:
                outcome.put((True, operation(details), details))
            except Exception as exc:
                outcome.put((False, exc, details))
        threading.Thread(target=worker, daemon=True).start()
        deadline = time.monotonic()+timeout+1.
        while time.monotonic() < deadline:
            self.robot.health()
            try:
                ok, result, details = outcome.get(timeout=.1)
            except queue.Empty:
                continue
            timings.update(details)
            if not ok:
                raise result
            return result
        raise MissionError('perception request timed out')

    def locate_new_targets(self, observation, detections):
        self.transition(State.LOCALIZE)
        new = []
        depth = None
        source = self.c['localization']['source']
        if detections and source == 'da3':
            if self.depth_client is None:
                raise MissionError('DA3 localization requires a depth client')
            depth = self.retry_read('depth estimation',
                lambda t: self.wait_perception(lambda details: self.depth_client.locate_targets(
                    observation, [d['mask'] for d in detections],
                    self.c['localization']['min_depth_pixels'],
                    self.c['localization']['max_relative_depth_mad'], details),
                                               DEPTH_TIMEOUT_S, t),
                frame_id=observation.frame_id, detection_point_id=self.detection_point_id)
        for index, detection in enumerate(detections, 1):
            attempts = [0]
            def locate(timings):
                attempts[0] += 1
                prefix = self.output / f'localize_{self.detection_point_id:03d}_{index:02d}_{attempts[0]:02d}'
                return self.robot.locate_target(observation, detection['mask'], timings=timings,
                                               diagnostic_prefix=prefix, depth=depth)
            context = dict(frame_id=observation.frame_id, detection_point_id=self.detection_point_id,
                           detection_index=index, localization_source=source,
                           confidence=detection['confidence'])
            try:
                position = self.retry_read('target localization',
                    locate,
                    **context)
            except TargetNotLocalizable as exc:
                self.event('detection_skipped', **context, reason=str(exc))
                continue
            existing = self.duplicate_target(position)
            if existing is not None:
                if (self.c['photography']['autofocus'] and existing['status'] == 'failed'
                        and existing.get('autofocus_failed')
                        and existing.get('autofocus_frame_id') != observation.frame_id):
                    existing.update(status='pending', exposure_pose=dict(observation.pose),
                                    confidence=detection['confidence'])
                    new.append((existing, detection['box']))
                    self.event('autofocus_revisit', target_id=existing['id'],
                               frame_id=observation.frame_id, position_cm=position,
                               detection_point_id=self.detection_point_id)
                    continue
                self.event('duplicate', target_id=existing['id'], position_cm=position)
                continue
            record = dict(id=len(self.targets)+1, position_cm=position, status='pending',
                          confidence=detection['confidence'], exposure_pose=dict(observation.pose),
                          localization_source=source)
            self.targets.append(record)
            new.append((record, detection['box']))
            self.event('new_target', target=record)
        return new

    def duplicate_target(self, position):
        threshold = self.c['patrol']['dedup_distance_cm']
        mode = self.c['patrol'].get('dedup_mode', '3d')
        for record in self.targets:
            delta = np.asarray(position) - record['position_cm']
            if mode == 'separate':
                duplicate = np.all(np.abs(delta) < threshold)
            else:
                duplicate = np.linalg.norm(delta) < threshold
            if duplicate:
                return record
        return None

    def orbit_points(self, target, exposure_pose):
        x, y, z = target['position_cm']
        z = max(self.c['safety']['safe_z_cm'], z)
        radius = self.c['orbit']['radius_m'] * 100
        count = self.c['orbit'].get('all_cand', 6)
        start = math.atan2(exposure_pose['y']-y, exposure_pose['x']-x)
        points = []
        for i in range(count):
            angle = start + i * math.tau / count  # Clockwise in the public Y-right frame.
            px, py = x + radius*math.cos(angle), y + radius*math.sin(angle)
            points.append(dict(x=px, y=py, z=z, yaw=wrap(math.degrees(math.atan2(y-py, x-px)))))
        return points

    def orbit_candidates(self, target, exposure_pose):
        self.transition(State.ORBIT_PLAN)
        points = self.orbit_points(target, exposure_pose)
        free = self.retry_read('orbit point batch query',
            lambda t: self.robot.points_are_free(points, timings=t))
        candidates = []
        for index, (point, available) in enumerate(zip(points, free)):
            self.event('orbit_candidate', target_id=target['id'], candidate_index=index+1,
                       pose=point, available=available)
            if available:
                candidates.append(dict(index=index, pose=point))
        return candidates

    def take_photo(self, target, index):
        self.transition(State.PHOTO)
        obs = self.capture_observation()
        path = self.output / f'target_{target["id"]:03d}_{index:02d}.jpg'
        if not self.artifacts.submit('photo', path, save_photo, obs.rgb):
            raise MissionError('photo storage queue full')
        self.event('photo', target_id=target['id'], file=path.name, pose=obs.pose,
                   frame_id=obs.frame_id, write_status='queued')
        self.guarded_wait(self.c['orbit']['dwell_s'])

    def visit_target(self, target):
        self.current_target = target
        candidates = self.orbit_candidates(target, target['exposure_pose'])
        limit = self.c['orbit'].get('top', 6)
        photos = 0
        reference_pose = dict(target['exposure_pose'])
        while candidates and photos < limit:
            self.transition(State.ORBIT_PLAN)
            candidates.sort(key=lambda p: (
                abs(wrap(p['pose']['yaw']-reference_pose['yaw'])),
                sum((p['pose'][k]-reference_pose[k])**2 for k in ('x', 'y', 'z')),
                p['index']))
            candidate = candidates.pop(0)
            point = candidate['pose']
            self.event('orbit_selection', target_id=target['id'],
                       candidate_indices=[candidate['index']+1], completed_photos=photos,
                       reference_pose=reference_pose,
                       yaw_delta_deg=abs(wrap(point['yaw']-reference_pose['yaw'])))
            context = dict(target_id=target['id'], candidate_index=candidate['index']+1, pose=point)
            if not self.retry_read('map point recheck',
                    lambda t: self.robot.point_is_free(point, timings=t),
                    pose=point, candidate_index=candidate['index']+1):
                self.event('orbit_point_skipped', **context, reason='point no longer free in current map')
            else:
                try:
                    self.fly_to(point, State.ORBIT_MOVE)
                except NavigationPlanningFailed as exc:
                    self.event('orbit_point_skipped', **context, reason=str(exc))
                else:
                    photos += 1
                    self.take_photo(target, photos)
            if candidates and photos < limit:
                reference_pose = self.robot.pose()
        if not photos:
            raise MissionError('zero reachable orbit points')
        self.fly_to(target['exposure_pose'], State.RETURN_CAPTURE)
        target['status'] = 'completed'
        self.event('target_completed', target=target, photo_count=photos)
        self.current_target = None

    def photograph_target(self, target, observation, box):
        self.current_target = target
        self.transition(State.AUTOFOCUS)
        timings = {}
        attempt = target.get('autofocus_attempts', 0)+1
        target.update(autofocus_attempts=attempt, autofocus_frame_id=observation.frame_id)
        self.event('autofocus_started', target_id=target['id'], frame_id=observation.frame_id,
                   attempt=attempt)
        try:
            result = self.robot.autofocus_photo(observation, box, timings=timings)
        except Exception as exc:
            self.event('autofocus_failed', target_id=target['id'], frame_id=observation.frame_id,
                       error=str(exc), timings=timings)
            raise
        path = self.output / f'target_{target["id"]:03d}_{attempt:02d}.jpg'
        if result['focused']:
            submitted = self.artifacts.submit('photo', path, save_photo, result['rgb'])
        else:
            submitted = self.artifacts.submit('photo', path, self.detector.save_fallback_photo,
                result['rgb'])
        if not submitted:
            raise MissionError('photo storage queue full')
        target['status'] = 'completed' if result['focused'] else 'failed'
        target['autofocus_failed'] = not result['focused']
        self.event('autofocus_finished', target=target, focused=result['focused'],
                   fallback_photo=not result['focused'], file=path.name, timings=timings,
                   write_status='queued')
        self.current_target = None

    def patrol(self):
        for waypoint in self.c['mission']['waypoints']:
            goal = self.mission_waypoint(waypoint)
            for _ in range(MAX_STOPS_PER_WAYPOINT):
                stop, final = self.plan_stop(goal)
                self.fly_to(stop, State.PATROL_MOVE)
                self.detection_point_id += 1
                observation = self.capture_observation()
                self.event('detection_point', detection_point_id=self.detection_point_id,
                           frame_id=observation.frame_id, exposure_pose=dict(observation.pose))
                detections = self.detect(observation)
                targets = self.locate_new_targets(observation, detections)
                # All identities/positions were computed while still at this exposure P.
                for target, box in targets:
                    if self.c['photography']['autofocus']:
                        self.photograph_target(target, observation, box)
                    else:
                        self.visit_target(target)
                if final:
                    break
            else:
                raise MissionError('waypoint exceeded the stop budget')

    def return_home(self):
        self.fly_to(self.origin, State.RETURN_HOME)

    def land(self):
        self.transition(State.LANDING)
        self.robot.land()
        self.transition(State.COMPLETE)

    def handle_error(self, error):
        for target in self.targets:
            if target['status'] == 'pending':
                target['status'] = 'failed'
        action = self.c['safety']['error_action']
        self.event('error', error=str(error) or type(error).__name__, action=action, targets=self.targets)
        if action == 'land':
            self.error_land()
        else:
            self.error_hold()

    def error_land(self):
        self.transition(State.ERROR_LANDING)
        try:
            self.robot.land()
        except ControlLost:
            raise
        except Exception as exc:
            # Acceptance may be uncertain; never replay or cancel this landing.
            self.event('landing_unconfirmed', error=str(exc))
            raise
        self.transition(State.FAILED)
        self.event('error_landing_confirmed')

    def error_hold(self):
        self.transition(State.ERROR_HOLD)
        try:
            self.robot.stop()
            self.event('hold_confirmed', message='Use console stop/land or pilot takeover; no automatic landing.')
        except ControlLost:
            raise
        except Exception as exc:
            self.event('hold_unconfirmed', error=str(exc))
        while True:
            try:
                self.robot.health()
            except ControlLost:
                raise
            except Exception as exc:
                self.event('hold_health_unavailable', error=str(exc))
            time.sleep(1.)

    def run(self):
        try:
            self.attach()
            self.patrol()
            self.return_home()
            self.land()
        except ControlLost as exc:
            self.transition(State.CONTROL_LOST)
            self.event('control_lost', error=str(exc))
        except (Exception, KeyboardInterrupt) as exc:
            if not self.attached or self.state == State.LANDING:
                raise
            try:
                self.handle_error(exc)
            except ControlLost as lost:
                self.transition(State.CONTROL_LOST)
                self.event('control_lost', error=str(lost))
            except KeyboardInterrupt:
                self.event('error_monitor_stopped')
        finally:
            if self.state != State.COMPLETE:
                for target in self.targets:
                    if target['status'] == 'pending':
                        target['status'] = 'failed'
                self.event('mission_ended', targets=self.targets)
            try:
                self.robot.close()
            finally:
                try:
                    self.artifacts.close()
                finally:
                    self.events.close()


def validate_config(config):
    if config['localization'].get('source') not in ('da3', 'grid'):
        raise ValueError('localization.source must be da3 or grid')
    minimum = config['localization']['min_depth_pixels']
    spread = config['localization']['max_relative_depth_mad']
    if type(minimum) is not int or not 1 <= minimum <= 1000000:
        raise ValueError('invalid localization.min_depth_pixels')
    if type(spread) not in (int,float) or not math.isfinite(spread) or not 0 < spread <= 1:
        raise ValueError('invalid localization.max_relative_depth_mad')
    if type(config.get('photography', {}).get('autofocus')) is not bool:
        raise ValueError('photography.autofocus must be boolean')
    if config['patrol'].get('dedup_mode', '3d') not in ('3d', 'separate'):
        raise ValueError('patrol.dedup_mode must be 3d or separate')
    if config['safety']['error_action'] not in ('hold', 'land'):
        raise ValueError('safety.error_action must be hold or land')
    positive = {
        'localization': ('target_depth_gap_m',), 'safety': ('safe_z_cm',),
        'patrol': ('detection_stop_interval_m','dedup_distance_cm','waypoint_tolerance_cm'),
        'orbit': ('radius_m','dwell_s'),
    }
    for section, keys in positive.items():
        for key in keys:
            value = config[section][key]
            if type(value) not in (int,float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{section}.{key} must be finite and positive')
    value = config['localization']['min_target_voxels']
    if type(value) is not int or not 1 <= value <= 1000:
        raise ValueError('invalid localization.min_target_voxels')
    all_cand = config['orbit'].get('all_cand', 6)
    top = config['orbit'].get('top', 6)
    if type(all_cand) is not int or type(top) is not int or not 1 <= top <= all_cand:
        raise ValueError('orbit.all_cand and orbit.top must be integers with 1 <= top <= all_cand')
    waypoints = config['mission']['waypoints']
    if not isinstance(waypoints,list) or not waypoints:
        raise ValueError('mission needs at least one waypoint')
    for waypoint in waypoints:
        for key in ('x_cm','y_cm','z_cm','yaw_deg'):
            value = waypoint[key]
            if type(value) not in (int,float) or not math.isfinite(value):
                raise ValueError('waypoint coordinates must be finite numbers')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1]/'config/v22.yaml')
    parser.add_argument('--detector-host', required=True, help='Windows detector IP')
    parser.add_argument('--output', type=Path, default=Path('logs')/('v22_'+time.strftime('%Y%m%d_%H%M%S')))
    args = parser.parse_args()
    with args.config.open(encoding='utf-8') as file:
        config = yaml.safe_load(file)
    try:
        validate_config(config)
    except (ValueError,KeyError,TypeError) as exc:
        parser.error(str(exc))
    detector = Detector(f'http://{args.detector_host}:{DETECTOR_PORT}')
    print(f'Checking detector: {detector.health_url} (timeout={DETECTOR_HEALTH_TIMEOUT_S:g}s)', flush=True)
    started = time.monotonic()
    try:
        detector.health()
    except MissionError as exc:
        parser.exit(1, f'Error: {exc}; mission not started.\n')
    print(f'Detector health OK ({time.monotonic()-started:.3f}s)', flush=True)
    try:
        robot = create_robot(config['localization'], autofocus=config['photography']['autofocus'],
                             tracker_host=args.detector_host)
    except (ValueError,KeyError,TypeError) as exc:
        parser.error(str(exc))
    depth_client = DepthClient(args.detector_host) if config['localization']['source'] == 'da3' else None
    Mission(robot, detector, config, args.output, depth_client=depth_client).run()


if __name__ == '__main__':
    main()
