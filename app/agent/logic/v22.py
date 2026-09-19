"""Stop-and-detect mission. All physical operations use the public Robot interface."""
import argparse
import enum
from http.client import HTTPException
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
import urllib.error
import urllib.request

import numpy as np
import yaml
from PIL import Image

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from app.timing import measure
from app.robot_client.base import ControlLost, MissionError, Robot
from app.robot_client.factory import create_robot


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
    State.LOCALIZE: {State.ORBIT_PLAN, State.PATROL_PLAN, State.RETURN_HOME},
    State.ORBIT_PLAN: {State.ORBIT_MOVE},
    State.ORBIT_MOVE: {State.PHOTO},
    State.PHOTO: {State.ORBIT_MOVE, State.RETURN_CAPTURE},
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
            body = dict(image=obs.jpeg_base64, frame_id=obs.frame_id)
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
            return self.decode(data, obs)

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
            results.append(dict(confidence=score, mask=decoded))
        return sorted(results, key=lambda d: d['confidence'], reverse=True)


class Mission:
    def __init__(self, robot: Robot, detector, config, output):
        self.robot, self.detector, self.c = robot, detector, config
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.state = State.WAIT_OPERATOR
        self.targets = []
        self.current_target = None
        self.origin = None
        self.attached = False
        self.operation_id = 0
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
        self.retry_read('stop preview', lambda t: self.robot.preview_path(point, require_arrival=True, timings=t), pose=point)
        return point, final

    def fly_to(self, pose, state):
        if pose['z'] < self.c['safety']['safe_z_cm']:
            raise MissionError('navigation goal below safety height')
        self.transition(state)
        self.event('navigate', pose=pose)
        self.robot.navigate(pose)
        self.robot.wait_stopped()

    def capture_observation(self):
        return self.retry_read('observation', lambda t: self.robot.observe())

    def detect(self, observation):
        self.transition(State.DETECT)
        def request(timings):
            outcome = queue.Queue(maxsize=1)
            def worker():
                details = {}
                try:
                    result = self.detector.detect(observation, timings=details)
                    outcome.put((True, result, details))
                except Exception as exc:
                    outcome.put((False, exc, details))
            threading.Thread(target=worker, daemon=True).start()
            deadline = time.monotonic() + DETECTOR_TIMEOUT_S + 1.
            while time.monotonic() < deadline:
                self.robot.health()
                try:
                    ok, result, details = outcome.get(timeout=.1)
                    timings.update(details)
                except queue.Empty:
                    continue
                if not ok:
                    raise result
                return result
            raise MissionError('detector request timed out')
        return self.retry_read('detection', request, frame_id=observation.frame_id)

    def locate_new_targets(self, observation, detections):
        self.transition(State.LOCALIZE)
        new = []
        for detection in detections:
            position = self.retry_read('target localization',
                lambda t: self.robot.locate_target(observation, detection['mask'], timings=t),
                frame_id=observation.frame_id)
            if any(np.linalg.norm(np.asarray(position)-record['position_cm']) < self.c['patrol']['dedup_distance_cm']
                   for record in self.targets):
                self.event('duplicate', position_cm=position)
                continue
            record = dict(id=len(self.targets)+1, position_cm=position, status='pending',
                          confidence=detection['confidence'], exposure_pose=dict(observation.pose))
            self.targets.append(record)
            new.append(record)
            self.event('new_target', target=record)
        return new

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

    def reachable_orbit(self, target, exposure_pose):
        self.transition(State.ORBIT_PLAN)
        reachable = []
        for point in self.orbit_points(target, exposure_pose):
            if not self.retry_read('map point query', lambda t: self.robot.point_is_free(point, timings=t), pose=point):
                continue
            try:
                self.retry_read('orbit preview', lambda t: self.robot.preview_path(point, require_arrival=True, timings=t), pose=point)
            except ControlLost:
                raise
            except MissionError as exc:
                self.event('unreachable_orbit_point', pose=point, error=str(exc))
                continue
            reachable.append(point)
        if not reachable:
            raise MissionError('zero reachable orbit points')
        ranked = sorted(range(len(reachable)), key=lambda i: sum(
            (reachable[i][k]-exposure_pose[k])**2 for k in ('x', 'y', 'z')))
        selected = sorted(ranked[:self.c['orbit'].get('top', 6)])
        start = selected.index(ranked[0])
        # Keep clockwise order, starting at the nearest selected point.
        return [reachable[i] for i in selected[start:] + selected[:start]]

    def take_photo(self, target, index):
        self.transition(State.PHOTO)
        obs = self.capture_observation()
        path = self.output / f'target_{target["id"]:03d}_{index:02d}.jpg'
        Image.fromarray(obs.rgb).save(path, quality=95)
        self.event('photo', target_id=target['id'], file=path.name, pose=obs.pose, frame_id=obs.frame_id)
        self.guarded_wait(self.c['orbit']['dwell_s'])

    def visit_target(self, target):
        self.current_target = target
        points = self.reachable_orbit(target, target['exposure_pose'])
        for index, point in enumerate(points):
            # Recheck each segment from the actual current stop, not only from P.
            if not self.retry_read('map point recheck', lambda t: self.robot.point_is_free(point, timings=t), pose=point):
                raise MissionError('orbit point became occupied or left map coverage')
            self.retry_read('orbit segment preview', lambda t: self.robot.preview_path(point, require_arrival=True, timings=t), pose=point)
            self.fly_to(point, State.ORBIT_MOVE)
            self.take_photo(target, index+1)
        self.fly_to(target['exposure_pose'], State.RETURN_CAPTURE)
        target['status'] = 'completed'
        self.event('target_completed', target=target)
        self.current_target = None

    def patrol(self):
        for waypoint in self.c['mission']['waypoints']:
            goal = self.mission_waypoint(waypoint)
            for _ in range(MAX_STOPS_PER_WAYPOINT):
                stop, final = self.plan_stop(goal)
                self.fly_to(stop, State.PATROL_MOVE)
                observation = self.capture_observation()
                detections = self.detect(observation)
                targets = self.locate_new_targets(observation, detections)
                # All identities/positions were computed while still at this exposure P.
                for target in targets:
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
                target['status'] = 'failed'  # Never automatically revisit failed targets.
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
                self.events.close()


def validate_config(config):
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
        robot = create_robot(config['localization'])
    except (ValueError,KeyError,TypeError) as exc:
        parser.error(str(exc))
    Mission(robot, detector, config, args.output).run()


if __name__ == '__main__':
    main()
