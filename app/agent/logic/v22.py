"""Stop-and-detect mission. All physical operations use the public Robot interface."""
import argparse
import base64
import enum
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

from app.robot_client.base import ControlLost, MissionError, Robot
from app.robot_client.factory import create_robot


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
    def __init__(self, url, image, box, timeout_s):
        self.url = url.rstrip('/') + '/detect'
        self.reference = base64.b64encode(Path(image).read_bytes()).decode()
        self.box = [float(v) for v in Path(box).read_text(encoding='utf-8').split()]
        if len(self.box) != 4:
            raise ValueError('reference box requires x1 y1 x2 y2')
        with Image.open(image) as reference:
            x1, y1, x2, y2 = self.box
            if (not all(math.isfinite(v) for v in self.box)
                    or not 0 <= x1 < x2 <= reference.width or not 0 <= y1 < y2 <= reference.height):
                raise ValueError('reference box is outside the reference image')
        self.timeout = timeout_s
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def detect(self, obs):
        body = dict(image=obs.jpeg_base64, reference_image=self.reference,
                    box_xyxy=self.box, frame_id=obs.frame_id)
        request = urllib.request.Request(self.url, json.dumps(body).encode(), {'Content-Type': 'application/json'})
        with self.http.open(request, timeout=self.timeout) as response:
            data = json.load(response)
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
        self.events = (self.output/'events.jsonl').open('a', encoding='utf-8', buffering=1)

    def event(self, kind, **fields):
        data = dict(time=time.time(), state=self.state.value, event=kind, **fields)
        self.events.write(json.dumps(data, allow_nan=False) + '\n')
        print(json.dumps(data, ensure_ascii=False), flush=True)

    def transition(self, state):
        if (state != self.state and state not in NEXT.get(self.state, set())
                and state not in (State.ERROR_HOLD, State.CONTROL_LOST)):
            raise RuntimeError(f'invalid state transition {self.state.value} -> {state.value}')
        self.state = state
        self.event('state')

    def guarded_wait(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.robot.health()
            time.sleep(min(.1, max(0., deadline-time.monotonic())))

    def retry_read(self, name, operation):
        # Only observation/detection/preview operations; never replay navigation.
        for attempt in range(self.c['runtime']['read_attempts']):
            self.robot.health()
            try:
                return operation()
            except ControlLost:
                raise
            except Exception as exc:
                self.event('read_retry', operation=name, attempt=attempt+1, error=str(exc))
                if attempt + 1 == self.c['runtime']['read_attempts']:
                    raise MissionError(f'{name}: {exc}') from exc
                self.guarded_wait(self.c['runtime']['retry_delay_s'])

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
        path = self.retry_read('preview', lambda: self.robot.preview_path(goal))
        point, final = stopping_point(path, goal, self.c['patrol']['detection_stop_interval_m']*100,
                                      self.c['patrol']['waypoint_tolerance_cm'])
        self.retry_read('stop preview', lambda: self.robot.preview_path(point, require_arrival=True))
        return point, final

    def fly_to(self, pose, state):
        if pose['z'] < self.c['safety']['safe_z_cm']:
            raise MissionError('navigation goal below safety height')
        self.transition(state)
        self.event('navigate', pose=pose)
        self.robot.navigate(pose)
        self.robot.wait_stopped()

    def capture_observation(self):
        return self.retry_read('observation', self.robot.observe)

    def detect(self, observation):
        self.transition(State.DETECT)
        def request():
            outcome = queue.Queue(maxsize=1)
            def worker():
                try:
                    outcome.put((True, self.detector.detect(observation)))
                except Exception as exc:
                    outcome.put((False, exc))
            threading.Thread(target=worker, daemon=True).start()
            deadline = time.monotonic() + self.c['detector']['timeout_s'] + 1.
            while time.monotonic() < deadline:
                self.robot.health()
                try:
                    ok, result = outcome.get(timeout=.1)
                except queue.Empty:
                    continue
                if not ok:
                    raise result
                return result
            raise MissionError('detector request timed out')
        return self.retry_read('detection', request)

    def locate_new_targets(self, observation, detections):
        self.transition(State.LOCALIZE)
        new = []
        for detection in detections:
            position = self.retry_read('target localization',
                lambda: self.robot.locate_target(observation, detection['mask']))
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
        start = math.atan2(exposure_pose['y']-y, exposure_pose['x']-x)
        points = []
        for i in range(6):
            angle = start + i * math.tau / 6  # Clockwise in the public Y-right frame.
            px, py = x + radius*math.cos(angle), y + radius*math.sin(angle)
            points.append(dict(x=px, y=py, z=z, yaw=wrap(math.degrees(math.atan2(y-py, x-px)))))
        return points

    def reachable_orbit(self, target, exposure_pose):
        self.transition(State.ORBIT_PLAN)
        reachable = []
        for point in self.orbit_points(target, exposure_pose):
            if not self.retry_read('map point query', lambda: self.robot.point_is_free(point)):
                continue
            try:
                self.retry_read('orbit preview', lambda: self.robot.preview_path(point, require_arrival=True))
            except ControlLost:
                raise
            except MissionError as exc:
                self.event('unreachable_orbit_point', pose=point, error=str(exc))
                continue
            reachable.append(point)
        if not reachable:
            raise MissionError('zero reachable orbit points')
        nearest = min(range(len(reachable)), key=lambda i: sum(
            (reachable[i][k]-exposure_pose[k])**2 for k in ('x', 'y', 'z')))
        return reachable[nearest:] + reachable[:nearest]

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
            if not self.retry_read('map point recheck', lambda: self.robot.point_is_free(point)):
                raise MissionError('orbit point became occupied or left map coverage')
            self.retry_read('orbit segment preview', lambda: self.robot.preview_path(point, require_arrival=True))
            self.fly_to(point, State.ORBIT_MOVE)
            self.take_photo(target, index+1)
        self.fly_to(target['exposure_pose'], State.RETURN_CAPTURE)
        target['status'] = 'completed'
        self.event('target_completed', target=target)
        self.current_target = None

    def patrol(self):
        for waypoint in self.c['mission']['waypoints']:
            goal = self.mission_waypoint(waypoint)
            for _ in range(self.c['patrol']['max_stops_per_waypoint']):
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

    def error_hold(self, error):
        self.transition(State.ERROR_HOLD)
        for target in self.targets:
            if target['status'] == 'pending':
                target['status'] = 'failed'  # Never automatically revisit failed targets.
        self.event('error', error=str(error), targets=self.targets)
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
        except KeyboardInterrupt:
            if self.attached:
                self.robot.stop()
        except Exception as exc:
            if not self.attached or self.state == State.LANDING:
                raise
            try:
                self.error_hold(exc)
            except ControlLost as lost:
                self.transition(State.CONTROL_LOST)
                self.event('control_lost', error=str(lost))
            except KeyboardInterrupt:
                self.event('hold_monitor_stopped')
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
    positive = {
        'robot': ('attach_timeout_s','stop_timeout_s','preview_timeout_s','navigation_timeout_s',
                  'target_depth_gap_m'),
        'detector': ('timeout_s',), 'runtime': ('retry_delay_s',), 'safety': ('safe_z_cm',),
        'patrol': ('detection_stop_interval_m','dedup_distance_cm','waypoint_tolerance_cm'),
        'orbit': ('radius_m','dwell_s'),
    }
    for section, keys in positive.items():
        for key in keys:
            value = config[section][key]
            if type(value) not in (int,float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{section}.{key} must be finite and positive')
    for section, key, maximum in (('runtime','read_attempts',5), ('robot','min_target_voxels',1000),
                                ('patrol','max_stops_per_waypoint',10000),('detector','port',65535)):
        value = config[section][key]
        if type(value) is not int or not 1 <= value <= maximum:
            raise ValueError(f'invalid {section}.{key}')
    if type(config['robot']['allow_approximate_geometry']) is not bool:
        raise ValueError('allow_approximate_geometry must be boolean')
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
    parser.add_argument('--img', type=Path, required=True, help='reference image on this machine')
    parser.add_argument('--box', type=Path, required=True, help='reference xyxy pixel box text file')
    parser.add_argument('--output', type=Path, default=Path('logs')/('v22_'+time.strftime('%Y%m%d_%H%M%S')))
    args = parser.parse_args()
    with args.config.open(encoding='utf-8') as file:
        config = yaml.safe_load(file)
    try:
        validate_config(config)
        robot = create_robot(config['robot'])
    except (ValueError,KeyError,TypeError) as exc:
        parser.error(str(exc))
    detector = Detector(f'http://{args.detector_host}:{config["detector"]["port"]}',
                        args.img, args.box, config['detector']['timeout_s'])
    Mission(robot, detector, config, args.output).run()


if __name__ == '__main__':
    main()
