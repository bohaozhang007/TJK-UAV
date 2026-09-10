#!/usr/bin/env python3
"""HTTP-only B/cancel/P/TRACK/P/B test. Default GET-only; no ROS or Agent imports."""
import argparse
import json
import math
from pathlib import Path
import queue
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid


class Failure(RuntimeError):
    pass


def offset(pose, x=0, y=0, z=0, yaw=0):
    a = math.radians(pose['yaw'])
    return dict(x=pose['x']+math.cos(a)*x-math.sin(a)*y,
                y=pose['y']+math.sin(a)*x+math.cos(a)*y, z=pose['z']+z,
                yaw=(pose['yaw']+yaw+180)%360-180)


def distance(a, b):
    return math.sqrt(sum((a[k]-b[k])**2 for k in ('x', 'y', 'z')))


class Runner:
    def __init__(self, args):
        self.a = args
        self.sid = None
        self.epoch = None
        self.phase = 'preflight'
        self.stop = threading.Event()
        self.hb_error = None
        self.hb_thread = None
        self.log_lock = threading.Lock()
        folder = Path(args.output)
        folder.mkdir(parents=True, exist_ok=True)
        self.log = (folder/'events.jsonl').open('x', buffering=1)
        self.summary = folder/'result.json'

    def record(self, event, **data):
        with self.log_lock:
            self.log.write(json.dumps(dict(time_s=time.time(), monotonic_s=time.monotonic(),
                                         phase=self.phase, event=event, **data), allow_nan=False)+'\n')

    def rpc(self, method, path, data=None, timeout=2):
        if method != 'GET' and not self.a.execute:
            raise Failure('Preview cannot send POST')
        begin = time.monotonic()
        req = urllib.request.Request(self.a.url.rstrip('/')+path, method=method,
            data=json.dumps(data).encode() if data is not None else None,
            headers={'Content-Type': 'application/json'})
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            with opener.open(req, timeout=timeout) as response:
                result = json.load(response)
        except urllib.error.HTTPError as e:
            result = json.load(e)
            self.record('http_error', path=path, response=result, latency_s=time.monotonic()-begin)
            raise Failure('HTTP %s %s: %s' % (e.code, path, result)) from e
        except Exception as e:
            self.record('transport_error', path=path, error=str(e))
            raise Failure('HTTP outcome uncertain: '+path+': '+str(e)) from e
        self.record('http', method=method, path=path, response=result, latency_s=time.monotonic()-begin)
        if result.get('ok') is not True:
            raise Failure(str(result))
        return result

    def post(self, path, **data):
        return self.rpc('POST', path, dict(session_id=self.sid, request_id=str(uuid.uuid4()), **data))

    def health(self, flight=True):
        h = self.rpc('GET', '/health')['health']
        if self.hb_error:
            raise Failure('Heartbeat failed: '+self.hb_error)
        if self.epoch and h.get('localization_epoch') != self.epoch:
            raise Failure('Localization epoch changed')
        if not h.get('odom_ok') or h.get('manual_takeover') or h.get('conflicting_publishers'):
            raise Failure('Localization/control unavailable: '+str(h))
        if flight:
            if not all(h.get(k) for k in ('initialized', 'airborne', 'control_ready', 'hold_ready')) or h.get('error'):
                raise Failure('Flight authority unavailable: '+str(h))
            if h.get('planner_state') not in ('starting', 'ready', 'not_required'):
                raise Failure('Planner unhealthy: '+str(h))
        return h

    def pose(self):
        value = self.rpc('GET', '/get_pose')
        if self.epoch and value['localization_epoch'] != self.epoch:
            raise Failure('Pose epoch changed')
        p = value['pose']
        if set(p) != {'x', 'y', 'z', 'yaw'} or any(type(v) not in (float, int) or not math.isfinite(v) for v in p.values()):
            raise Failure('Invalid pose')
        return p

    def heartbeat(self):
        # Deadline scheduling keeps the target period at 0.5 s, including HTTP time.
        due = time.monotonic()
        while not self.stop.is_set():
            try:
                self.rpc('POST', '/v21/heartbeat', dict(session_id=self.sid))
            except Exception as e:
                self.hb_error = str(e)
                return
            due += .5
            if self.stop.wait(max(0, due-time.monotonic())):
                return

    def stage(self, name):
        self.phase = name
        self.record('stage')
        print(name, flush=True)
        if self.a.step:
            answer = input('Enter 继续；输入 q 中止（等待期间心跳继续）: ')
            if answer.strip().lower() == 'q':
                raise Failure('Operator aborted')
        self.health()

    def task(self, tid):
        return self.rpc('GET', '/v21/navigation/status?'+urllib.parse.urlencode({'task_id': tid}))

    def wait_task(self, tid, allowed=('arrived',), cleanup=False):
        deadline = time.monotonic()+(8 if cleanup else self.a.nav_timeout_s)
        while time.monotonic() < deadline:
            if not cleanup:
                self.health()
            task = self.task(tid)
            if task['status'] in ('failed', 'arrived', 'cancelled'):
                if task['status'] in allowed and task.get('stopped') is True:
                    return task
                raise Failure('Task did not complete successfully: '+str(task))
            time.sleep(.1)
        raise Failure('Task/stop confirmation timed out: '+tid)

    def navigate(self, goal):
        self.health()
        return self.post('/v21/navigation', pose=goal, localization_epoch=self.epoch)['task_id']

    def arrive(self, goal):
        tid = self.navigate(goal)
        task = self.wait_task(tid)
        actual = self.pose()
        if (distance(actual, goal) > self.tolerances['position_tolerance_cm'] or
                abs((actual['yaw']-goal['yaw']+180)%360-180) > self.tolerances['yaw_tolerance_deg']):
            raise Failure('Reported arrival disagrees with measured pose')
        self.record('arrival', goal=goal, actual=actual, task=task)
        return tid

    def blocking(self, path, timeout, flight=True, **data):
        # Worker owns the blocking HTTP call; foreground remains cancellable.
        result = queue.Queue()
        payload = dict(session_id=self.sid, request_id=str(uuid.uuid4()), **data)
        def call():
            try:
                result.put(self.rpc('POST', path, payload, timeout=timeout))
            except Exception as e:
                result.put(e)
        threading.Thread(target=call, daemon=True).start()
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.health(flight=flight)
            try:
                value = result.get(timeout=.1)
                if isinstance(value, Exception):
                    raise value
                return value
            except queue.Empty:
                pass
        raise Failure('Blocking operation timed out: '+path)

    def cleanup(self):
        if not self.sid:
            return True
        confirmed = True
        self.phase = 'cleanup'
        # Preserve heartbeat until stop confirmation attempt finishes. Never
        # issue automatic landing or reinitialize after failure/manual takeover.
        try:
            h = self.rpc('GET', '/health')['health']
            if h.get('active_task_id') and not h.get('manual_takeover'):
                tid = h['active_task_id']
                self.post('/v21/navigation/cancel', task_id=tid)
                self.wait_task(tid, ('cancelled', 'arrived'), cleanup=True)
            elif h.get('airborne') and not h.get('manual_takeover'):
                deadline = time.monotonic()+8
                while not (h.get('stopped') and h.get('hold_ready')):
                    if not h.get('hold_ready') or time.monotonic() >= deadline:
                        raise Failure('No confirmed hold after failure')
                    time.sleep(.1)
                    h = self.rpc('GET', '/health')['health']
        except Exception as e:
            confirmed = False
            self.record('stop_unconfirmed', error=str(e))
            print('停止未获确认；请用遥控器接管。'+str(e), flush=True)
        self.stop.set()
        if self.hb_thread:
            self.hb_thread.join(2.5)
        try:
            self.rpc('POST', '/v21/session/release', dict(session_id=self.sid))
        except Exception as e:
            confirmed = False
            self.record('release_unconfirmed', error=str(e))
            print('会话释放未确认；机载租约仍负责失联处理。', flush=True)

        self.record('cleanup_complete', confirmed=confirmed)
        return confirmed

    def run(self):
        failed = False
        try:
            c = self.rpc('GET', '/v21/capabilities')
            if c.get('backend') != 'owl_ego' or c.get('protocol_version') != 1 or not all(
                    c.get(k) is True for k in ('async_navigation', 'cancel_and_hold', 'control_lease', 'relative_xyz_yaw')):
                raise Failure('Unsupported Robot capabilities')
            h = self.health(flight=False)
            self.epoch = h['localization_epoch']
            start = self.pose()
            self.tolerances = self.rpc('GET', '/motion_tolerances')['motion_tolerances']
            if self.a.b_forward_cm < self.a.capture_after_cm+2*self.tolerances['position_tolerance_cm']+50:
                raise Failure('B is too close for a genuine moving interruption')
            preview = dict(mode='execute' if self.a.execute else 'preview', start=start,
                B_relative_body_cm=self.a.b_forward_cm, capture_after_cm=self.a.capture_after_cm,
                delay_s=self.a.delay_s, takeoff=self.a.takeoff, finish=self.a.finish,
                track=[[30,0,0,0],[0,30,0,0],[-30,0,0,0],[0,0,0,20],[20,-20,0,-20]])
            self.record('plan', **preview)
            print(json.dumps(preview, ensure_ascii=False, indent=2), flush=True)
            if not self.a.execute:
                print('只读预览完成，未获取会话、未提交运动。B 将以执行时悬停位姿计算。', flush=True)
                self.summary.write_text(json.dumps(dict(ok=True, executed=False, plan=preview), indent=2))
                return
            if not h.get('stopped') or h.get('active_task_id'):
                raise Failure('Vehicle must be stopped with no active task')
            if not h.get('airborne') and not self.a.takeoff:
                raise Failure('Ground vehicle: explicitly pass --takeoff to request takeoff')
            if not h.get('airborne') and (h.get('landed_state') != 1 or not h.get('landed_state_fresh')):
                raise Failure('Ground state is not confirmed')
            if not self.a.yes and input('确认现场净空及飞手接管条件，输入 EXECUTE 开始: ') != 'EXECUTE':
                raise Failure('Execution not confirmed')
            self.sid = self.rpc('POST', '/v21/session', dict(request_id=str(uuid.uuid4())))['session_id']
            self.hb_thread = threading.Thread(target=self.heartbeat, daemon=True)
            self.hb_thread.start()
            self.post('/init')  # Robot checks actual configured flight authorization.
            if not h['airborne']:
                self.phase = 'takeoff'
                print('等待飞手按既有流程进入 OFFBOARD 并解锁；起飞高度由 Robot 配置决定。', flush=True)
                self.blocking('/takeoff', 65, flight=False)
            self.health()
            origin = self.pose()
            B = offset(origin, x=self.a.b_forward_cm)
            self.record('waypoint', B=B, origin=origin)
            self.stage('outbound-to-B')
            tid = self.navigate(B)
            deadline = time.monotonic()+self.a.nav_timeout_s
            while True:
                self.health()
                task = self.task(tid)
                current = self.pose()
                if task['status'] in ('failed','arrived','cancelled') or time.monotonic() > deadline:
                    raise Failure('Outbound ended before moving capture')
                if task['status']=='executing' and distance(current, origin)>=self.a.capture_after_cm:
                    P = current
                    break
                time.sleep(.1)
            self.record('capture_P', P=P, epoch=self.epoch, source='get_pose (not camera exposure)')
            deadline = time.monotonic()+self.a.delay_s
            while time.monotonic() < deadline:
                self.health()
                if self.task(tid)['status'] != 'executing':
                    raise Failure('Outbound ended during simulated detection delay')
                time.sleep(.1)
            if distance(self.pose(), P)<5:
                raise Failure('Insufficient movement after P to validate interruption')
            self.phase = 'cancel-and-confirm-stop'  # Never pause for input while still moving.
            self.post('/v21/navigation/cancel', task_id=tid)
            self.wait_task(tid, ('cancelled',))
            self.record('cancelled', P=P, stop_pose=self.pose(), task=self.task(tid))
            self.stage('return-to-P')
            self.arrive(P)
            for n, values in enumerate(preview['track'], 1):
                self.stage('TRACK-%d'%n)
                before = self.pose()
                goal = offset(before, *values)
                value = self.blocking('/move_relative_xyz_yaw', 18,
                    **dict(zip(('x','y','z','yaw'), values)), timeout_s=15)
                task = self.wait_task(value['task_id'])
                actual = self.pose()
                if (distance(actual, goal)>self.tolerances['position_tolerance_cm'] or
                        abs((actual['yaw']-goal['yaw']+180)%360-180)>self.tolerances['yaw_tolerance_deg']):
                    raise Failure('TRACK arrival error exceeds tolerances')
                self.record('track_complete', relative=values, start=before, goal=goal, actual=actual, task=task)
            self.stage('return-to-P-after-TRACK')
            self.arrive(P)
            self.stage('resume-B-with-new-task')
            resumed = self.arrive(B)
            if resumed == tid:
                raise Failure('Robot reused old navigation task')
            if self.a.finish == 'land':
                self.stage('land-at-B')
                self.blocking('/land', 95, flight=False)
                h = self.health(flight=False)
                if h.get('airborne') or h.get('landed_state')!=1 or not h.get('landed_state_fresh'):
                    raise Failure('Landing not confirmed')
            self.summary.write_text(json.dumps(dict(ok=True, executed=True, P=P, B=B,
                original_task=tid, resumed_task=resumed, finish=self.a.finish), indent=2))
            print('流程通过。'+('已落地。' if self.a.finish=='land' else '停在 B；释放会话后由机载 bridge 保持，请飞手接管并降落。'), flush=True)
        except BaseException as e:
            failed = True
            self.record('failure', error=str(e), type=type(e).__name__)
            self.summary.write_text(json.dumps(dict(ok=False, error=str(e), phase=self.phase), indent=2))
            raise
        finally:
            cleaned = self.cleanup()
            if self.summary.exists():
                summary = json.loads(self.summary.read_text())
                summary['cleanup_confirmed'] = cleaned
                if not cleaned:
                    summary['ok'] = False
                self.summary.write_text(json.dumps(summary, indent=2))
            if not cleaned and not failed:
                raise Failure('Flight sequence finished but cleanup was not confirmed')
            # A timed-out daemon HTTP request may still finish and record its
            # outcome; keep the log open until process exit.


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', default='http://127.0.0.1:8765')
    p.add_argument('--execute', action='store_true', help='Authorize actual Robot HTTP flight mutations')
    p.add_argument('--takeoff', action='store_true', help='Request configured takeoff; pilot arms/selects OFFBOARD')
    p.add_argument('--finish', choices=('hold','land'), default='hold', help='Hold at B (default), or land at B')
    p.add_argument('--step', action='store_true', help='Pause only at stable stages; heartbeat remains active')
    p.add_argument('--yes', action='store_true', help='Skip the initial interactive EXECUTE confirmation')
    p.add_argument('--b-forward-cm', type=float, default=200)
    p.add_argument('--capture-after-cm', type=float, default=100,
                   help='Record P after this displacement from initial hover (default 100 cm)')
    p.add_argument('--delay-s', type=float, default=.8)
    p.add_argument('--nav-timeout-s', type=float, default=45)
    p.add_argument('--output', default='logs/owl_live/'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    a = p.parse_args(argv)
    u = urllib.parse.urlsplit(a.url)
    if u.scheme not in ('http','https') or not u.hostname or u.path not in ('','/') or u.query or u.fragment or u.username:
        p.error('--url must be an HTTP(S) server origin')
    for key, lower, upper in [('b_forward_cm',100,500),('capture_after_cm',5,100),('delay_s',.2,2),('nav_timeout_s',10,120)]:
        v = getattr(a,key)
        if not math.isfinite(v) or not lower <= v <= upper:
            p.error('%s must be within [%s,%s]'%(key,lower,upper))
    return a


if __name__ == '__main__':
    try:
        Runner(parse_args()).run()
    except (Exception, KeyboardInterrupt) as exc:
        print('测试结束：'+str(exc), flush=True)
        raise SystemExit(1)
