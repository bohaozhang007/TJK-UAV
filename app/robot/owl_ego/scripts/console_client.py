#!/usr/bin/env python3
"""HTTP session and motion helpers for the OWL v22 operator console."""
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






class Runner:
    def __init__(self, args):
        self.a = args
        self.sid = None
        self.epoch = None
        self.phase = 'preflight'
        self.stop = threading.Event()
        self.hb_error = None
        self.hb_thread = None
        self.delegated = False
        self.log_lock = threading.Lock()
        folder = Path(args.output)
        folder.mkdir(parents=True, exist_ok=True)
        self.log = (folder/'events.jsonl').open('x', buffering=1)

    def record(self, event, **data):
        with self.log_lock:
            self.log.write(json.dumps(dict(time_s=time.time(), monotonic_s=time.monotonic(),
                                         phase=self.phase, event=event, **data), allow_nan=False)+'\n')

    def rpc(self, method, path, data=None, timeout=2):
        if path == '/init':
            timeout = max(timeout, 15.)
        begin = time.monotonic()
        if method=='POST' and path!='/v21/heartbeat':
            self.record('http_request',method=method,path=path,request=data)
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

    def health(self, flight=True, require_localization=True, allow_manual=False):
        h = self.rpc('GET', '/health')['health']
        if self.hb_error:
            raise Failure('Heartbeat failed: '+self.hb_error)
        if require_localization and self.epoch and h.get('localization_epoch') != self.epoch:
            raise Failure('Localization epoch changed')
        if ((require_localization and not h.get('odom_ok')) or (h.get('manual_takeover') and not allow_manual)
                or h.get('conflicting_publishers')):
            raise Failure('Localization/control unavailable: '+str(h))
        if flight:
            if h.get('error'):
                raise Failure('Robot motion failed: '+str(h['error']))
            if not all(h.get(k) for k in ('initialized', 'airborne', 'control_ready', 'hold_ready')):
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
                result = self.rpc('POST', '/v21/heartbeat', dict(session_id=self.sid))
                if result.get('delegated'):
                    self.delegated = True
                    return
            except Exception as e:
                self.hb_error = str(e)
                return
            due += .5
            if self.stop.wait(max(0, due-time.monotonic())):
                return


    def task(self, tid):
        return self.rpc('GET', '/v21/navigation/status?'+urllib.parse.urlencode({'task_id': tid}))

    def wait_task(self, tid, allowed=('arrived',), cleanup=False):
        started=time.monotonic();budget=8 if cleanup or 'cancelled' in allowed else self.a.nav_timeout_s
        deadline = started+budget
        next_report=started
        while time.monotonic() < deadline:
            h={}
            if not cleanup:
                h=self.health()
            task = self.task(tid)
            if task['status'] in ('failed', 'arrived', 'cancelled'):
                if task['status'] in allowed and task.get('stopped') is True:
                    return task
                raise Failure('Task did not complete successfully: '+str(task))
            if task['status']=='stopping' and time.monotonic()>=next_report:
                self.report_stop_wait(h,task,time.monotonic()-started,budget)
                next_report=time.monotonic()+2
            time.sleep(.1)
        raise Failure('Task/stop confirmation timed out: '+tid)

    def report_stop_wait(self, health, task, elapsed, budget):
        d=(health or {}).get('stop_diagnostics') or task.get('diagnostics',{})
        def number(key):
            v=d.get(key)
            return '未知' if v is None else '%.3f'%v
        self.record('stop_wait_progress',elapsed_s=elapsed,budget_s=budget,
                    task_id=task.get('task_id'),diagnostics=d)
        if d.get('method')=='pose_window':
            print('停止确认 %.1f/%.0f s：位置范围 %s/%s m，航向范围 %s/%s°，稳定窗口 %s s（报告速度 %s m/s）。'%(
                elapsed,budget,number('position_span_m'),number('position_limit_m'),number('yaw_span_deg'),
                number('yaw_span_limit_deg'),number('stable_duration_s'),number('speed_m_s')),flush=True)
        else:
            print('停止确认 %.1f/%.0f s：速度 %s m/s，偏航 %s °/s，连续低速 %s s；尚未发送下一段运动。'%(
                elapsed,budget,number('speed_m_s'),number('yaw_rate_deg_s'),number('stable_duration_s')),flush=True)

    def wait_motion_ready(self, timeout=8.):
        deadline=time.monotonic()+timeout
        waiting=False
        next_report=0.
        while True:
            h=self.health()
            if h.get('active_task_id') or h.get('landing'):
                raise Failure('Previous task still active; refusing to queue a new motion')
            if h.get('stopped') is True:
                if waiting:self.record('motion_ready',health=h)
                return
            if not waiting:
                self.record('waiting_for_stop',health=h)
                print('等待实测停止确认（最多 %.0f 秒，心跳持续）…'%timeout,flush=True)
                waiting=True
            if time.monotonic()>=next_report:
                self.report_stop_wait(h,{},timeout-(deadline-time.monotonic()),timeout)
                next_report=time.monotonic()+2
            if time.monotonic()>=deadline:
                self.record('stop_wait_timeout',health=h)
                raise Failure('等待停止超时，未发送运动指令：'+str(h.get('stop_diagnostics',h)))
            time.sleep(.1)



    def blocking(self, path, timeout, flight=True, on_poll=None, **data):
        if path=='/move_relative_xyz_yaw':
            self.wait_motion_ready()
        # Worker owns the blocking HTTP call; foreground remains cancellable.
        result = queue.Queue()
        payload = dict(session_id=self.sid, request_id=str(uuid.uuid4()), **data)
        cancel_event = getattr(self,'abort',None)
        def call():
            try:
                if cancel_event is not None and cancel_event.is_set():
                    raise Failure('Operation cancelled before HTTP submission')
                result.put(self.rpc('POST', path, payload, timeout=timeout))
            except Exception as e:
                result.put(e)
        threading.Thread(target=call, daemon=True).start()
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if path=='/land':
                self.health(flight=False,require_localization=False)
            else:
                self.health(flight=flight)
            if on_poll is not None:
                on_poll()
            try:
                value = result.get(timeout=.1)
                if isinstance(value, Exception):
                    raise value
                if path=='/land':
                    # A successful landing may span a localization reset. A
                    # later init must acquire the current ground epoch afresh.
                    self.epoch=None
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
            if h.get('landing'):
                raise Failure('AUTO.LAND still pending; cannot confirm stop or cancel landing')
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



            # A timed-out daemon HTTP request may still finish and record its
            # outcome; keep the log open until process exit.


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', default='http://127.0.0.1:8765')
    p.add_argument('--nav-timeout-s', type=float, default=45.)
    p.add_argument('--output', default='logs/owl_v22_console/'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    a = p.parse_args(argv)
    u = urllib.parse.urlsplit(a.url)
    if u.scheme not in ('http','https') or not u.hostname or u.path not in ('','/') or u.query or u.fragment or u.username:
        p.error('--url must be an HTTP(S) server origin')
    if not math.isfinite(a.nav_timeout_s) or not 10 <= a.nav_timeout_s <= 120:
        p.error('--nav-timeout-s must be within [10,120]')
    return a
