"""OWL-only EGO map and preview adapter. Preview trajectories never reach FlightCore."""
import json
import math
import threading
import time
import uuid

import numpy as np


class OwlQueries:
    def __init__(self, hardware, config):
        self.hw, self.c = hardware, config
        self.lock = threading.Lock()

    def guard(self, session, epoch):
        snap = self.hw.snapshot()
        h = snap['health']
        if (snap.get('session_id') != session or h['localization_epoch'] != epoch
                or not all(h.get(k) for k in ('initialized', 'airborne', 'stopped', 'control_ready', 'hold_ready', 'odom_ok'))
                or h.get('active_task_id') is not None):
            raise RuntimeError('query requires the current owner, epoch and actual stopped hold')
        return snap

    def map(self, session, epoch):
        import rospy
        from std_srvs.srv import Trigger
        snap = self.guard(session, epoch)
        generation = snap.get('planner_generation')
        if not generation:
            raise RuntimeError('EGO map process is not ready')
        service = '/owl_ego_v22/planners/g_' + generation + '/ego/grid_map/query'
        outcome = {}
        event = threading.Event()
        def call():
            try:
                rospy.wait_for_service(service, timeout=.5)
                outcome['value'] = rospy.ServiceProxy(service, Trigger)()
            except Exception as exc:
                outcome['error'] = exc
            finally:
                event.set()
        threading.Thread(target=call, daemon=True).start()
        if not event.wait(1.5):
            raise RuntimeError('map service timed out')
        if 'error' in outcome:
            raise RuntimeError('map query failed: ' + str(outcome['error']))
        response = outcome['value']
        if not response.success:
            raise RuntimeError(response.message)
        result = json.loads(response.message)
        now = rospy.Time.now().to_sec()
        after = self.guard(session, epoch)
        if (after.get('planner_generation') != generation
                or not 0 <= now-result['stamp_s'] <= self.c['hardware']['rgb_max_age_s'] + .1):
            raise RuntimeError('map changed generation or became stale during query')
        return result

    def preview(self, session, epoch, goal, require_arrival):
        from owl_nav_v22.planner import PlannerProcess
        from owl_nav_v22.core import Polynomial
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('preview already running')
        process = None
        try:
            snap = self.guard(session, epoch)
            grid = self.map(session, epoch)
            resolution = grid['resolution_m']
            start = np.asarray(snap['pose'][:3], float)
            xyz = np.asarray(goal[:3], float)
            index = np.floor(xyz / resolution).astype(int)
            if require_arrival and (np.any(index < grid['lower']) or np.any(index > grid['upper'])):
                raise RuntimeError('goal outside current map bounds')
            if not grid['ground_m'] < xyz[2] < grid['ceiling_m']:
                raise RuntimeError('goal outside map height bounds')
            if require_arrival and tuple(index) in {tuple(v) for v in grid['inflated']}:
                raise RuntimeError('goal is in inflated occupancy')
            if np.linalg.norm(start-xyz) < .01:
                return [start.tolist(), xyz.tolist()]
            result = {}
            event = threading.Event()
            callback_lock = threading.Lock()
            def trajectory(generation, message):
                with callback_lock:
                    if event.is_set():
                        return
                    try:
                        result['trajectory'] = Polynomial(message)
                    except Exception as exc:
                        result['error'] = exc
                    event.set()
            # This callback only stores samples for the caller. It is NEVER Node.trajectory.
            process = PlannerProcess(self.c, 'preview_' + uuid.uuid4().hex, goal,
                                     trajectory, lambda generation: None)
            deadline = time.monotonic() + self.c['queries']['preview_timeout_s']
            while not event.is_set():
                self.guard(session, epoch)
                if time.monotonic() >= deadline:
                    raise RuntimeError('EGO preview timed out')
                process.poll()
                event.wait(.05)
            if 'error' in result:
                raise result['error']
            self.guard(session, epoch)
            trajectory = result['trajectory']
            n = min(20000, max(2, math.ceil(trajectory.duration/.02)+1))
            points = np.array([trajectory.sample(trajectory.start+t)[0]
                               for t in np.linspace(0, trajectory.duration, n)])
            if np.linalg.norm(points[0]-start) > .15:
                raise RuntimeError('preview start does not match current hold')
            if require_arrival:
                end, velocity, _ = trajectory.sample(trajectory.start+trajectory.duration)
                if np.linalg.norm(end-xyz) > .05 or np.linalg.norm(velocity) > .02:
                    raise RuntimeError('EGO did not provide a complete path with zero terminal velocity')
            # Requery the live execution map after preview startup/optimization.
            grid = self.map(session, epoch)
            resolution = grid['resolution_m']
            # Check the sampled trajectory against current map coverage and inflation.
            blocked = {tuple(v) for v in grid['inflated']}
            indices = np.floor(points / resolution).astype(int)
            if (np.any(indices < grid['lower']) or np.any(indices > grid['upper'])
                    or np.any(points[:,2] <= grid['ground_m']) or np.any(points[:,2] >= grid['ceiling_m'])
                    or any(tuple(v) in blocked for v in indices)):
                raise RuntimeError('preview leaves map coverage or crosses inflated occupancy')
            return points.tolist()
        finally:
            try:
                if process is not None:
                    process.close()
            finally:
                self.lock.release()
