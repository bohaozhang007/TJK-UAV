"""OWL-only EGO map and preview adapter. Preview trajectories never reach FlightCore."""
import json
import threading

import numpy as np
from app.timing import measure


def validate_preview_path(points, grid):
    resolution = grid['resolution_m']
    indices = np.floor(points / resolution).astype(int)
    blocked = {tuple(v) for v in grid['inflated']}
    checks = {
        'preview_outside_map': np.any((indices < grid['lower']) | (indices > grid['upper']), axis=1),
        'preview_below_ground': points[:, 2] <= grid['ground_m'],
        'preview_above_ceiling': points[:, 2] >= grid['ceiling_m'],
        'preview_inflated_collision': np.array([tuple(v) in blocked for v in indices]),
    }
    failures = [(int(np.flatnonzero(mask)[0]), reason) for reason, mask in checks.items() if mask.any()]
    if not failures:
        return
    index, reason = min(failures)
    detail = dict(sample_index=index, sample_count=len(points), point_world_m=points[index].tolist(),
                  voxel_index=indices[index].tolist(), start_world_m=points[0].tolist(),
                  end_world_m=points[-1].tolist(), resolution_m=resolution,
                  map_lower=grid['lower'], map_upper=grid['upper'], ground_m=grid['ground_m'],
                  ceiling_m=grid['ceiling_m'], map_version=grid['version'], map_stamp_s=grid['stamp_s'],
                  violation_counts={key:int(mask.sum()) for key,mask in checks.items() if mask.any()})
    raise RuntimeError(reason + ': ' + json.dumps(detail, allow_nan=False))


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

    def preview(self, session, epoch, goal, require_arrival, timings=None):
        import rospy
        from owl_nav_v22.srv import Command
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('preview already running')
        try:
            snap = self.guard(session, epoch)
            with measure(timings, 'map_before'):
                grid = self.map(session, epoch)
            with measure(timings, 'goal_check'):
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
                    points = np.array([start, xyz])
                    validate_preview_path(points, grid)
                    return points.tolist()
            result = {}
            event = threading.Event()
            timeout = self.c['queries']['preview_timeout_s']
            payload = dict(session_id=session, localization_epoch=epoch, goal=goal,
                           deadline=rospy.Time.now().to_sec()+timeout)
            def call():
                try:
                    service = '/owl_ego_v22/preview'
                    rospy.wait_for_service(service, timeout=.5)
                    response = rospy.ServiceProxy(service, Command)(json.dumps(payload, allow_nan=False))
                    result['value'] = json.loads(response.json)
                except Exception as exc:
                    result['error'] = exc
                finally:
                    event.set()
            with measure(timings, 'bridge_preview_round_trip'):
                threading.Thread(target=call, daemon=True).start()
                if not event.wait(timeout+.5):
                    raise RuntimeError('EGO preview timed out')
                if 'error' in result:
                    raise result['error']
                data = result['value']
                if timings is not None:
                    timings['bridge'] = data.get('timings', {})
                if not data.get('ok'):
                    raise RuntimeError(data.get('error', 'EGO preview failed'))
            with measure(timings, 'trajectory_check'):
                after = self.guard(session, epoch)
                if (data['localization_epoch'] != epoch
                        or data['planner_generation'] != snap.get('planner_generation')
                        or data['planner_generation'] != after.get('planner_generation')):
                    raise RuntimeError('preview map process or localization epoch changed')
                points = np.asarray(data['points'], float)
                velocity = np.asarray(data['end_velocity'], float)
                if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 2
                        or not np.isfinite(points).all() or velocity.shape != (3,) or not np.isfinite(velocity).all()):
                    raise RuntimeError('invalid EGO preview samples')
                if np.linalg.norm(points[0]-start) > .15:
                    raise RuntimeError('preview start does not match current hold')
                if require_arrival:
                    if np.linalg.norm(points[-1]-xyz) > .05 or np.linalg.norm(velocity) > .02:
                        raise RuntimeError('EGO did not provide a complete path with zero terminal velocity')
            # Recheck the same persistent map after optimization and fresh cloud updates.
            with measure(timings, 'map_after'):
                grid = self.map(session, epoch)
            if self.guard(session, epoch).get('planner_generation') != data['planner_generation']:
                raise RuntimeError('preview map process changed during validation')
            with measure(timings, 'path_collision_check'):
                validate_preview_path(points, grid)
            return points.tolist()
        finally:
            self.lock.release()
