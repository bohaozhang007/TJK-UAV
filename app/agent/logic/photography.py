"""Stationary, autofocus and orbit photography through the public Robot interface."""
import math

from app.robot_client.base import MissionError, NavigationPlanningFailed
from app.robot_client.artifact_writer import save_photo
from .state import State


def wrap(yaw):
    return (yaw + 180) % 360 - 180


class PhotographyMixin:
    def capture_target(self, target, observation, box):
        if self.c["photography"]["mode"] == "autofocus":
            self.photograph_target(target, observation, box)
        elif self.c["photography"]["mode"] == "photo":
            self.photograph_stationary(target)
        else:
            self.visit_target(target)

    def photograph_stationary(self, target):
        self.current_target = target
        self.transition(State.PHOTO)
        obs = self.capture_observation()
        path = self.output / f'target_{target["id"]:03d}_01.jpg'
        if not self.artifacts.submit('photo', path, save_photo, obs.rgb):
            raise MissionError('photo storage queue full')
        self.event('photo', target_id=target['id'], file=path.name, pose=obs.pose,
                   frame_id=obs.frame_id, write_status='queued')
        target['status'] = 'completed'
        self.event('target_completed', target=target, photo_count=1)
        self.current_target = None

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

