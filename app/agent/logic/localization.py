"""Target localization and deduplication, independent of photography and hardware."""
import numpy as np

from app.robot_client.base import MissionError, TargetNotLocalizable
from app.robot_client.depth import DEPTH_TIMEOUT_S
from .state import State


class LocalizationMixin:
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
                if (self.c['photography']['mode'] == 'autofocus' and existing['status'] == 'failed'
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

