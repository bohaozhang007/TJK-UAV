"""i7 camera jobs over the shared v22 flight protocol."""
import base64
import io
import math
import time
from PIL import Image
import numpy as np

from .owl_ego import OwlEgoClient
from .base import MissionError


class I7Client(OwlEgoClient):
    observation_max_age_s = 1.
    observation_sync_max_s = .12

    def diagnostic_cloud(self, observation):
        key = (observation.epoch, observation.frame_id)
        if getattr(self, '_diagnostic_cloud_key', None) == key:
            return self._diagnostic_cloud_value
        try:
            result = self.rpc('POST', '/v22/diagnostic/cloud',
                dict(frame_id=observation.frame_id, localization_epoch=observation.epoch), timeout=2.)
            if result['localization_epoch'] != observation.epoch or result['frame_id'] != observation.frame_id:
                raise ValueError('diagnostic cloud exposure mismatch')
            points = np.frombuffer(base64.b64decode(result['points_base64'], validate=True),
                                   dtype='<f4').reshape(-1,3)
            if len(points) != result['metadata']['point_count'] or not np.isfinite(points).all():
                raise ValueError('invalid diagnostic point cloud')
            value = dict(points_world_cm=points.astype(float)*100, metadata=result['metadata'])
        except Exception as exc:
            value = dict(error=str(exc))
        self._diagnostic_cloud_key, self._diagnostic_cloud_value = key, value
        return value

    def observation_distortion(self, data):
        g = data.get('geometry_assumptions', {})
        d = np.asarray(data.get('distortion_coefficients', []), float)
        if (data.get('rectified') is not False or g.get('distortion') != 'raw_calibrated'
                or g.get('intrinsics') != 'configured_calibration' or g.get('extrinsics') != 'sensor_geometry'
                or not g.get('profile_id') or data.get('distortion_model') not in ('plumb_bob', 'rational_polynomial')
                or d.ndim != 1 or len(d) not in (4,5,8,12,14) or not np.isfinite(d).all()):
            raise MissionError('i7 requires raw pixels with calibrated distortion metadata; restart server and Agent')
        return d

    def rpc(self, method, path, data=None, timeout=1., timings=None):
        if path == '/v21/observation':
            timeout = max(timeout, 25.)
        if path in ('/v21/observation', '/init', '/v21/navigation'):
            timeout = max(timeout, 15.)
        result = super().rpc(method, path, data, timeout, timings)
        if path == '/v21/observation':
            span = result.get('odom_bracket_span_s')
            if type(span) not in (int, float) or not math.isfinite(span) or not 0 <= span <= .12:
                raise MissionError('i7 observation requires an odometry bracket no wider than 120 ms')
            timing = result.get('capture_timing', {})
            if timing.get('source') != 'stationary_receipt' or timing.get('stationary_checked') is not True:
                raise MissionError('i7 observation lacks stationary receipt-time verification')
            if timings is not None:
                timings['capture_timing'] = timing
        return result

    def autofocus_photo(self, observation, box, timings=None):
        self.wait_stopped()
        if observation.epoch != self.epoch:
            raise MissionError('photo exposure epoch mismatch')
        result = self.rpc('POST', '/v22/autofocus', dict(frame_id=observation.frame_id,
            box=box, localization_epoch=self.epoch, tracker_host=self.tracker_host,
            image_cache=observation.metadata.get('detector_image_cache')), timings=timings)
        task = result['task_id']
        timeout = result['timeout_s']
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 < timeout <= 720:
            raise MissionError('invalid camera task timeout')
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.health()
            state = self.rpc('GET', '/v22/autofocus/status', dict(task_id=task, session_id=self.session), timeout=3.)
            if state['status'] == 'failed':
                if timings is not None:
                    timings['camera'] = {k:v for k,v in state.items() if k not in ('session_id','image_base64')}
                raise MissionError('camera task failed: '+state['error'])
            if state['status'] == 'completed':
                self.health()
                with Image.open(io.BytesIO(base64.b64decode(state.pop('image_base64'), validate=True))) as image:
                    rgb = np.array(image.convert('RGB'))
                if timings is not None:
                    timings['camera'] = {k:v for k,v in state.items() if k != 'session_id'}
                return dict(rgb=rgb, focused=state['focused'], details=state['details'])
            time.sleep(.1)
        # An uncertain camera operation is never submitted a second time.
        raise MissionError('camera task outcome uncertain: status deadline expired')
