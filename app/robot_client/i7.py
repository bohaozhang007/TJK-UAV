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
    def rpc(self, method, path, data=None, timeout=1., timings=None):
        if path in ('/v21/observation', '/init', '/v21/navigation'):
            timeout = max(timeout, 15.)
        result = super().rpc(method, path, data, timeout, timings)
        if path == '/v21/observation':
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
            box=box, localization_epoch=self.epoch, tracker_host=self.tracker_host))
        task = result['task_id']
        timeout = result['timeout_s']
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 < timeout <= 720:
            raise MissionError('invalid camera task timeout')
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.health()
            state = self.rpc('GET', '/v22/autofocus/status', dict(task_id=task, session_id=self.session), timeout=3.)
            if state['status'] == 'failed':
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
