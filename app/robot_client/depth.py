"""DA3 transport and mask depth localization in the exposure camera frame."""
import io
import json
import http.client
import urllib.parse
from dataclasses import dataclass

import cv2
import numpy as np

from app.timing import measure
from .base import ControlLost, MissionError, TargetNotLocalizable

DEPTH_PORT = 8792
DEPTH_TIMEOUT_S = 30.


@dataclass
class DepthFrame:
    frame_id: str
    epoch: str
    depth_m: object
    intrinsics: object

    def locate(self, obs, mask, min_pixels, max_relative_mad):
        if (self.frame_id, self.epoch) != (obs.frame_id, obs.epoch):
            raise ControlLost('depth exposure identity mismatch')
        if mask.shape != obs.rgb.shape[:2] or mask.dtype != np.bool_:
            raise TargetNotLocalizable('depth mask does not match exposure')
        ys, xs = np.nonzero(mask)
        if len(xs) < min_pixels:
            raise TargetNotLocalizable('insufficient target mask pixels')
        pixels = np.column_stack((xs, ys)).astype(np.float64)
        if obs.distortion is not None:
            pixels = cv2.undistortPoints(pixels.reshape(-1,1,2), obs.intrinsics,
                                         obs.distortion, P=self.intrinsics).reshape(-1,2)
        depths = self.depth_m[ys,xs]
        with np.errstate(invalid='ignore'):
            valid = np.isfinite(depths) & (depths > 0) & np.isfinite(pixels).all(axis=1)
        pixels, depths = pixels[valid], depths[valid]
        if len(depths) < min_pixels:
            raise TargetNotLocalizable('insufficient valid target depth pixels')
        median = float(np.median(depths))
        mad = float(np.median(np.abs(depths-median)))
        if mad/median > max_relative_mad:
            raise TargetNotLocalizable('target depth is too dispersed')
        keep = np.abs(depths-median) <= max(3*mad, median*.05)
        if np.count_nonzero(keep) < min_pixels:
            raise TargetNotLocalizable('insufficient target depth inliers')
        rays = np.linalg.solve(self.intrinsics,
            np.column_stack((pixels[keep],np.ones(np.count_nonzero(keep)))).T)
        camera_cm = rays*depths[keep]*100
        world = obs.world_from_camera_cm[:3,:3] @ camera_cm + obs.world_from_camera_cm[:3,3:4]
        point = np.median(world,axis=1)*[1,-1,1]
        if not np.isfinite(point).all():
            raise TargetNotLocalizable('nonfinite depth target position')
        return point.tolist(), dict(source='da3', frame_id=obs.frame_id, localization_epoch=obs.epoch,
            valid_depth_pixels=len(depths), inlier_pixels=int(keep.sum()), median_depth_m=median,
            relative_depth_mad=mad/median, target_position_cm=point.tolist())


class DepthClient:
    def __init__(self, host):
        self.url = f'http://{host}:{DEPTH_PORT}/estimate'

    def locate_targets(self, obs, masks, min_pixels, max_relative_mad, timings):
        return self.estimate(obs, timings, masks, min_pixels, max_relative_mad)

    def estimate(self, obs, timings, masks=None, min_pixels=16, max_relative_mad=.3):
        with measure(timings, 'depth_prepare'):
            payload = dict(intrinsics=obs.intrinsics.tolist(),output='depth',frame_id=obs.frame_id)
            if masks is not None:
                if not 1 <= len(masks) <= 3:
                    raise MissionError('expected 1 to 3 target masks')
                encoded = []
                for mask in masks:
                    if mask.shape != obs.rgb.shape[:2] or mask.dtype != np.bool_:
                        raise MissionError('depth mask does not match exposure')
                    flat = mask.ravel(order='C')
                    changes = np.flatnonzero(flat[1:] != flat[:-1]) + 1
                    counts = np.diff(np.r_[0, changes, flat.size]).tolist()
                    if flat[0]:
                        counts.insert(0, 0)
                    encoded.append(dict(size=list(mask.shape), encoding='rle', order='C', counts=counts))
                payload.update(masks=encoded, world_from_camera_cm=obs.world_from_camera_cm.tolist(),
                    distortion=obs.distortion.tolist() if obs.distortion is not None else None,
                    min_depth_pixels=min_pixels, max_relative_depth_mad=max_relative_mad)
            cached = obs.metadata.get('detector_image_cache')
            if cached and cached['frame_id'] == obs.frame_id:
                payload['image_cache'] = cached['token']
            else:
                payload['image'] = obs.image_base64
            timings['depth_image_source'] = 'detector_cache' if 'image_cache' in payload else 'upload'
            body = json.dumps(payload, allow_nan=False).encode()
        timings['depth_request_bytes'] = len(body)
        endpoint = urllib.parse.urlsplit(self.url if masks is None else self.url.rsplit('/', 1)[0] + '/locate')
        connection = http.client.HTTPConnection(endpoint.hostname, endpoint.port, timeout=DEPTH_TIMEOUT_S)
        with measure(timings, 'depth_http'):
            try:
                with measure(timings, 'depth_connect'):
                    connection.connect()
                # Upload measures socket writes, not remote receipt completion.
                with measure(timings, 'depth_upload'):
                    connection.request('POST',endpoint.path,body,{'Content-Type':'application/json'})
                with measure(timings, 'depth_wait_response'):
                    response = connection.getresponse()
                timings['depth_response_status'] = response.status
                headers = {key:response.headers.get(key) for key in (
                    'X-Unit','X-Depth-Type','X-Output','X-Coordinate-Frame','X-Frame-Id')}
                timings['depth_response_headers'] = headers
                timings['depth_server_image_source'] = response.headers.get('X-Image-Source')
                timings['depth_cache_lookup_s'] = response.headers.get('X-Cache-Lookup-S')
                for key, header in [('depth_server_elapsed_s','X-Elapsed-S'),
                                    ('depth_server_receive_s','X-Receive-S'),
                                    ('depth_server_decode_s','X-Decode-S'),
                                    ('depth_server_inference_s','X-Inference-S'),
                                    ('depth_server_encode_s','X-Encode-S')]:
                    timings[key] = response.headers.get(header)
                limit = 40*1024*1024 if masks is None else 64*1024
                with measure(timings, 'depth_download'):
                    content = response.read(limit+1 if response.status == 200 else 4096)
                timings['depth_response_bytes'] = len(content)
                if response.status == 409 and 'image_cache' in payload:
                    error = json.loads(content)
                    if error.get('error_code') == 'image_cache_miss':
                        obs.metadata.pop('detector_image_cache',None)
                        timings['depth_cache_miss'] = True
                if response.status != 200:
                    raise MissionError(f'Depth HTTP {response.status}: '+content.decode('utf-8',errors='replace'))
                if len(content) > limit:
                    raise MissionError('depth response exceeds size limit')
                if (headers['X-Unit'] != ('m' if masks is None else 'cm') or headers['X-Depth-Type'] != 'camera-z'
                        or headers['X-Output'] != ('depth' if masks is None else 'position')
                        or headers['X-Coordinate-Frame'] != ('camera-optical-right-down-forward' if masks is None else 'public-world-x-neg-y-z')
                        or headers['X-Frame-Id'] != obs.frame_id):
                    raise MissionError('depth response geometry or frame identity mismatch; restart depth server')
            finally:
                connection.close()
        with measure(timings, 'depth_decode'):
            if masks is not None:
                data = json.loads(content)
                if (not isinstance(data, dict) or data.get('ok') is not True
                        or data.get('frame_id') != obs.frame_id or data.get('unit') != 'cm'
                        or data.get('coordinate_frame') != 'public-world-x-neg-y-z'
                        or not isinstance(data.get('results'), list) or len(data['results']) != len(masks)):
                    raise MissionError('invalid remote localization response')
                for result in data['results']:
                    if not isinstance(result, dict):
                        raise MissionError('invalid target localization result')
                    if result.get('ok') is True:
                        point = np.asarray(result.get('target_position_cm'), dtype=float)
                        if point.shape != (3,) or not np.isfinite(point).all():
                            raise MissionError('invalid remote target position')
                    elif result.get('ok') is not False or result.get('error_code') != 'target_not_localizable':
                        raise MissionError('unexpected remote localization failure')
                return LocatedFrame(obs.frame_id, obs.epoch, [m.copy() for m in masks],
                                    data['results'], min_pixels, max_relative_mad)
            depth = np.load(io.BytesIO(content),allow_pickle=False)
            if depth.dtype != np.float32 or depth.shape != obs.rgb.shape[:2]:
                raise MissionError('depth response shape or dtype mismatch')
            if not (np.isfinite(depth) & (depth > 0)).any():
                raise MissionError('depth response has no valid pixels')
        return DepthFrame(obs.frame_id,obs.epoch,depth,obs.intrinsics.copy())


@dataclass
class LocatedFrame:
    frame_id: str
    epoch: str
    masks: list
    results: list
    min_pixels: int
    max_relative_mad: float

    def locate(self, obs, mask, min_pixels, max_relative_mad):
        if (self.frame_id, self.epoch) != (obs.frame_id, obs.epoch):
            raise ControlLost('depth exposure identity mismatch')
        if (min_pixels, max_relative_mad) != (self.min_pixels, self.max_relative_mad):
            raise MissionError('remote localization thresholds differ from Robot thresholds')
        for original, result in zip(self.masks, self.results):
            if np.array_equal(mask, original):
                if not result['ok']:
                    raise TargetNotLocalizable(result['error'])
                summary = dict(result, source='da3', frame_id=self.frame_id, localization_epoch=self.epoch)
                summary.pop('ok')
                return list(result['target_position_cm']), summary
        raise MissionError('mask not included in remote localization request')


def save_depth_report(prefix, report):
    prefix.with_suffix('.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
