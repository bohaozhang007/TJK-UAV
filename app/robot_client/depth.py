"""DA3 transport and mask depth localization in the exposure camera frame."""
import io
import json
import urllib.request
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
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def estimate(self, obs, timings):
        with measure(timings, 'depth_prepare'):
            body = json.dumps(dict(image=obs.image_base64, intrinsics=obs.intrinsics.tolist(),
                                   output='depth', frame_id=obs.frame_id), allow_nan=False).encode()
        request = urllib.request.Request(self.url,body,{'Content-Type':'application/json'},method='POST')
        with measure(timings, 'depth_http'):
            with self.http.open(request,timeout=DEPTH_TIMEOUT_S) as response:
                if (response.headers.get('X-Unit') != 'm'
                        or response.headers.get('X-Depth-Type') != 'camera-z'
                        or response.headers.get('X-Output') != 'depth'
                        or response.headers.get('X-Coordinate-Frame') != 'camera-optical-right-down-forward'
                        or response.headers.get('X-Frame-Id') != obs.frame_id):
                    raise MissionError('depth response geometry or frame identity mismatch; restart depth server')
                content = response.read(40*1024*1024+1)
                if len(content) > 40*1024*1024:
                    raise MissionError('depth response exceeds size limit')
                timings['depth_server_elapsed_s'] = response.headers.get('X-Elapsed-S')
        with measure(timings, 'depth_decode'):
            depth = np.load(io.BytesIO(content),allow_pickle=False)
            if depth.dtype != np.float32 or depth.shape != obs.rgb.shape[:2]:
                raise MissionError('depth response shape or dtype mismatch')
            if not (np.isfinite(depth) & (depth > 0)).any():
                raise MissionError('depth response has no valid pixels')
        return DepthFrame(obs.frame_id,obs.epoch,depth,obs.intrinsics.copy())


def save_depth_report(prefix, report):
    prefix.with_suffix('.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
