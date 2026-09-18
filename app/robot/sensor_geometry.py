"""Robot-independent static sensor geometry. Transforms use metres, ROS axes.

T_target_from_source maps source coordinates into target coordinates. Unknown
geometry is an error, never an implicit identity transform.
"""
import copy
import math
import numpy as np


def transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=1e-5)):
        raise ValueError('sensor geometry requires finite rigid 4x4 transforms')
    return t


def sensor_geometry(config):
    g = copy.deepcopy(config.get('sensor_geometry'))
    if not isinstance(g, dict):
        raise ValueError('camera/lidar/IMU sensor_geometry is not configured')
    if g.get('units') != 'm' or not g.get('profile_id') or not g.get('quality'):
        raise ValueError('sensor geometry requires profile_id, quality and units=m')
    for key in ('camera_optical', 'lidar', 'imu', 'body'):
        if not g.get('frames', {}).get(key):
            raise ValueError('sensor geometry missing frame: ' + key)
    il = transform(g['imu_from_lidar'])
    cl = transform(g['camera_optical_from_lidar'])
    bi = transform(g['body_from_imu'])
    g['imu_from_camera_optical'] = (il @ np.linalg.inv(cl)).tolist()
    g['body_from_camera_optical'] = (bi @ il @ np.linalg.inv(cl)).tolist()
    g['direction'] = 'P_target = T_target_from_source @ P_source (homogeneous)'
    return g


class CameraPreflight:
    """Passive OWL feedback gate; timestamps use both ROS and monotonic clocks.

    Only orientation is consumed from gimbal IMU. It is a pitch feedback marker,
    not a measured full camera attitude or a replacement for the fixed extrinsic.
    Caller serializes callbacks/status with its existing bridge lock.
    """
    def __init__(self, config):
        self.config = config
        self.geometry = sensor_geometry(config)
        self.policy = config.get('camera_preflight', {})
        self.image = self.gimbal = None
        self.pitch = None
        self.image_error = 'RGB unavailable'
        self.gimbal_error = 'gimbal feedback unavailable'
        self.max_age = float(self.policy.get('max_age_s', .5))
        self.bounds = self.policy.get('pitch_range_deg', [18., 21.])
        if (not math.isfinite(self.max_age) or self.max_age <= 0
                or len(self.bounds) != 2 or not np.isfinite(self.bounds).all()
                or not -90 <= self.bounds[0] < self.bounds[1] <= 90):
            raise ValueError('invalid camera preflight age/pitch range')
        self.owl = self.policy.get('profile') == 'owl_fixed_gimbal'
        if self.policy.get('profile') not in (None, 'owl_fixed_gimbal'):
            raise ValueError('unknown camera preflight profile')

    def on_image(self, msg, now):
        self.image = (now, msg.header.stamp.to_sec())
        expected = self.config['hardware'].get('calibration', {}).get('image_size')
        self.image_error = None
        if not msg.width or not msg.height or not msg.data:
            self.image_error = 'empty RGB image'
        elif expected and list(expected) != [msg.width, msg.height]:
            self.image_error = 'RGB dimensions differ from calibration'
        elif msg.header.frame_id != self.geometry['frames']['camera_optical']:
            self.image_error = 'RGB frame differs from sensor geometry'

    def on_gimbal(self, msg, now):
        self.gimbal = (now, msg.header.stamp.to_sec())
        self.pitch = None
        self.gimbal_error = 'invalid gimbal orientation'
        q = msg.orientation
        v = np.array([q.x, q.y, q.z, q.w], dtype=float)
        if (msg.orientation_covariance[0] == -1 or not np.isfinite(v).all()
                or abs(np.linalg.norm(v)-1) > .02):
            return
        x,y,z,w = v/np.linalg.norm(v)
        self.pitch = math.degrees(math.asin(float(np.clip(2*(w*y-z*x), -1, 1))))
        self.gimbal_error = None

    def status(self, now, ros_now):
        errors = []
        def check(sample, label, error):
            if error:
                errors.append(error)
            if (sample is None or sample[1] <= 0
                    or not 0 <= now-sample[0] <= self.max_age
                    or not 0 <= ros_now-sample[1] <= self.max_age):
                errors.append(label + ' missing/stale timestamp')
        check(self.image, 'RGB', self.image_error)
        if self.owl:
            check(self.gimbal, 'gimbal', self.gimbal_error)
            if self.pitch is not None and not self.bounds[0]-1e-8 <= self.pitch <= self.bounds[1]+1e-8:
                errors.append('gimbal pitch outside %s degrees' % self.bounds)
        return dict(ready=not errors, errors=errors, pitch_deg=self.pitch,
                    pitch_range_deg=self.bounds if self.owl else None,
                    profile_id=self.geometry['profile_id'])

    def require_ready(self, now, ros_now):
        status = self.status(now, ros_now)
        if not status['ready']:
            raise ValueError('camera preflight: ' + '; '.join(status['errors']))
