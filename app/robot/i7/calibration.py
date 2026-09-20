"""Validate baseline geometry and stationary camera capture settings."""
import math
from types import SimpleNamespace
from app.robot.sensor_geometry import sensor_geometry
from app.robot.controllers.owl_ego_observation import camera_intrinsics


def validate(config, *, require_geometry=True):
    try:
        if require_geometry:
            for key in ('imu_from_lidar', 'camera_optical_from_lidar', 'body_from_imu'):
                if config.get('sensor_geometry', {}).get(key) is None:
                    raise ValueError('sensor_geometry.' + key + ' is not measured')
            sensor_geometry(config)
        c = config['hardware']['calibration']
        size = c['image_size']
        if not isinstance(size, list) or len(size) != 2 or any(type(v) is not int or v <= 0 for v in size):
            raise ValueError('image_size must contain native width and height')
        camera_intrinsics(config['hardware'], SimpleNamespace(width=size[0], height=size[1], header=SimpleNamespace(frame_id=config['hardware']['camera_optical_frame'])), None)
        timeout = config['autofocus']['timeout_s']
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 1 <= timeout <= 600:
            raise ValueError('autofocus.timeout_s must be within [1, 600]')
        camera = config['camera']
        for key in ('observation_settle_s', 'settle_s'):
            value = camera.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 10:
                raise ValueError(key + ' must be within (0, 10] seconds')
        for key, low, high in (('baseline_yaw_deg', -180, 180), ('baseline_pitch_deg', -90, 30), ('baseline_zoom', 1, 160)):
            value = camera[key]
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError('invalid ' + key)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('i7 baseline calibration/timing incomplete: ' + str(exc)) from exc
