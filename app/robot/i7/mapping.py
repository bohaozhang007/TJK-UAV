"""MID-360 fixed geometry in the vendor's rotated lidar and IMU axes."""
import hashlib
import json

import numpy as np

from app.robot.sensor_geometry import transform


def parameters(config):
    g = config['sensor_geometry']
    body_imu = transform(g['body_from_imu'])
    imu_lidar = transform(g['imu_from_lidar'])
    if not np.allclose(body_imu[:3, 3], 0):
        raise ValueError('vendor virtual body must share the Livox IMU origin')
    virtual = body_imu @ imu_lidar @ np.linalg.inv(body_imu)
    return dict(extrinsic_est_en=False, extrinsic_R=virtual[:3, :3].reshape(-1).tolist(),
                extrinsic_T=virtual[:3, 3].tolist(),
                rotation2virtualbody=body_imu[:3, :3].reshape(-1).tolist())


def fingerprint(config):
    return hashlib.sha256(json.dumps(parameters(config), sort_keys=True).encode()).hexdigest()


def require_running(config, ros):
    expected = parameters(config)
    actual = ros.get_param('/mapping', {})
    for key, value in expected.items():
        if key == 'extrinsic_est_en':
            valid = actual.get(key) is False
        else:
            found = np.asarray(actual.get(key, []), float)
            valid = found.shape == np.asarray(value).shape and np.allclose(found, value, atol=1e-8)
        if not valid:
            raise ValueError('running Faster-LIO '+key+' differs from the i7 v22 fixed geometry; '
                             'stop the old LIO launcher and start app/run_i7.sh hardware')
    if ros.get_param('/laserMapping/i7_geometry_fingerprint', '') != fingerprint(config):
        raise ValueError('running Faster-LIO was not started with the i7 v22 geometry profile')
    common = ros.get_param('/common', {})
    if common.get('imu_topic') != '/livox/imu' or common.get('lid_topic') != '/livox/lidar':
        raise ValueError('Faster-LIO must use the MID-360 lidar and its internal IMU')
    return expected
