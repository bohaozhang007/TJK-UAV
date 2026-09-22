"""Restore the calibrated camera baseline before stack preflight."""
import json
import math
import os
import sys

from app.robot.config_loader import load_robot_config
from app.robot.hardware.k40t import K40TClient


def reset_camera(config, client=None):
    camera = config['camera']
    tolerance = math.degrees(config['control']['yaw_tolerance_rad'])
    if client is None:
        client = K40TClient(camera['host'], camera['port'], camera['timeout_s'],
                            angle_tolerance_deg=tolerance)
    target = dict(yaw_deg=camera['baseline_yaw_deg'], pitch_deg=camera['baseline_pitch_deg'],
                  zoom=camera['baseline_zoom'])
    retries, actions = [], []
    def read(method):
        try:
            return method()
        except RuntimeError as exc:
            # Retry only an unconfirmed read, never a movement or a device fault.
            if 'confirmation timed out' not in str(exc) or 'target=None' not in str(exc):
                raise
            retries.append(str(exc))
            print('Camera state read timed out; retrying read once.', flush=True)
            return method()
    zoom = read(client.get_zoom)
    if not math.isfinite(zoom['zoom']):
        raise RuntimeError('invalid camera zoom feedback')
    if abs(zoom['zoom']-target['zoom']) > .05:
        actions.append(dict(command='zoom', value=target['zoom']))
        client.set_zoom(target['zoom'])
    for axis, method in (('yaw_deg', client.gimbal_yaw), ('pitch_deg', client.gimbal_pitch)):
        pose = read(client.get_gimbal)
        if not math.isfinite(pose[axis]):
            raise RuntimeError('invalid camera angle feedback: '+axis)
        delta = round(target[axis]-pose[axis], 2)
        if abs(target[axis]-pose[axis]) > tolerance:
            actions.append(dict(command='gimbal_'+axis[:-4], value=delta))
            method(delta)
    pose = read(client.get_gimbal)
    if any(not math.isfinite(pose[axis]) or abs(pose[axis]-target[axis]) > tolerance
           for axis in ('yaw_deg', 'pitch_deg')):
        raise RuntimeError('camera baseline angle verification failed: '+str(pose))
    zoom = read(client.get_zoom)
    if (zoom.get('zooming') or not math.isfinite(zoom['zoom'])
            or abs(zoom['zoom']-target['zoom']) > .05):
        raise RuntimeError('camera zoom is not settled at baseline: '+str(zoom))
    return dict(ok=True, target=target, final=dict(yaw_deg=pose['yaw_deg'],
                pitch_deg=pose['pitch_deg'], zoom=zoom['zoom']), actions=actions,
                read_retries=retries, angle_tolerance_deg=tolerance, zoom_tolerance=.05)


def main():
    try:
        config = load_robot_config('i7', os.environ.get('I7_V22_CONFIG'))
        print('Camera baseline restored: '+json.dumps(reset_camera(config)), flush=True)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print('I7 camera startup reset failed: '+str(exc), file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
