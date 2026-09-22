"""Restore the calibrated camera baseline before stack preflight."""
import json
import math
import os
import sys

from app.robot.config_loader import load_robot_config
from app.robot.controllers.i7_framing import restore_camera
from app.robot.hardware.k40t import K40TClient


def reset_camera(config, client=None):
    camera = config['camera']
    tolerance = math.degrees(config['control']['yaw_tolerance_rad'])
    if client is None:
        client = K40TClient(camera['host'], camera['port'], camera['timeout_s'],
                            angle_tolerance_deg=tolerance)
    target = dict(yaw_deg=camera['baseline_yaw_deg'], pitch_deg=camera['baseline_pitch_deg'],
                  zoom=camera['baseline_zoom'])
    result = restore_camera(client, target, angle_tolerance_deg=tolerance, set_zoom=client.set_zoom)
    if not result['ok']:
        raise RuntimeError('camera baseline restoration failed: '+json.dumps(result))
    zoom = client.get_zoom()
    if (zoom.get('zooming') or not math.isfinite(zoom['zoom'])
            or abs(zoom['zoom']-target['zoom']) > .05):
        raise RuntimeError('camera zoom is not settled at baseline: '+str(zoom))
    return result


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
