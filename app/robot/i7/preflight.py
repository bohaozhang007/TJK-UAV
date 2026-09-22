"""Read-only i7 deployment checks; never changes flight or calibration settings."""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import numpy as np
import yaml

from app.robot.config_loader import load_robot_config
from app.robot.i7.calibration import validate
from app.robot.sensor_geometry import sensor_geometry


def inspect(config, live=False):
    report = dict(checks={}, errors=[], flight_gates={})
    def check(name, operation):
        try:
            report['checks'][name] = operation()
        except Exception as exc:
            report['errors'].append(name + ': ' + str(exc))
    def geometry():
        validate(config)
        return sensor_geometry(config)['profile_id']
    def camera_config():
        validate(config, require_geometry=False)
        return dict(timing='stationary_receipt', exposure_delay_s=None,
                    observation_settle_s=config['camera']['observation_settle_s'])
    def planner():
        p = config['planner']
        if not all(p.get(k) for k in ('executable', 'sha256', 'parameters')):
            raise ValueError('run bash app/robot/i7/build.sh')
        binary = Path(p['executable'])
        if hashlib.sha256(binary.read_bytes()).hexdigest() != p['sha256']:
            raise ValueError('executable SHA256 mismatch')
        with Path(p['parameters']).open() as stream:
            if not isinstance(yaml.safe_load(stream), dict):
                raise ValueError('invalid planner parameters')
        return str(binary)
    check('camera_config', camera_config)
    check('sensor_geometry', geometry)
    check('planner', planner)
    for key in ('flight_enabled', 'failsafe_validated', 'sensors_validated'):
        report['flight_gates'][key] = config['control'][key]
    if live:
        live_checks(config, report, check)
    report['deployment_ready'] = not report['errors']
    report['flight_ready'] = live and report['deployment_ready'] and all(report['flight_gates'].values())
    return report


def live_checks(config, report, check):
    import rospy
    import rosgraph
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image, PointCloud2
    from mavros_msgs.msg import State
    from app.robot.hardware.k40t import K40TClient
    rospy.init_node('i7_readonly_preflight', anonymous=True, disable_signals=True)
    def source():
        m = rospy.wait_for_message(config['source']['odom'], Odometry, timeout=3.)
        if (m.header.frame_id != config['source']['world_frame']
                or m.child_frame_id != config['source']['body_frame']):
            raise ValueError('unexpected odometry frames: '+m.header.frame_id+' -> '+m.child_frame_id)
        return dict(world=m.header.frame_id, body=m.child_frame_id)
    def mapping():
        from app.robot.i7.mapping import require_running
        return require_running(config, rospy)
    def sample(key, cls):
        m = rospy.wait_for_message(config['topics'][key], cls, timeout=3.)
        age = rospy.Time.now().to_sec()-m.header.stamp.to_sec()
        max_age = (config['control']['state_timeout_s'] if cls == State
                   else config['control']['sensor_timeout_s'])
        if cls != Image and not 0 <= age <= max_age:
            raise ValueError('stale timestamp: '+str(age))
        if cls == State:
            if not m.connected:
                raise ValueError('PX4 disconnected')
            return dict(connected=m.connected, armed=m.armed, mode=m.mode)
        if cls == Image:
            if ([m.width, m.height] != config['hardware']['calibration']['image_size']
                    or m.header.frame_id != config['hardware']['camera_optical_frame'] or not m.data):
                raise ValueError('image geometry or frame mismatch')
        elif m.header.frame_id != config['control']['world_frame']:
            raise ValueError('unexpected world frame: '+m.header.frame_id)
        if cls == Odometry and m.child_frame_id != 'base_link':
            raise ValueError('unexpected normalized body frame')
        return dict(frame=m.header.frame_id, age_s=age)
    def camera():
        c = config['camera']
        angle_tolerance = math.degrees(config['control']['yaw_tolerance_rad'])
        client = K40TClient(c['host'], c['port'], c['timeout_s'],
                            angle_tolerance_deg=angle_tolerance)
        pose, zoom = client.get_gimbal(), client.get_zoom()
        if (abs(pose['yaw_deg']-c['baseline_yaw_deg']) > angle_tolerance
                or abs(pose['pitch_deg']-c['baseline_pitch_deg']) > angle_tolerance
                or abs(zoom['zoom']-c['baseline_zoom']) > .05 or zoom.get('zooming')):
            raise ValueError('camera is not at calibrated baseline: '+str(dict(pose, **zoom)))
        return dict(pose, **zoom)
    def alignment():
        from collections import deque
        from tf.transformations import euler_from_quaternion
        odoms, poses = deque(maxlen=60), deque(maxlen=60)
        handles = [rospy.Subscriber(config['topics']['odom'], Odometry, odoms.append, queue_size=10),
                   rospy.Subscriber('/mavros/local_position/pose', PoseStamped, poses.append, queue_size=10)]
        try:
            time.sleep(1.5)
            samples, references = list(odoms), list(poses)
        finally:
            for handle in handles:
                handle.unregister()
        errors = []
        for m in samples:
            if not references:
                break
            r = min(references, key=lambda r: abs((r.header.stamp-m.header.stamp).to_sec()))
            if abs((r.header.stamp-m.header.stamp).to_sec()) > .05:
                continue
            if r.header.frame_id != 'map':
                raise ValueError('unexpected MAVROS local pose frame: '+r.header.frame_id)
            a, b = m.pose.pose, r.pose
            def values(pose):
                q = pose.orientation
                xyz = np.array([pose.position.x, pose.position.y, pose.position.z])
                quat = [q.x, q.y, q.z, q.w]
                if not np.isfinite([*xyz, *quat]).all() or abs(np.linalg.norm(quat)-1) > .02:
                    raise ValueError('invalid alignment pose')
                return xyz, euler_from_quaternion(quat)[2]
            pa, ya = values(a)
            pb, yb = values(b)
            errors.append((float(np.linalg.norm(pa-pb)), abs((ya-yb+math.pi)%(2*math.pi)-math.pi)))
        if len(errors) < 5:
            raise ValueError('not enough synchronized LIO/PX4 local pose samples')
        distance, angle = np.max(errors, axis=0)
        if distance > config['control']['position_tolerance_m'] or angle > config['control']['yaw_tolerance_rad']:
            raise ValueError('LIO/PX4 local frame mismatch: %.3f m, %.2f deg' % (distance, math.degrees(angle)))
        return dict(samples=len(errors), max_distance_m=float(distance), max_yaw_deg=math.degrees(angle))
    def owners():
        pubs = rosgraph.Master(rospy.get_name()).getSystemState()[0]
        ignored = {'/mavros/setpoint_raw/target_local', '/mavros/setpoint_raw/target_global',
                   '/mavros/setpoint_raw/target_attitude', '/mavros/setpoint_trajectory/desired'}
        conflicts = {t: n for t, n in pubs if t.startswith('/mavros/setpoint_') and t not in ignored and n}
        if conflicts:
            raise ValueError('existing motion publishers: '+str(conflicts))
        return 'no active motion publishers'
    check('source_odometry', source)
    check('mapping_geometry', mapping)
    for key, cls in [('odom', Odometry), ('cloud', PointCloud2), ('rgb', Image), ('state', State)]:
        check(key, lambda key=key, cls=cls: sample(key, cls))
    check('camera_baseline', camera)
    check('lio_px4_alignment', alignment)
    check('motion_publishers', owners)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--live', action='store_true')
    args = parser.parse_args()
    report = inspect(load_robot_config('i7', args.config), args.live)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(0 if report['deployment_ready'] else 1)


if __name__ == '__main__':
    main()
