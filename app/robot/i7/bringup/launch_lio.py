"""Launch the vendor binary with an isolated v22 fixed-geometry configuration."""
import os
from pathlib import Path
import signal
import socket
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from xmlrpc.client import ServerProxy
from urllib.parse import urlsplit

import rosgraph
import rospy
import yaml

from app.robot.config_loader import load_robot_config
from app.robot.i7.mapping import parameters, fingerprint, require_running
from app.robot.i7.bringup.reuse_ros_component import run
from app.robot.i7.bringup.launch_process import run_launcher


def registered_nodes(master):
    return {name for group in master.getSystemState() for _, names in group for name in names}


def require_no_lio_processes():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = (proc/'cmdline').read_bytes().decode(errors='replace').split('\0')
        except FileNotFoundError:
            continue
        names = {Path(arg).name for arg in args if arg and '\n' not in arg}
        command = ' '.join(args)
        if (names & {'run_mapping_online', 'lio_to_mavros'}
                or ('roslaunch' in names and ('i7_v22_lio_' in command
                    or ('faster_lio' in command and 'mapping' in command)))):
            raise RuntimeError('LIO process/launcher still exists; refusing stale registration cleanup: '
                               +proc.name+' '+command)


def clean_stale_lio(master, nodes):
    import rosnode
    local_hosts = {host.lower() for host in
                   ('localhost', '127.0.0.1', '::1', socket.gethostname(), socket.getfqdn())}
    for name in sorted(set(nodes) & registered_nodes(master)):
        uri = master.lookupNode(name)
        try:
            code, _, _ = ServerProxy(uri).getPid('/i7_lio_start_check')
        except (OSError, ConnectionError):
            code = None
        if code == 1:
            continue
        endpoint = urlsplit(uri)
        if endpoint.hostname not in local_hosts or endpoint.port is None:
            raise RuntimeError('Unreachable LIO registration is not confirmed local: '+name+' '+uri)
        # A timeout is not proof of exit; require a closed local endpoint.
        try:
            with socket.create_connection(('127.0.0.1', endpoint.port), timeout=1.):
                raise RuntimeError('LIO endpoint still accepts connections: '+name)
        except ConnectionRefusedError:
            pass
        require_no_lio_processes()
        if master.lookupNode(name) != uri:
            raise RuntimeError('LIO registration changed during cleanup: '+name)
        print('Removing stale local LIO registration: '+name+' '+uri, flush=True)
        rosnode.cleanup_master_blacklist(master, [name])


def require_ground():
    from mavros_msgs.msg import State, ExtendedState
    if not rospy.core.is_initialized():
        rospy.init_node('i7_lio_start_check', anonymous=True, disable_signals=True)
    changed = threading.Condition()
    samples = {}
    subscribers = []
    def receive(message, key):
        with changed:
            samples[key] = (message, time.monotonic())
            changed.notify_all()
    try:
        for topic, cls, key in (('/mavros/state', State, 'state'),
                                ('/mavros/extended_state', ExtendedState, 'landed')):
            subscribers.append(rospy.Subscriber(topic, cls, receive, callback_args=key, queue_size=1))
        deadline = time.monotonic()+3.
        with changed:
            while not rospy.is_shutdown():
                now, wall = rospy.Time.now().to_sec(), time.monotonic()
                details = {'missing': sorted({'state', 'landed'}-samples.keys())}
                fresh = not details['missing']
                for key, (msg, received) in samples.items():
                    age, receipt_age = now-msg.header.stamp.to_sec(), wall-received
                    details[key+'_age_s'] = round(age, 3)
                    details[key+'_receipt_age_s'] = round(receipt_age, 3)
                    fresh = fresh and msg.header.stamp.to_sec() > 0 and 0 <= age <= .5 and 0 <= receipt_age <= .5
                if 'state' in samples:
                    state = samples['state'][0]
                    details.update(connected=state.connected, armed=state.armed, mode=state.mode)
                if 'landed' in samples:
                    details['landed_state'] = samples['landed'][0].landed_state
                if (fresh and details['connected'] and not details['armed']
                        and details['mode'] == 'POSCTL'
                        and details['landed_state'] == ExtendedState.LANDED_STATE_ON_GROUND):
                    return
                remaining = deadline-wall
                if remaining <= 0:
                    raise RuntimeError('LIO replacement requires fresh MAVROS feedback: connected, '
                                       'disarmed, on ground and POSITION mode; actual='+str(details))
                changed.wait(min(.05, remaining))
        raise RuntimeError('ROS shut down during LIO ground check')
    finally:
        for subscriber in subscribers:
            subscriber.unregister()


def legacy_launcher(master, nodes):
    # Only the known local, dedicated vendor launcher may be stopped.
    parents = set()
    pids = set()
    directory = Path('/home/jkhk/jkhk_robot/release/slam')
    for name in nodes:
        code, message, pid = ServerProxy(master.lookupNode(name)).getPid('/i7_lio_start_check')
        if code != 1:
            raise RuntimeError(message)
        proc = Path('/proc')/str(pid)
        executable = 'run_mapping_online' if name == '/laserMapping' else 'lio_to_mavros'
        if (proc/'exe').resolve() != (directory/'lib/faster_lio'/executable).resolve():
            raise RuntimeError('LIO node is not the known local vendor executable: '+name)
        fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
        parents.add(int(fields[1]))
        pids.add(pid)
    if len(parents) != 1:
        raise RuntimeError('LIO nodes do not share one dedicated launcher')
    pid = parents.pop()
    proc = Path('/proc')/str(pid)
    command = (proc/'cmdline').read_bytes().split(b'\0')
    expected = [b'/usr/bin/python3', b'/opt/ros/noetic/bin/roslaunch',
                os.fsencode(directory/'share/faster_lio/launch/mapping_mid360.launch'), b'rviz:=false', b'']
    children = set()
    for child in Path('/proc').iterdir():
        if not child.name.isdigit():
            continue
        try:
            fields = (child/'stat').read_text().rsplit(')', 1)[1].split()
        except FileNotFoundError:
            continue
        if int(fields[1]) == pid:
            children.add(int(child.name))
    if command != expected or children != pids:
        raise RuntimeError('Unknown/shared LIO launcher; stop it manually before starting v22 hardware')
    return pid


def replace_legacy(master, nodes):
    if any('i7_ego_v22' in name for name in registered_nodes(master)):
        raise RuntimeError('Stop the v22 bridge before replacing LIO and resetting localization')
    pid = legacy_launcher(master, nodes)
    require_ground()
    # Recheck identity after waiting for flight feedback.
    if legacy_launcher(master, nodes) != pid:
        raise RuntimeError('LIO launcher changed during ground check')
    print('Replacing incompatible vendor LIO on disarmed POSITION ground state. '
          'MAVROS and LiDAR stay running; localization will reset.', flush=True)
    os.kill(pid, signal.SIGINT)
    deadline = time.monotonic()+10.
    while time.monotonic() < deadline:
        if not set(nodes) & registered_nodes(master) and not Path('/proc', str(pid)).exists():
            require_ground()
            return
        time.sleep(.1)
    raise RuntimeError('Old LIO did not exit within 10 seconds; no replacement was started')


def main():
    socket.setdefaulttimeout(1.)
    config = load_robot_config('i7', os.environ.get('I7_V22_CONFIG'))
    nodes = ['/laserMapping', '/lio_to_mavros']
    master = rosgraph.Master('/i7_lio_start_check')
    clean_stale_lio(master, nodes)
    registered = registered_nodes(master)
    if set(nodes) & registered:
        if not set(nodes) <= registered:
            raise RuntimeError('LIO is only partially running: present='+str(sorted(set(nodes) & registered))
                               +'; missing='+str(sorted(set(nodes)-registered))
                               +'; stop its launcher before restarting hardware')
        try:
            require_running(config, rospy)
        except ValueError as exc:
            print('Existing LIO configuration is incompatible: '+str(exc), flush=True)
            replace_legacy(master, nodes)
        else:
            run(nodes, [])
            return
    if set(nodes) & registered_nodes(master):
        raise RuntimeError('LIO reappeared before launch; refusing duplicate startup')
    params = yaml.safe_load(Path(__file__).with_name('lio.yaml').read_text())
    params['mapping'].update(parameters(config))
    root = Path(__file__).resolve().parents[4]
    logs = root/'logs/i7_v22/trajectories'
    logs.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='i7_v22_lio_') as directory:
        folder = Path(directory)
        param_file = folder/'lio.yaml'
        param_file.write_text(yaml.safe_dump(params))
        launch = ET.Element('launch')
        ET.SubElement(launch, 'rosparam', command='load', file=str(param_file))
        for key, value in dict(localization_mode_en='false', savePCD='false').items():
            ET.SubElement(launch, 'param', name=key, value=value, type='bool')
        node = ET.SubElement(launch, 'node', pkg='faster_lio', type='run_mapping_online',
                             name='laserMapping', output='screen', required='true',
                             args='--traj_log_dir='+str(logs))
        ET.SubElement(node, 'param', name='i7_geometry_fingerprint', value=fingerprint(config))
        ET.SubElement(launch, 'node', pkg='faster_lio', type='lio_to_mavros', name='lio_to_mavros',
                      output='screen', required='true')
        launch_file = folder/'mapping.launch'
        ET.ElementTree(launch).write(str(launch_file), encoding='unicode')
        raise SystemExit(run_launcher(['roslaunch', str(launch_file)]))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
    except (RuntimeError, ValueError, rospy.ROSException) as exc:
        print('I7 LIO startup failed: '+str(exc), file=sys.stderr, flush=True)
        sys.exit(1)
