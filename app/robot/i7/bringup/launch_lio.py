"""Launch the vendor binary with an isolated v22 fixed-geometry configuration."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from xmlrpc.client import ServerProxy

import rosgraph
import rospy
import yaml

from app.robot.config_loader import load_robot_config
from app.robot.i7.mapping import parameters, fingerprint, require_running
from app.robot.i7.bringup.reuse_ros_component import run


def registered_nodes(master):
    return {name for group in master.getSystemState() for _, names in group for name in names}


def require_ground():
    from mavros_msgs.msg import State, ExtendedState
    if not rospy.core.is_initialized():
        rospy.init_node('i7_lio_start_check', anonymous=True, disable_signals=True)
    state = rospy.wait_for_message('/mavros/state', State, timeout=3.)
    landed = rospy.wait_for_message('/mavros/extended_state', ExtendedState, timeout=3.)
    now = rospy.Time.now().to_sec()
    if (not state.connected or state.armed or state.mode != 'POSCTL'
            or landed.landed_state != ExtendedState.LANDED_STATE_ON_GROUND
            or any(not 0 <= now-msg.header.stamp.to_sec() <= .5 for msg in (state, landed))):
        raise RuntimeError('LIO replacement requires fresh MAVROS feedback: connected, '
                           'disarmed, on ground and POSITION mode')


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
    registered = registered_nodes(master)
    if set(nodes) & registered:
        if not set(nodes) <= registered:
            raise RuntimeError('LIO is only partially running; stop its launcher before restarting hardware')
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
        raise SystemExit(subprocess.call(['roslaunch', str(launch_file)]))


if __name__ == '__main__':
    main()
