"""Recover only identified local startup components on a disarmed aircraft."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import sys
import time
from urllib.parse import urlsplit
from xmlrpc.client import ServerProxy


def require_ground():
    import rospy
    from mavros_msgs.msg import State, ExtendedState
    state = rospy.wait_for_message('/mavros/state', State, timeout=5)
    landed = rospy.wait_for_message('/mavros/extended_state', ExtendedState, timeout=5)
    fresh = all(0 <= (rospy.Time.now()-m.header.stamp).to_sec() <= 1. for m in (state, landed))
    if (not fresh or not state.connected or state.armed or state.mode != 'POSCTL'
            or landed.landed_state != ExtendedState.LANDED_STATE_ON_GROUND):
        raise RuntimeError('Recovery requires fresh connected, disarmed, landed POSCTL feedback')


def require_no_control(master):
    groups = master.getSystemState()
    nodes = {n for group in groups for _, names in group for n in names}
    if nodes & {'/i7_ego_v22_bridge', '/i7_ego_v22_robot'}:
        raise RuntimeError('Recovery refused while flight bridge/server is registered')
    ignored = {'/mavros/setpoint_raw/target_local', '/mavros/setpoint_raw/target_global',
               '/mavros/setpoint_raw/target_attitude', '/mavros/setpoint_trajectory/desired'}
    if any(topic.startswith('/mavros/setpoint_') and topic not in ignored and names
           for topic, names in groups[0]):
        raise RuntimeError('Recovery refused while motion publishers exist')


def process(pid):
    folder = Path('/proc')/str(pid)
    fields = (folder/'stat').read_text().rsplit(')', 1)[1].split()
    return dict(parent=int(fields[1]), birth=fields[19], state=fields[0],
                args=(folder/'cmdline').read_bytes().decode().split('\0')[:-1])


def stop_component(master, names, executable, launch_names):
    present = {n for group in master.getSystemState() for _, ns in group for n in ns}
    selected = set(names) & present
    if not selected:
        return
    pids, parents = {}, set()
    hosts = {'localhost', '127.0.0.1', '::1', socket.gethostname().lower(), socket.getfqdn().lower()}
    for name in selected:
        uri = master.lookupNode(name)
        if urlsplit(uri).hostname not in hosts:
            raise RuntimeError('Refusing to stop a nonlocal ROS node: '+name)
        code, message, pid = ServerProxy(uri).getPid('/i7_startup_recovery')
        if code != 1:
            raise RuntimeError(message)
        info = process(pid)
        if Path(info['args'][0]).name not in executable:
            raise RuntimeError('Unrecognized executable for '+name)
        pids[pid] = info['birth']
        parents.add(info['parent'])
    if len(parents) != 1:
        raise RuntimeError('Component does not have a dedicated common launcher')
    parent = parents.pop()
    info = process(parent)
    args = info['args']
    if not any(Path(a).name == 'roslaunch' for a in args) or not any(Path(a).name in launch_names for a in args):
        raise RuntimeError('Unrecognized component launcher: '+str(args))
    children = set()
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            try:
                if process(int(entry.name))['parent'] == parent:
                    children.add(int(entry.name))
            except (FileNotFoundError, ProcessLookupError):
                pass
    if children != set(pids):
        raise RuntimeError('Launcher contains unrelated processes; refusing restart')
    require_ground()
    require_no_control(master)
    if process(parent)['birth'] != info['birth']:
        raise RuntimeError('Launcher identity changed')
    os.kill(parent, signal.SIGINT)
    pids[parent] = info['birth']
    deadline = time.monotonic()+12.
    while time.monotonic() < deadline:
        alive = []
        for pid, birth in pids.items():
            try:
                current = process(pid)
                if current['birth'] == birth and current['state'] != 'Z':
                    alive.append(pid)
            except (FileNotFoundError, ProcessLookupError):
                pass
        if not alive:
            return
        time.sleep(.1)
    raise RuntimeError('Component did not stop cleanly: '+str(alive))


def reboot_px4():
    import rospy
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandLong
    require_ground()
    rospy.wait_for_service('/mavros/cmd/command', timeout=5)
    result = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)(
        broadcast=False, command=246, confirmation=0,
        param1=1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0)
    if not result.success:
        raise RuntimeError('PX4 rejected reboot: '+str(result))
    started = time.monotonic()
    while time.monotonic()-started < 30:
        try:
            state = rospy.wait_for_message('/mavros/state', State, timeout=2)
            if time.monotonic()-started > 5 and state.connected:
                require_ground()
                return
        except rospy.ROSException:
            pass
    raise RuntimeError('PX4 did not reconnect in 30 seconds')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', required=True)
    parser.add_argument('actions', nargs='*')
    args = parser.parse_args()
    if set(args.actions)-{'stop_lio', 'stop_livox', 'stop_mavros', 'reboot_px4'}:
        parser.error('Unknown recovery action')
    def record(action, status, **fields):
        row = dict(time_s=time.time(), action=action, status=status, **fields)
        print(json.dumps(row), flush=True)
        with Path(args.log).open('a') as file:
            file.write(json.dumps(row)+'\n')
    action = 'ground_check'
    try:
        import rospy
        import rosgraph
        rospy.init_node('i7_startup_recovery', anonymous=True, disable_signals=True)
        socket.setdefaulttimeout(2.)
        master = rosgraph.Master(rospy.get_name())
        require_ground()
        require_no_control(master)
        for action in args.actions:
            require_ground()
            require_no_control(master)
            record(action, 'started')
            if action == 'reboot_px4':
                reboot_px4()
            elif action == 'stop_lio':
                stop_component(master, ['/laserMapping', '/lio_to_mavros'],
                               {'run_mapping_online', 'lio_to_mavros'}, {'mapping.launch', 'mapping_mid360.launch'})
            elif action == 'stop_livox':
                stop_component(master, ['/livox_lidar_publisher2'],
                               {'livox_ros_driver2_node'}, {'msg_MID360.launch'})
            elif action == 'stop_mavros':
                stop_component(master, ['/mavros'], {'mavros_node'}, {'px4.launch'})
            record(action, 'completed')
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        record(action, 'failed', error=str(exc))
        return 1


if __name__ == '__main__':
    sys.exit(main())
