#!/usr/bin/env python3
"""Ground-only shutdown of known vendor controllers before owl_ego startup."""
import argparse
import socket
import threading
import time

CONTROLLERS = ('/captain', '/mavros_controller')
FEEDBACK = {'/mavros/setpoint_raw/target_local', '/mavros/setpoint_raw/target_global',
            '/mavros/setpoint_raw/target_attitude', '/mavros/setpoint_trajectory/desired'}


def require_ground(sample):
    if not sample.get('fresh') or not sample.get('connected'):
        raise RuntimeError('拒绝交接：飞控连接或状态不新鲜')
    if sample.get('armed') is not False or sample.get('landed_state') != 1:
        raise RuntimeError('拒绝交接：必须确认已落地且未解锁')


def inspect_graph(graph):
    pubs, subs, services = graph
    nodes = {n for group in graph for _, names in group for n in names}
    motion = {t: names for t, names in pubs if t.startswith('/mavros/setpoint_') and t not in FEEDBACK and names}
    unknown = {n for names in motion.values() for n in names} - set(CONTROLLERS)
    if unknown:
        raise RuntimeError('拒绝交接：存在其他运动发布者，请先处理 '+', '.join(sorted(unknown)))
    return nodes, motion


def handover(graph, sample, shutdown, clock=time.monotonic, sleep=time.sleep):
    # Inspect all publishers before stopping anything, including an existing
    # owl bridge. Never kill by executable name or stop the parent rc-local unit.
    inspect_graph(graph())
    require_ground(sample())
    stopped = []
    for node in CONTROLLERS:
        nodes, _ = inspect_graph(graph())
        require_ground(sample())
        if node in nodes:
            print('停止厂家控制节点 '+node, flush=True)
            shutdown(node)
            stopped.append(node)
    deadline = clock()+5.
    quiet_since = None
    while clock() < deadline:
        nodes, motion = inspect_graph(graph())
        require_ground(sample())
        if not set(CONTROLLERS).intersection(nodes) and not motion:
            if quiet_since is None:
                quiet_since = clock()
            if clock()-quiet_since >= 1.:
                print('控制权交接检查通过；MAVROS、定位、相机保持运行。', flush=True)
                return stopped
        else:
            quiet_since = None
        sleep(.1)
    raise RuntimeError('厂家控制节点未退出或再次启动；未启动 owl_ego bridge')


def main():
    import yaml
    import rospy
    import rosgraph
    import rosnode
    from mavros_msgs.msg import State, ExtendedState

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    with open(args.config) as f:
        config = yaml.safe_load(f)
    socket.setdefaulttimeout(2.)
    rospy.init_node('owl_ego_prepare', anonymous=True, disable_signals=True)
    lock = threading.Lock()
    latest = {}

    def receive(key, msg):
        with lock:
            latest[key] = (msg, time.monotonic())

    subscriptions = [rospy.Subscriber(config['topics'][key], cls,
        lambda m, k=key: receive(k, m), queue_size=1)
        for key, cls in [('state', State), ('extended_state', ExtendedState)]]
    try:
        deadline = time.monotonic()+4.
        while time.monotonic() < deadline:
            with lock:
                ready = len(latest) == 2
            if ready:
                break
            time.sleep(.05)

        def sample():
            with lock:
                messages = dict(latest)
            now, stamp = time.monotonic(), rospy.Time.now().to_sec()
            fresh = len(messages) == 2 and stamp > 0 and all(
                now-at <= 2. and msg.header.stamp.to_sec() > 0
                and 0 <= stamp-msg.header.stamp.to_sec() <= 2.
                for msg, at in messages.values())
            state = messages.get('state', (None,))[0]
            extended = messages.get('extended_state', (None,))[0]
            return dict(fresh=fresh, connected=getattr(state, 'connected', False),
                        armed=getattr(state, 'armed', None),
                        landed_state=getattr(extended, 'landed_state', None))

        def shutdown(node):
            succeeded, failed = rosnode.kill_nodes([node])
            if failed or node not in succeeded:
                raise RuntimeError('无法停止 '+node+'；未启动 owl_ego bridge')

        handover(rosgraph.Master(rospy.get_name()).getSystemState, sample, shutdown)
    finally:
        for sub in subscriptions:
            sub.unregister()


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print('准备失败：'+str(exc), flush=True)
        raise SystemExit(1)
