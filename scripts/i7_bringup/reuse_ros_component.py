#!/usr/bin/env python3
"""Monitor existing ROS nodes, or replace this process with their launcher."""

import argparse
import os
import socket
import sys
import time
from xmlrpc.client import ServerProxy

import rosgraph


def check_node(uri):
    code, message, _ = ServerProxy(uri).getPid('/i7_bringup_monitor')
    if code != 1:
        raise RuntimeError(message)


def run(nodes, command):
    socket.setdefaulttimeout(1.0)
    master = rosgraph.Master('/i7_bringup_monitor')
    # Query registrations before launching: an unresponsive registered node
    # must not be mistaken for an absent driver and started a second time.
    state = master.getSystemState()
    registered = {name for group in state for _, names in group for name in names}
    present = set(nodes) & registered
    if not present:
        os.execvp(command[0], command)
    if present != set(nodes):
        raise RuntimeError(
            f'Component is only partially running: present={sorted(present)}, '
            f'missing={sorted(set(nodes) - present)}; refusing duplicate startup'
        )

    uris = {name: master.lookupNode(name) for name in nodes}
    for uri in uris.values():
        check_node(uri)
    print(
        f'Reusing existing ROS component: {", ".join(nodes)}. '
        'Keeping its current configuration; it will not be stopped on exit.',
        flush=True,
    )
    while True:
        time.sleep(1.0)
        for name, uri in uris.items():
            if master.lookupNode(name) != uri:
                raise RuntimeError(f'Reused ROS node was replaced: {name}')
            check_node(uri)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nodes', required=True, help='Comma-separated ROS node names')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('a launch command is required after --')
    try:
        run(args.nodes.split(','), command)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f'ROS component unavailable: {exc}', file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
