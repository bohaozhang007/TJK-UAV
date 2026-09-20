#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
exec python3 - <<'PY'
import os
import socket
import sys
import time

import rosgraph

socket.setdefaulttimeout(2.0)
master = rosgraph.Master('/i7_bringup_master_monitor')


def master_available():
    try:
        master.getPid()
        return True
    except (OSError, rosgraph.MasterException):
        return False


if not master_available():
    os.execvp('roscore', ['roscore'])

print(f'Reusing existing ROS master: {master.master_uri}', flush=True)
# Keep this component alive for the supervisor without owning the shared master.
try:
    while True:
        time.sleep(1.0)
        if not master_available():
            print('Existing ROS master is unavailable; stopping I7 hardware.', file=sys.stderr)
            sys.exit(1)
except KeyboardInterrupt:
    sys.exit(0)
PY
