#!/usr/bin/env bash
I7_V22_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
read_i7_bringup_config() {
  PYTHONPATH="$I7_V22_ROOT${PYTHONPATH:+:$PYTHONPATH}" python3 -c \
    'import os, sys; from app.robot.config_loader import load_robot_config; print(load_robot_config("i7", os.environ.get("I7_V22_CONFIG"))["bringup"][sys.argv[1]])' "$1"
}
