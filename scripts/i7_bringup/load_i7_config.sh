#!/usr/bin/env bash

I7_BRINGUP_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
I7_NAV_CONFIG_PATH="${I7_BRINGUP_SCRIPT_DIR}/../../ros/i7_nav/config/i7_nav.yaml"

read_i7_bringup_config() {
  python3 -c \
    'import sys, yaml; print(yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["bringup"][sys.argv[2]])' \
    "${I7_NAV_CONFIG_PATH}" "$1"
}
