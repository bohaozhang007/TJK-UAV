#!/usr/bin/env bash
# Build v22 into separate checkout/workspace; v21 artifacts are never patched.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
WS="${I7_EGO_V22_WS:-$HOME/i7_ego_v22_ws}"
UPSTREAM="${I7_EGO_V22_UPSTREAM:-$HOME/i7_ego_v22_upstream}"
COMMIT=9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010
WS="$(realpath -m -- "$WS")"
UPSTREAM="$(realpath -m -- "$UPSTREAM")"
for path in "$WS" "$UPSTREAM"; do
  case "$path/" in
    /home/jkhk/jkhk_robot/release/*|/home/visbot/owl_ego*/|/home/visbot/owl_ego*/*) echo "Refusing to modify existing deployment: $path"; exit 1 ;;
  esac
  [[ "$path" != / && "$path" != "$ROOT" && "$path" != /home/visbot/owl_ego_ws && "$path" != /home/visbot/owl_ego_upstream && "$path" != /home/visbot/owl_ego_v22_ws && "$path" != /home/visbot/owl_ego_v22_upstream && "$path" != /home/jkhk/jkhk_robot/release/planner ]] || { echo 'v22 needs independent directories'; exit 1; }
done
[[ "$WS" != "$UPSTREAM" ]] || { echo 'Workspace and upstream must differ'; exit 1; }
ARCHIVE_SHA256=e5b1da27b0f74f1a68b096926ee9667fb9edf676238a7b6a20b0cba021d5221b
if [[ ! -e "$UPSTREAM" ]]; then
  ARCHIVE="$(mktemp --suffix=.tar.gz)"
  trap 'rm -f -- "$ARCHIVE"' EXIT
  if [[ -n "${I7_EGO_V22_ARCHIVE:-}" ]]; then
    cp -- "$I7_EGO_V22_ARCHIVE" "$ARCHIVE"
  else
    curl -fL --connect-timeout 15 --max-time 180 \
      "https://codeload.github.com/ZJU-FAST-Lab/EGO-Planner-v2/tar.gz/$COMMIT" -o "$ARCHIVE"
  fi
  [[ "$(sha256sum "$ARCHIVE" | cut -d ' ' -f1)" == "$ARCHIVE_SHA256" ]] || { echo 'EGO archive checksum mismatch'; exit 1; }
  mkdir -p "$UPSTREAM"
  tar -xzf "$ARCHIVE" --strip-components=1 -C "$UPSTREAM"
  echo "$COMMIT $ARCHIVE_SHA256" > "$UPSTREAM/.tjk-v22-source"
fi
[[ "$(cat "$UPSTREAM/.tjk-v22-source")" == "$COMMIT $ARCHIVE_SHA256" ]] || { echo 'Wrong independent EGO source'; exit 1; }
python3 "$ROOT/app/robot/owl_ego/patch_ego.py" "$UPSTREAM"
mkdir -p "$WS/src"
link_package() {
  local target="$1" name="$2"
  if [[ -e "$WS/src/$name" || -L "$WS/src/$name" ]]; then
    [[ "$(realpath -- "$WS/src/$name")" == "$(realpath -- "$target")" ]] || { echo "Wrong package link: $name"; exit 1; }
  else
    ln -s "$target" "$WS/src/$name"
  fi
}
for name in plan_env path_searching traj_opt traj_utils plan_manage; do
  link_package "$UPSTREAM/swarm-playground/main_ws/src/planner/$name" "$name"
done
link_package "$UPSTREAM/swarm-playground/main_ws/src/Utils/quadrotor_msgs" quadrotor_msgs
link_package "$ROOT/app/robot/owl_ego/ros" owl_nav_v22
source /opt/ros/noetic/setup.bash
cd "$WS"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
python3 "$ROOT/app/robot/owl_ego/configure.py" --workspace "$WS" --upstream "$UPSTREAM" --config "$ROOT/app/robot/config/i7.yaml"
