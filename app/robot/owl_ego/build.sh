#!/usr/bin/env bash
# Build v22 into separate checkout/workspace; v21 artifacts are never patched.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
WS="${OWL_EGO_V22_WS:-/home/visbot/owl_ego_v22_ws}"
UPSTREAM="${OWL_EGO_V22_UPSTREAM:-/home/visbot/owl_ego_v22_upstream}"
COMMIT=9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010
WS="$(realpath -m -- "$WS")"
UPSTREAM="$(realpath -m -- "$UPSTREAM")"
for path in "$WS" "$UPSTREAM"; do
  [[ "$path" != / && "$path" != "$ROOT" && "$path" != /home/visbot/owl_ego_ws && "$path" != /home/visbot/owl_ego_upstream ]] || { echo 'v22 needs independent directories'; exit 1; }
done
[[ "$WS" != "$UPSTREAM" ]] || { echo 'Workspace and upstream must differ'; exit 1; }
if [[ ! -d "$UPSTREAM/.git" ]]; then
  git clone https://github.com/ZJU-FAST-Lab/EGO-Planner-v2.git "$UPSTREAM"
  git -C "$UPSTREAM" checkout --detach "$COMMIT"
fi
[[ "$(git -C "$UPSTREAM" rev-parse HEAD)" == "$COMMIT" ]] || { echo 'Wrong EGO commit'; exit 1; }
if [[ ! -f "$UPSTREAM/.git/tjk-v22-map-patch" ]]; then
  [[ -z "$(git -C "$UPSTREAM" status --porcelain)" ]] || { echo 'Initial v22 checkout must be clean'; exit 1; }
fi
python3 "$ROOT/app/robot/owl_ego/patch_ego.py" "$UPSTREAM"
touch "$UPSTREAM/.git/tjk-v22-map-patch"
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
python3 "$ROOT/app/robot/owl_ego/configure.py" --workspace "$WS" --upstream "$UPSTREAM"
