#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${OWL_EGO_WS:-/home/visbot/owl_ego_ws}"
UPSTREAM="${OWL_EGO_UPSTREAM:-/home/visbot/owl_ego_upstream}"
COMMIT=9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010
if [[ ! -d "$UPSTREAM/.git" ]]; then
  git clone https://github.com/ZJU-FAST-Lab/EGO-Planner-v2.git "$UPSTREAM"
fi
[[ "$(git -C "$UPSTREAM" rev-parse HEAD)" == "$COMMIT" ]] || { echo 'Wrong EGO commit'; exit 1; }
[[ -z "$(git -C "$UPSTREAM" status --porcelain)" ]] || { echo 'Upstream checkout must be clean'; exit 1; }
mkdir -p "$WS/src"
for name in plan_env path_searching traj_opt traj_utils plan_manage; do
  [[ -e "$WS/src/$name" ]] || ln -s "$UPSTREAM/swarm-playground/main_ws/src/planner/$name" "$WS/src/$name"
done
[[ -e "$WS/src/quadrotor_msgs" ]] || ln -s "$UPSTREAM/swarm-playground/main_ws/src/Utils/quadrotor_msgs" "$WS/src/quadrotor_msgs"
[[ -e "$WS/src/owl_nav" ]] || ln -s "$ROOT/ros/owl_nav" "$WS/src/owl_nav"
source /opt/ros/noetic/setup.bash
cd "$WS"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
python3 "$ROOT/scripts/owl_ego/configure.py" --workspace "$WS" --upstream "$UPSTREAM"
