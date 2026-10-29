#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.bash"
go2_acquire_launch_lock
test -f "$GO2_WS/maps/go2_slam.yaml" || { echo 'Create and save the SLAM map first: bash scripts/save_map.bash'; exit 1; }
GO2_LAUNCH_ARGS=()
for GO2_ARG in "$@"; do
  if [[ "$GO2_ARG" == control_law:=* ]]; then
    GO2_LAUNCH_ARGS+=("path_control_law:=${GO2_ARG#control_law:=}")
  else
    GO2_LAUNCH_ARGS+=("$GO2_ARG")
  fi
done
exec ros2 launch go2_smc_ros navigation.launch.xml "map:=$GO2_WS/maps/go2_slam.yaml" "${GO2_LAUNCH_ARGS[@]}"
