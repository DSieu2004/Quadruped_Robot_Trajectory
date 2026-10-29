#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.bash"
go2_acquire_launch_lock
mkdir -p "$GO2_WS/logs"
cd "$GO2_WS"
GO2_OUTPUT_SET=false
GO2_SHAPE=circle
GO2_LAW=smc
GO2_LAUNCH_ARGS=()
for GO2_ARG in "$@"; do
  if [[ "$GO2_ARG" == output_csv:=* ]]; then GO2_OUTPUT_SET=true; fi
  if [[ "$GO2_ARG" == shape:=* ]]; then GO2_SHAPE="${GO2_ARG#shape:=}"; fi
  if [[ "$GO2_ARG" == control_law:=* ]]; then
    GO2_LAW="${GO2_ARG#control_law:=}"
    GO2_LAUNCH_ARGS+=("trajectory_law:=$GO2_LAW")
  elif [[ "$GO2_ARG" == trajectory_law:=* ]]; then
    GO2_LAW="${GO2_ARG#trajectory_law:=}"
    GO2_LAUNCH_ARGS+=("$GO2_ARG")
  elif [[ "$GO2_ARG" == robot_control_law:=* ]]; then
    GO2_LAUNCH_ARGS+=("$GO2_ARG")
  else
    GO2_LAUNCH_ARGS+=("$GO2_ARG")
  fi
done
if [[ "$GO2_OUTPUT_SET" == true ]]; then
  exec ros2 launch go2_smc_ros trajectory.launch.xml "${GO2_LAUNCH_ARGS[@]}"
fi
GO2_RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
exec ros2 launch go2_smc_ros trajectory.launch.xml "output_csv:=$GO2_WS/logs/trajectory_${GO2_SHAPE}_${GO2_LAW}_${GO2_RUN_STAMP}.csv" "${GO2_LAUNCH_ARGS[@]}"
