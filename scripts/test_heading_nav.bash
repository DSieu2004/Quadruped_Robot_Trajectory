#!/usr/bin/env bash
# Uses the saved map: in-place rotation, forward at 90 deg, a corner and final goal yaw.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.bash"
go2_acquire_launch_lock
cd "$GO2_WS"
mkdir -p logs
GO2_PREFIX=heading
if [[ "${1:-}" == "--large-room" ]]; then
  GO2_PREFIX=heading_large
elif [[ -n "${1:-}" ]]; then
  echo 'Usage: bash scripts/test_heading_nav.bash [--large-room]'
  exit 2
fi
GO2_RUN_PID=""
cleanup() {
  if [[ -n "$GO2_RUN_PID" ]]; then
    kill -TERM -- "-$GO2_RUN_PID" 2>/dev/null || true
    wait "$GO2_RUN_PID" || true
    GO2_RUN_PID=""
  fi
}
trap cleanup EXIT INT TERM
setsid ros2 launch go2_smc_ros navigation.launch.xml "map:=$GO2_WS/maps/go2_slam.yaml" \
  headless:=true rviz:=false realtime_factor:=3.0 "log_file:=$GO2_WS/logs/${GO2_PREFIX}_physics.csv" \
  > "logs/${GO2_PREFIX}_nav.log" 2>&1 &
GO2_RUN_PID=$!
goal() {
  ros2 run go2_smc_ros navigation_goal --ros-args -p use_sim_time:=true \
    "-p" "x:=$1" "-p" "y:=$2" "-p" "yaw:=$3" \
    "-p" "trace_file:=$GO2_WS/logs/${GO2_PREFIX}_$4.csv" | tee "logs/${GO2_PREFIX}_$4-result.txt"
}
if [[ "$GO2_PREFIX" == heading_large ]]; then
  goal 4.6 0.0 1.57079632679 outbound
  goal -2.5 2.5 3.14159265359 cross_room
  goal 0.0 0.0 -1.57079632679 return
else
  goal 0.0 0.0 1.57079632679 rotate
  goal 0.0 1.0 1.57079632679 forward
  goal -1.0 1.0 3.14159265359 corner
  goal 0.0 0.0 -1.57079632679 return
fi
cleanup
awk -F, 'NR>1 {n++; if($11!=0||$12!=0)failed=1; if($13>0.05)positive=1; if($13< -0.05)negative=1; if($7>1.3||$7< -1.3)turned=1} END {exit (n<100||failed||!positive||!negative||!turned)}' "logs/${GO2_PREFIX}_physics.csv"
echo "PASS ($GO2_PREFIX): Nav2 heading, corners and final goal orientation; no falls or obstacle contacts."
