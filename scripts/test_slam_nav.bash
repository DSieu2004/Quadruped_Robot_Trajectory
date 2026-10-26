#!/usr/bin/env bash
# End-to-end test: large-room SLAM, save, reset, AMCL and three distant Nav2 goals.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.bash"
go2_acquire_launch_lock
cd "$GO2_WS"
mkdir -p logs maps
if ros2 node list 2>/dev/null | grep -qx /mujoco_bridge; then
  echo "Stop the existing Go2 launch in ROS_DOMAIN_ID=$ROS_DOMAIN_ID before this test."
  exit 2
fi
GO2_RUN_PID=""
cleanup() {
  if [[ -n "$GO2_RUN_PID" ]]; then
    # Each launch owns a process group, so no orphan simulator can keep /clock alive.
    kill -TERM -- "-$GO2_RUN_PID" 2>/dev/null || true
    wait "$GO2_RUN_PID" || true
    GO2_RUN_PID=""
  fi
}
trap cleanup EXIT INT TERM
setsid ros2 launch go2_smc_ros mapping.launch.xml headless:=true rviz:=false explore:=true realtime_factor:=4.0 \
  "log_file:=$GO2_WS/logs/slam_physics.csv" > logs/slam_e2e.log 2>&1 &
GO2_RUN_PID=$!
GO2_MAPPING_DONE=false
for GO2_TRY in {1..240}; do
  if grep -q 'Mapping loop complete' logs/slam_e2e.log; then GO2_MAPPING_DONE=true; break; fi
  if ! kill -0 "$GO2_RUN_PID" 2>/dev/null; then cat logs/slam_e2e.log; exit 3; fi
  sleep 1
done
if [[ "$GO2_MAPPING_DONE" != true ]]; then echo 'Mapping route timed out'; exit 3; fi
bash scripts/save_map.bash > logs/map_save.log 2>&1
test -s maps/go2_slam.yaml && test -s maps/go2_slam.pgm && test -s maps/go2_slam.posegraph
awk 'NR==2 {exit !($1>=190 && $2>=150)}' maps/go2_slam.pgm
cleanup
setsid ros2 launch go2_smc_ros navigation.launch.xml "map:=$GO2_WS/maps/go2_slam.yaml" headless:=true rviz:=false realtime_factor:=3.0 \
  "log_file:=$GO2_WS/logs/nav2_physics.csv" > logs/nav2_e2e.log 2>&1 &
GO2_RUN_PID=$!
ros2 run go2_smc_ros navigation_goal --ros-args -p use_sim_time:=true -p x:=4.6 -p y:=0.0 \
  "-p" "trace_file:=$GO2_WS/logs/nav2_outbound.csv" | tee logs/nav2_outbound_result.txt
ros2 run go2_smc_ros navigation_goal --ros-args -p use_sim_time:=true -p x:=-2.5 -p y:=2.5 \
  "-p" "trace_file:=$GO2_WS/logs/nav2_cross_room.csv" | tee logs/nav2_cross_room_result.txt
ros2 run go2_smc_ros navigation_goal --ros-args -p use_sim_time:=true -p x:=0.0 -p y:=0.0 \
  "-p" "trace_file:=$GO2_WS/logs/nav2_return.csv" | tee logs/nav2_return_result.txt
grep -q 'NAVIGATION SUCCEEDED' logs/nav2_outbound_result.txt
grep -q 'NAVIGATION SUCCEEDED' logs/nav2_cross_room_result.txt
grep -q 'NAVIGATION SUCCEEDED' logs/nav2_return_result.txt
cleanup
awk -F, 'NR>1 && ($11!=0 || $12!=0) {failed=1} END {exit failed}' logs/slam_physics.csv
awk -F, 'NR>1 && ($11!=0 || $12!=0) {failed=1} END {exit failed}' logs/nav2_physics.csv
awk -F, 'NR>1 {if ($3 < -0.4 || $3 > 0.4) detour=1} END {exit !detour}' logs/nav2_outbound.csv
echo 'PASS: large-room SLAM, saved occupancy/pose graph, three distant Nav2 goals, physical detour, no falls or obstacle contacts.'
