#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.bash"
mkdir -p "$GO2_WS/maps"
GO2_MAP_PATH="${1:-$GO2_WS/maps/go2_slam}"
ros2 run nav2_map_server map_saver_cli -f "$GO2_MAP_PATH" --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$GO2_MAP_PATH'}"
