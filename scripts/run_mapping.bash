#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/ros_env.bash"
go2_acquire_launch_lock
exec ros2 launch go2_smc_ros mapping.launch.xml explore:=true "$@"
