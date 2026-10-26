# Source this file from Bash: source scripts/ros_env.bash
GO2_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
# VS Code Snap exports these; SLAM Toolbox otherwise rewrites absolute save paths.
unset SNAP SNAP_NAME SNAP_INSTANCE_NAME SNAP_COMMON SNAP_USER_COMMON SNAP_USER_DATA SNAP_DATA
GO2_DEPS="$GO2_WS/third_party/ros_deps/root"
if [[ -d "$GO2_DEPS/opt/ros/humble" ]]; then
  export AMENT_PREFIX_PATH="$GO2_DEPS/opt/ros/humble:$AMENT_PREFIX_PATH"
  export CMAKE_PREFIX_PATH="$GO2_DEPS/opt/ros/humble:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$GO2_DEPS/opt/ros/humble/lib:$GO2_DEPS/usr/lib/x86_64-linux-gnu:$GO2_DEPS/usr/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$GO2_DEPS/opt/ros/humble/local/lib/python3.10/dist-packages:$GO2_DEPS/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}"
  export PATH="$GO2_DEPS/opt/ros/humble/bin:$PATH"
fi
if [[ -f "$GO2_WS/ros2_install/local_setup.bash" ]]; then
  source "$GO2_WS/ros2_install/local_setup.bash"
fi
export LD_LIBRARY_PATH="$GO2_WS/third_party/mujoco-3.3.5/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
# Apply to MuJoCo and RViz processes launched in this environment.
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Hold one workspace/domain launch across exec; prevents duplicate /clock and TF.
go2_acquire_launch_lock() {
  mkdir -p "$GO2_WS/ros2_logs"
  exec 9>"$GO2_WS/ros2_logs/domain_${ROS_DOMAIN_ID}.lock"
  if ! flock -n 9; then
    echo "Go2 is already running in ROS_DOMAIN_ID=$ROS_DOMAIN_ID. Stop the current launch before starting another mode." >&2
    return 2
  fi
}
