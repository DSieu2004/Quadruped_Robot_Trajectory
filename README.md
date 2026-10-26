# Unitree Go2 Trajectory Control

A ROS 2 Humble and MuJoCo workspace for simulating a Unitree Go2 quadruped. It includes reusable C++ implementations of Sliding Mode Control (SMC), PID, and Model Predictive Control (MPC), timestamped trajectory tracking, SLAM, and Nav2 navigation.

## Features

- MuJoCo Go2 simulation with a 360-degree ray-cast LiDAR.
- Circle, ellipse, and lemniscate trajectory generation and tracking.
- PID, SMC, and MPC trajectory controllers with CSV logging and live plots.
- SLAM Toolbox map creation and Nav2 path planning with an SMC/PID controller plugin.
- Standalone C++ control libraries and MuJoCo viewer.

## Requirements

- Ubuntu 22.04, ROS 2 Humble, CMake 3.22+, a C++17 compiler, Eigen, GLFW, OpenGL, and tinyxml2.
- MuJoCo 3.3.5 Linux SDK extracted to `third_party/mujoco-3.3.5`.
- Python 3 and Matplotlib for trajectory plotting and comparison.

On a fresh machine, install ROS dependencies and build the ROS package:

```bash
cd /path/to/quadruped_ws
bash scripts/install_ros_deps.bash
bash scripts/build_ros.bash
```

`install_ros_deps.bash` downloads Nav2 and SLAM Toolbox packages into the ignored `third_party/ros_deps` directory. ROS 2 Humble and the MuJoCo SDK must already be installed or prepared as listed above.

## Run a trajectory

Each run disables static scene obstacles, starts the robot at its current odometry pose, and writes telemetry to a timestamped CSV under `logs/`.

```bash
# SMC (default)
bash scripts/run_trajectory.bash shape:=lemniscate semi_axis_x:=1.2 semi_axis_y:=0.75 period:=120 cycles:=1

# PID trajectory follower with the stable SMC low-level Go2 controller
bash scripts/run_trajectory.bash shape:=lemniscate control_law:=pid semi_axis_x:=1.2 semi_axis_y:=0.75 period:=120 cycles:=1

# MPC trajectory comparison with SMC low-level control
bash scripts/run_trajectory.bash shape:=lemniscate control_law:=mpc robot_control_law:=smc semi_axis_x:=1.2 semi_axis_y:=0.75 period:=120 cycles:=1
```

Set `shape:=circle` with `radius:=1.5`, or `shape:=ellipse` with `semi_axis_x` and `semi_axis_y`. `period` is seconds per cycle. PID low-level control and full-body MPC are experimental and have shown falls; use SMC low-level control for trajectory comparisons.

Generate offline error plots after collecting matching CSV logs:

```bash
/usr/bin/python src/trajectory/trajectory_compare.py --shape lemniscate
```

Use `--shape circle` for circle logs. The plot and RMS/max/final error summary are saved under `logs/`.

## Run Nav2

A prebuilt map is included at `maps/go2_slam.yaml`. Start MuJoCo and RViz with:

```bash
bash scripts/run_navigation.bash
```

In RViz, select **Nav2 Goal**, click a free point on the map, drag to set the final heading, and release. Press Ctrl+C in the launch terminal before starting another simulation mode. Mapping, navigation, and trajectory launches share ROS topics and should not run simultaneously.

To create and save a new map, run `bash scripts/run_mapping.bash`, then in another terminal run `bash scripts/save_map.bash` while mapping is active. See [SLAM and Nav2 details](docs/SLAM_NAV2.md).

## Build and test the standalone C++ project

```bash
cmake --preset dev
cmake --build --preset dev
ctest --preset dev
```

For ROS package rebuilds, use `bash scripts/build_ros.bash`. The detailed design notes are in [the trajectory report](docs/TRAJECTORY_REPORT.md), [the SMC navigation guide](docs/NAVIGATION_SMC.md), and the library READMEs under `libraries/`.
