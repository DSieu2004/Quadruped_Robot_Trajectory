#!/usr/bin/env python3
"""Live plots for timestamped trajectory tracking with PID, SMC or MPC."""

import signal
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__("trajectory_plotter")
        self.trajectory_law = self.declare_parameter("trajectory_law", "smc").value.upper()
        self._lock = threading.Lock()
        self._samples = []
        self._smoothed = []
        self._filters = [deque(maxlen=7) for _ in range(3)]
        self._reference_xy = []
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Path, "trajectory/reference", self._on_reference, path_qos)
        self.create_subscription(
            Float64MultiArray, "trajectory/telemetry", self._on_telemetry, 20
        )

    def _on_reference(self, message):
        with self._lock:
            self._reference_xy = [
                (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
            ]
        self.get_logger().info(f"Loaded fixed XY reference with {len(message.poses)} timestamped poses")

    def _on_telemetry(self, message):
        if len(message.data) < 11:
            return
        with self._lock:
            sample = tuple(message.data[:11])
            self._samples.append(sample)
            smooth = []
            for filter_buffer, column in zip(self._filters, range(5, 8)):
                filter_buffer.append(sample[column])
                smooth.append(sum(filter_buffer) / len(filter_buffer))
            self._smoothed.append(tuple(smooth))

    def snapshot(self):
        with self._lock:
            return list(self._samples), list(self._reference_xy), list(self._smoothed)


def main():
    matlab_colors = {
        "blue": "#0072BD",
        "orange": "#D95319",
        "yellow": "#EDB120",
        "purple": "#7E2F8E",
        "green": "#77AC30",
        "cyan": "#4DBEEE",
        "red": "#A2142F",
    }
    plt.rcParams.update({
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "axes.edgecolor": "#333333",
        "axes.linewidth": 0.8,
        "axes.grid": True,
        "grid.color": "#bdbdbd",
        "grid.linestyle": "-",
        "grid.linewidth": 0.65,
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Liberation Sans", "DejaVu Sans"],
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "legend.fontsize": 8,
        "legend.frameon": True,
        "legend.framealpha": 0.95,
        "legend.edgecolor": "#b0b0b0",
        "lines.linewidth": 1.5,
        "xtick.direction": "in",
        "ytick.direction": "in",
    })
    rclpy.init()
    node = TrajectoryPlotter()
    signal.signal(signal.SIGINT, lambda _signum, _frame: plt.close("all"))

    def spin_node():
        try:
            rclpy.spin(node)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()

    figure, axes = plt.subplots(2, 2, figsize=(14, 8))
    figure.canvas.manager.set_window_title(f"Go2 {node.trajectory_law} trajectory response")
    figure.suptitle(f"Go2 {node.trajectory_law} trajectory and controller response", fontsize=15, fontweight="bold")
    ax_xy, ax_position = axes[0]
    ax_command, ax_acceleration = axes[1]

    reference_xy, = ax_xy.plot([], [], "--", color=matlab_colors["blue"], label="Reference")
    actual_xy, = ax_xy.plot([], [], "-", color=matlab_colors["orange"], label="Robot")
    ax_xy.set_title("XY trajectory")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.grid(True)
    ax_xy.legend(loc="best")

    position_lines = [
        ax_position.plot([], [], "--", color=matlab_colors["blue"], label="x reference")[0],
        ax_position.plot([], [], "-", color=matlab_colors["blue"], label="x actual")[0],
        ax_position.plot([], [], "--", color=matlab_colors["red"], label="y reference")[0],
        ax_position.plot([], [], "-", color=matlab_colors["red"], label="y actual")[0],
    ]
    ax_position.set_title("Position response")
    ax_position.set_xlabel("simulation time [s]")
    ax_position.set_ylabel("position [m]")
    ax_position.grid(True)
    ax_position.legend(loc="best", ncol=2)

    error_lines = [
        ax_command.plot([], [], color=matlab_colors["blue"], label="e_x [m]")[0],
        ax_command.plot([], [], color=matlab_colors["orange"], label="e_y [m]")[0],
        ax_command.plot([], [], color=matlab_colors["purple"], linestyle="--", label="position error norm [m]")[0],
    ]
    ax_command.axhline(0.0, color="#444444", linewidth=0.8)
    ax_command.set_title("Trajectory tracking error")
    ax_command.set_xlabel("simulation time [s]")
    ax_command.set_ylabel("position error [m]")
    ax_command.grid(True)
    ax_command.legend(loc="best")

    command_colors = [matlab_colors["blue"], matlab_colors["orange"]]
    command_raw_lines = [
        ax_acceleration.plot([], [], color=color, linewidth=0.65, alpha=0.22, label="_nolegend_")[0]
        for color in command_colors
    ]
    command_lines = [
        ax_acceleration.plot([], [], color=color, label=label)[0]
        for color, label in zip(command_colors, ("cmd vx [m/s]", "cmd vy [m/s]"))
    ]
    ax_acceleration.set_title("Commanded velocity")
    ax_acceleration.set_xlabel("simulation time [s]")
    ax_acceleration.set_ylabel("linear velocity [m/s]")
    ax_acceleration.grid(True)
    ax_yaw_command = ax_acceleration.twinx()
    yaw_command_line, = ax_yaw_command.plot(
        [], [], color=matlab_colors["green"], label="cmd yaw rate [rad/s]"
    )
    ax_yaw_command.set_ylabel("yaw rate [rad/s]")
    ax_yaw_command.grid(False)
    command_handles = command_lines + [yaw_command_line]
    ax_acceleration.legend(command_handles, [line.get_label() for line in command_handles], loc="best")

    xy_limits_set = False
    position_limits_set = False

    def update(_frame):
        nonlocal xy_limits_set, position_limits_set
        samples, reference, smoothed = node.snapshot()
        if reference:
            ref_x, ref_y = [point[0] for point in reference], [point[1] for point in reference]
            reference_xy.set_data(ref_x, ref_y)
            if not xy_limits_set:
                pad_x = max(0.15, (max(ref_x) - min(ref_x)) * 0.08)
                pad_y = max(0.15, (max(ref_y) - min(ref_y)) * 0.08)
                ax_xy.set_xlim(min(ref_x) - pad_x, max(ref_x) + pad_x)
                ax_xy.set_ylim(min(ref_y) - pad_y, max(ref_y) + pad_y)
                xy_limits_set = True
            if not position_limits_set:
                all_reference = ref_x + ref_y
                pad = max(0.3, (max(all_reference) - min(all_reference)) * 0.08)
                ax_position.set_ylim(min(all_reference) - pad, max(all_reference) + pad)
                position_limits_set = True
        if not samples:
            return
        t = [row[0] for row in samples]
        actual_x, actual_y = [row[3] for row in samples], [row[4] for row in samples]

        actual_xy.set_data(actual_x, actual_y)

        ref_x, ref_y = [row[1] for row in samples], [row[2] for row in samples]
        position_lines[0].set_data(t, ref_x)
        position_lines[1].set_data(t, actual_x)
        position_lines[2].set_data(t, ref_y)
        position_lines[3].set_data(t, actual_y)
        error_x = [reference_x - actual for reference_x, actual in zip(ref_x, actual_x)]
        error_y = [reference_y - actual for reference_y, actual in zip(ref_y, actual_y)]
        error_norm = [(ex * ex + ey * ey) ** 0.5 for ex, ey in zip(error_x, error_y)]
        error_lines[0].set_data(t, error_x)
        error_lines[1].set_data(t, error_y)
        error_lines[2].set_data(t, error_norm)
        smooth_command_data = [[row[column] for row in smoothed] for column in range(3)]
        for index, raw_line in enumerate(command_raw_lines):
            raw_line.set_data(t, [row[index + 5] for row in samples])
        for smooth_line, values in zip(command_lines, smooth_command_data[:2]):
            smooth_line.set_data(t, values)
        yaw_command_line.set_data(t, smooth_command_data[2])

        left, right = t[0], max(t[-1], t[0] + 1e-3)
        for axis in (ax_position, ax_command, ax_acceleration):
            axis.set_xlim(left, right + 0.05)
        ax_command.set_ylim(-0.20, 0.20)
        ax_acceleration.set_ylim(-0.14, 0.14)
        ax_yaw_command.set_ylim(-0.22, 0.22)
        figure.suptitle(f"Go2 {node.trajectory_law} trajectory and controller response")
        return [reference_xy, actual_xy, *position_lines, *error_lines,
                *command_raw_lines, *command_lines, yaw_command_line]

    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    animation = FuncAnimation(figure, update, interval=500, blit=False, cache_frame_data=False)
    try:
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
    finally:
        _ = animation
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
