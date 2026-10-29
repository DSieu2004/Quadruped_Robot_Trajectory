#!/usr/bin/env python3
"""Compare saved PID, SMC and MPC trajectory tracking runs."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


COLORS = {"smc": "#0072BD", "pid": "#D95319", "mpc": "#77AC30"}
CONTROL_OUTPUT_KEYS = (
    "controller_x_output",
    "controller_y_output",
    "controller_yaw_output",
)
CONTROL_COLORS = {"x": "#0072BD", "y": "#D95319", "yaw": "#77AC30"}


def load_run(path: Path):
    rows = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            try:
                sample = {
                    key: float(row[key])
                    for key in (
                        "sim_time",
                        "reference_x",
                        "reference_y",
                        "actual_x",
                        "actual_y",
                    )
                }
            except (KeyError, TypeError, ValueError):
                continue
            if all(math.isfinite(value) for value in sample.values()):
                try:
                    outputs = {key: float(row[key]) for key in CONTROL_OUTPUT_KEYS}
                    outputs_available = all(math.isfinite(value) for value in outputs.values())
                except (KeyError, TypeError, ValueError):
                    outputs = {}
                    outputs_available = False
                sample.update(outputs)
                sample["controller_output_available"] = outputs_available
                rows.append(sample)
    if len(rows) < 2:
        raise ValueError(f"{path}: expected at least two valid telemetry samples")
    rows.sort(key=lambda sample: sample["sim_time"])
    return rows


def get_errors(rows, duration):
    selected = [row for row in rows if row["sim_time"] <= duration]
    if not selected:
        raise ValueError("No samples remain after applying the comparison duration")
    times = [row["sim_time"] for row in selected]
    ex = [row["reference_x"] - row["actual_x"] for row in selected]
    ey = [row["reference_y"] - row["actual_y"] for row in selected]
    norm = [math.hypot(x, y) for x, y in zip(ex, ey)]
    rms = math.sqrt(sum(value * value for value in norm) / len(norm))
    return selected, times, ex, ey, norm, rms, max(norm), norm[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shape", choices=("circle", "lemniscate"), required=True)
    parser.add_argument("--smc", type=Path, help="SMC CSV path; otherwise picks the newest named run")
    parser.add_argument("--pid", type=Path, help="PID CSV path; otherwise picks the newest named run")
    parser.add_argument("--mpc", type=Path, help="MPC CSV path; otherwise picks the newest named run")
    parser.add_argument("--logs", type=Path, default=Path("logs"))
    parser.add_argument("--duration", type=float, help="comparison duration in simulation seconds")
    parser.add_argument("--output", type=Path, help="PNG output path")
    parser.add_argument("--control-output", type=Path, help="controller output plot path")
    args = parser.parse_args()

    smc_path = args.smc or newest_run(args.logs, args.shape, "smc")
    pid_path = args.pid or newest_run(args.logs, args.shape, "pid")
    mpc_path = args.mpc or newest_run(args.logs, args.shape, "mpc")
    smc_rows, pid_rows, mpc_rows = load_run(smc_path), load_run(pid_path), load_run(mpc_path)
    duration = args.duration
    if duration is None:
        # Trajectory runs record two seconds of post-path holding data.
        duration = min(smc_rows[-1]["sim_time"], pid_rows[-1]["sim_time"], mpc_rows[-1]["sim_time"]) - 2.0
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Comparison duration must be positive")

    results = {
        "SMC": (smc_rows, get_errors(smc_rows, duration)),
        "PID": (pid_rows, get_errors(pid_rows, duration)),
        "MPC": (mpc_rows, get_errors(mpc_rows, duration)),
    }
    plt.rcParams.update({
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "axes.grid": True,
        "grid.color": "#c7c7c7",
        "grid.linewidth": 0.65,
        "font.size": 10,
        "lines.linewidth": 1.6,
        "legend.framealpha": 0.95,
    })
    figure, axes = plt.subplots(2, 2, figsize=(13, 9))
    figure.canvas.manager.set_window_title(f"{args.shape.title()} trajectory: PID vs SMC vs MPC")
    figure.suptitle(f"{args.shape.title()} trajectory tracking: PID vs SMC vs MPC", fontsize=15, fontweight="bold")
    ax_xy, ax_norm = axes[0]
    ax_ex, ax_ey = axes[1]

    smc_path_rows = results["SMC"][1][0]
    ref_x = [row["reference_x"] for row in smc_path_rows]
    ref_y = [row["reference_y"] for row in smc_path_rows]
    # Center the path at the first reference pose, so separate runs with
    # slightly different world origins can still be compared directly.
    origin_x, origin_y = ref_x[0], ref_y[0]
    ax_xy.plot([row["reference_x"] - origin_x for row in smc_path_rows],
               [row["reference_y"] - origin_y for row in smc_path_rows],
               "--", color="#222222", label="Reference")
    for label, (rows, metrics) in results.items():
        selected = metrics[0]
        color = COLORS[label.lower()]
        ax_xy.plot([row["actual_x"] - origin_x for row in selected],
                   [row["actual_y"] - origin_y for row in selected],
                   color=color, label=f"{label} actual")
        times, ex, ey, norm = metrics[1], metrics[2], metrics[3], metrics[4]
        ax_norm.plot(times, norm, color=color, label=label)
        ax_ex.plot(times, ex, color=color, label=label)
        ax_ey.plot(times, ey, color=color, label=label)

    ax_xy.set_title("XY path")
    ax_xy.set_xlabel("x relative to start [m]")
    ax_xy.set_ylabel("y relative to start [m]")
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=3, fontsize=8)
    ax_norm.set_title("Position error magnitude")
    ax_norm.set_xlabel("simulation time [s]")
    ax_norm.set_ylabel("||e|| [m]")
    ax_norm.legend(loc="best")
    summary = "\n".join(
        f"{label}: RMS {metrics[5] * 100:.2f} cm, max {metrics[6] * 100:.2f} cm"
        for label, (_, metrics) in results.items()
    )
    ax_norm.text(
        0.02, 0.98, summary, transform=ax_norm.transAxes, ha="left", va="top",
        fontsize=9, bbox={"facecolor": "white", "edgecolor": "#777777", "alpha": 0.92, "pad": 5}
    )
    ax_ex.set_title("X tracking error")
    ax_ex.set_xlabel("simulation time [s]")
    ax_ex.set_ylabel("e_x [m]")
    ax_ex.axhline(0, color="#444444", linewidth=0.8)
    ax_ex.legend(loc="best")
    ax_ey.set_title("Y tracking error")
    ax_ey.set_xlabel("simulation time [s]")
    ax_ey.set_ylabel("e_y [m]")
    ax_ey.axhline(0, color="#444444", linewidth=0.8)
    ax_ey.legend(loc="best")
    figure.tight_layout(rect=(0, 0, 1, 0.95), h_pad=4.0)

    output = args.output or args.logs / f"trajectory_{args.shape}_pid_vs_smc_vs_mpc.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180, bbox_inches="tight")
    print(f"Saved comparison plot: {output}")
    print(f"Compared duration: {duration:.2f} simulation seconds")
    print(f"{'Controller':<12}{'RMS [m]':>12}{'Max [m]':>12}{'Final [m]':>14}")
    for label, (_, metrics) in results.items():
        print(f"{label:<12}{metrics[5]:>12.4f}{metrics[6]:>12.4f}{metrics[7]:>14.4f}")
    control_output = args.control_output or output.with_name(
        f"{output.stem}_controller_response{output.suffix}"
    )
    if all(
        all(row.get("controller_output_available", False) for row in metrics[0])
        for _, metrics in results.values()
    ):
        save_controller_response(results, duration, control_output)
        print(f"Saved controller response plot: {control_output}")
    else:
        print("Skipped controller response plot: one or more CSVs lack controller output columns")
    if "agg" not in plt.get_backend().lower():
        plt.show()


def save_controller_response(results, duration: float, output: Path) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    figure.canvas.manager.set_window_title("Controller output response: PID vs SMC vs MPC")
    figure.suptitle("Trajectory controller output response", fontsize=15, fontweight="bold")

    for axis, (label, (_, metrics)) in zip(axes, results.items()):
        selected, times = metrics[0], metrics[1]
        linear_unit = "m/s" if label == "PID" else "m/s²"
        yaw_unit = "rad/s" if label == "PID" else "rad/s²"
        ux = [row["controller_x_output"] for row in selected]
        uy = [row["controller_y_output"] for row in selected]
        uyaw = [row["controller_yaw_output"] for row in selected]

        x_line, = axis.plot(times, ux, color=CONTROL_COLORS["x"], label=f"u_x [{linear_unit}]")
        y_line, = axis.plot(times, uy, color=CONTROL_COLORS["y"], label=f"u_y [{linear_unit}]")
        axis.axhline(0.0, color="#444444", linewidth=0.75)
        axis.set_title(f"{label} controller output")
        axis.set_ylabel(f"linear u [{linear_unit}]")
        axis.grid(True)

        yaw_axis = axis.twinx()
        yaw_line, = yaw_axis.plot(
            times, uyaw, color=CONTROL_COLORS["yaw"], label=f"u_yaw [{yaw_unit}]"
        )
        yaw_axis.set_ylabel(f"yaw u [{yaw_unit}]")
        yaw_axis.grid(False)
        lines = (x_line, y_line, yaw_line)
        axis.legend(lines, [line.get_label() for line in lines], loc="upper right", ncol=3, fontsize=8)

        linear_floor = 0.02 if label == "PID" else 0.05
        yaw_floor = 0.03 if label == "PID" else 0.05
        linear_limit = max(linear_floor, 1.08 * max(max(map(abs, ux)), max(map(abs, uy))))
        yaw_limit = max(yaw_floor, 1.08 * max(map(abs, uyaw)))
        axis.set_ylim(-linear_limit, linear_limit)
        yaw_axis.set_ylim(-yaw_limit, yaw_limit)

    axes[-1].set_xlabel("simulation time [s]")
    axes[-1].set_xlim(0.0, duration)
    figure.tight_layout(rect=(0, 0, 1, 0.96), h_pad=1.5)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180, bbox_inches="tight")


def newest_run(log_dir: Path, shape: str, law: str) -> Path:
    candidates = sorted(log_dir.glob(f"trajectory_{shape}_{law}_*.csv"), key=lambda item: item.stat().st_mtime)
    if candidates:
        return candidates[-1]
    known_runs = {
        ("circle", "smc"): "trajectory_circle_smc_20261026_112708.csv",
        ("circle", "pid"): "trajectory_circle_pid_20261026_214639.csv",
        ("circle", "mpc"): "trajectory_circle_mpc_20261026_122852.csv",
        ("lemniscate", "smc"): "trajectory_lemniscate_smc_20261026_113234.csv",
        ("lemniscate", "pid"): "trajectory_lemniscate_pid_20261026_150748.csv",
        ("lemniscate", "mpc"): "trajectory_lemniscate_mpc_20261026_144159.csv",
    }
    fallback_name = known_runs.get((shape, law))
    fallback = log_dir / fallback_name if fallback_name else None
    if fallback is not None and fallback.is_file():
        return fallback
    raise FileNotFoundError(
        f"No {law.upper()} {shape} CSV found in {log_dir}. "
        "Run trajectory once with that controller, or pass --smc/--pid paths."
    )


if __name__ == "__main__":
    main()
