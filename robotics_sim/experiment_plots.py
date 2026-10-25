"""Matplotlib figures built from a recorded experiment, independent of Tk.

The caller owns the snapshot and embeds the returned Figure in its plot window.
No live robot/sensor reads occur while a figure is being drawn.
"""

import re

import numpy as np
from matplotlib.figure import Figure


TEAL = "#087f83"
ORANGE = "#cf8826"
PURPLE = "#8558ae"
SLATE = "#61778a"
GREEN = "#32947d"


def _numeric_table(columns, rows):
    """Keep missing measurements as gaps instead of inventing zeros."""
    rows = tuple(rows)
    result = {}
    for index, name in enumerate(columns):
        values = []
        for row in rows:
            try:
                value = float(row[index])
            except (ValueError, TypeError, IndexError):
                value = np.nan
            values.append(value if np.isfinite(value) else np.nan)
        result[name] = np.asarray(values)
    return result


def _line(ax, time, values, label, color, *, style="-", step=False):
    if values is None or len(time) != len(values) or not np.any(np.isfinite(values)):
        return
    ax.plot(time, values, style, color=color, label=label, linewidth=1.8,
            marker="o" if len(time) == 1 else None, markersize=4,
            drawstyle="steps-post" if step else "default")


def _series(ax, table, keys, *, title, ylabel, time_key="time_s", step=False):
    time = table.get(time_key, np.empty(0))
    for key, label, color in keys:
        _line(ax, time, table.get(key), label, color, step=step)
    ax.set(title=title, xlabel="Time [s]", ylabel=ylabel)


def _trajectory(ax, table, *, x="x_m", y="y_m", label="Actual", color=TEAL):
    if x not in table or y not in table:
        return
    ax.plot(table[x], table[y], color=color, linewidth=2, label=label,
            marker="o" if len(table[x]) == 1 else None)
    if len(table[x]):
        ax.scatter(table[x][0], table[y][0], color=color, s=22, marker="o", zorder=4)
        ax.scatter(table[x][-1], table[y][-1], color=color, s=35, marker="x", zorder=4)


def _goals(ax, goals):
    for index, goal in enumerate(goals):
        x, y = goal[:2]
        ax.scatter([x], [y], s=75, marker="*", color=ORANGE, zorder=5,
                   label="Goal" if index == 0 else None)
        ax.annotate(str(index + 1), (x, y), xytext=(7, 6), textcoords="offset points",
                    fontsize=8, color=SLATE)
        if len(goal) >= 3:
            ax.annotate("", (x + .18 * np.cos(goal[2]), y + .18 * np.sin(goal[2])),
                        (x, y), arrowprops={"arrowstyle": "->", "color": ORANGE})


def _xy_labels(ax, title="Trajectory"):
    ax.set(title=title, xlabel="x [m]", ylabel="y [m]")
    ax.set_aspect("equal", adjustable="datalim")


def _base(axes, data, snapshot):
    _trajectory(axes[0], data)
    _goals(axes[0], snapshot.get("goals", ()))
    _xy_labels(axes[0])
    if "position_error_m" in data:
        _series(axes[1], data, [("position_error_m", "Distance to active goal", TEAL)],
                title="Position error", ylabel="Distance [m]")
        _series(axes[2], data, [("angle_error_rad", "Final-heading error", PURPLE)],
                title="Target-angle errors (wrapped)", ylabel="Angle [rad]")
        goals = snapshot.get("goals", ())
        if len(goals):
            bearing = np.full(len(data["time_s"]), np.nan)
            for i, goal_index in enumerate(data["goal_index"]):
                if not np.isfinite(goal_index) or not 0 <= int(goal_index) < len(goals):
                    continue
                gx, gy = goals[int(goal_index)][:2]
                dx, dy = gx - data["x_m"][i], gy - data["y_m"][i]
                if np.hypot(dx, dy) > 1e-9:
                    bearing[i] = (np.arctan2(dy, dx) - data["yaw_rad"][i] + np.pi) % (2 * np.pi) - np.pi
            _line(axes[2], data["time_s"], bearing, "Bearing error", TEAL)
        goal_indices = data.get("goal_index", np.empty(0))
        changes = np.flatnonzero(np.diff(goal_indices) != 0) + 1
        for index in changes:
            for ax in axes[1:3]:
                ax.axvline(data["time_s"][index], color=SLATE, linestyle=":", alpha=.55)
        if len(changes):
            axes[1].text(.02, .98, "Dotted lines: target changes", transform=axes[1].transAxes,
                         fontsize=8, va="top", color=SLATE)
    else:
        _series(axes[1], data, [("x_m", "x", TEAL), ("y_m", "y", ORANGE),
                               ("z_m", "z", PURPLE)], title="Position in the world", ylabel="Position [m]")
        time = data["time_s"]
        for key, label, color in (("roll_rad", "Roll", ORANGE), ("pitch_rad", "Pitch", PURPLE),
                                   ("yaw_rad", "Yaw (unwrapped)", TEAL)):
            if key in data:
                _line(axes[2], time, np.unwrap(data[key]), label, color)
        axes[2].set(title="Orientation", xlabel="Time [s]", ylabel="Angle [rad]")
    if "left_rad_s" in data:
        _series(axes[3], data, [("left_rad_s", "Left", TEAL), ("right_rad_s", "Right", ORANGE)],
                title="Wheel angular speeds", ylabel="Angular speed [rad/s]")
    else:
        time = data["time_s"]
        if len(time) >= 2 and np.all(np.diff(time) > 0):
            speed = np.hypot(np.gradient(data["x_m"], time), np.gradient(data["y_m"], time))
            _line(axes[3], time, speed, "From recorded positions", TEAL)
        else:
            axes[3].text(.5, .5, "At least two samples are needed\nto estimate speed from position.",
                         ha="center", va="center", transform=axes[3].transAxes, color=SLATE)
        axes[3].set(title="Planar speed estimate", xlabel="Time [s]", ylabel="Speed [m/s]")


def _estimation(axes, data):
    _trajectory(axes[0], data)
    _trajectory(axes[0], data, x="enc_x_m", y="enc_y_m", label="Encoder", color=ORANGE)
    _trajectory(axes[0], data, x="imu_x_m", y="imu_y_m", label="IMU", color=PURPLE)
    _xy_labels(axes[0], "Actual and reconstructed trajectories")
    sample_time = data.get("sensor_time_s", data["time_s"])
    selection = np.r_[True, np.diff(sample_time) > 1e-10]
    # Between acquisitions, estimates describe an earlier instant than the
    # current physics pose. Compare errors only when both timestamps match.
    aligned = selection & np.isclose(data["time_s"], sample_time, rtol=0, atol=1e-9)
    for key, label, color in (("enc_position_error_m", "Encoder", ORANGE),
                               ("imu_position_error_m", "IMU", PURPLE)):
        if key in data:
            _line(axes[1], sample_time[aligned], data[key][aligned], label, color)
    axes[1].set(title="Position errors at sensor acquisitions", xlabel="Time [s]", ylabel="Error [m]")
    time = sample_time[selection]
    if "enc_speed_m_s" in data:
        _line(axes[2], time, np.abs(data["enc_speed_m_s"])[selection], "Encoder (interval mean)", ORANGE, step=True)
    if "imu_vx_m_s" in data:
        magnitude = np.hypot(data["imu_vx_m_s"], data["imu_vy_m_s"])
        _line(axes[2], time, magnitude[selection], "IMU (integrated)", PURPLE, step=True)
    axes[2].set(title="Speed magnitude at sensor samples", xlabel="Time [s]", ylabel="Speed [m/s]")
    _line(axes[3], data["time_s"], np.unwrap(data["yaw_rad"]), "Actual", TEAL)
    for key, label, color in (("enc_yaw_rad", "Encoder", ORANGE), ("imu_yaw_rad", "IMU", PURPLE)):
        _line(axes[3], time, np.unwrap(data[key][selection]), label, color, step=True)
    axes[3].set(title="Orientation estimates (unwrapped)", xlabel="Time [s]", ylabel="Yaw [rad]")


def _front_range(ax, sensors):
    ranges = sorted((key for key in sensors if re.fullmatch(r"range_\d+_m", key)),
                    key=lambda key: int(key.split("_")[1]))
    if not ranges:
        return
    key = ranges[len(ranges) // 2]
    index = key.split("_")[1]
    time, values = sensors["time_s"], sensors[key]
    hits = sensors.get(f"hit_{index}", np.ones(len(time))) == 1
    _line(ax, time, np.where(hits, values, np.nan), "Measured hit", TEAL)
    ideal = sensors.get(f"true_{key}")
    if ideal is not None:
        _line(ax, time, np.where(hits, ideal, np.nan), "Ideal hit", SLATE, style="--")
    misses = (~hits) & np.isfinite(values)
    if np.any(misses):
        ax.scatter(time[misses], values[misses], s=22, marker="x", color=SLATE,
                   label="No return (range limit)")
    ax.set(title="Forward range: hits and misses", xlabel="Time [s]", ylabel="Range [m]")


def _raw_sensors(axes, sensors, snapshot):
    task = snapshot["task"]
    if task == "sensor_lidar":
        names = sorted((key for key in sensors if re.fullmatch(r"range_\d+_m", key)),
                       key=lambda key: int(key.split("_")[1]))
        if names:
            metadata = snapshot.get("metadata", {})
            angles = np.asarray(metadata.get("lidar_angles", [0] if len(names) == 1 else
                                             np.linspace(-np.pi / 2, np.pi / 2, len(names))))
            ranges = np.array([sensors[key][-1] for key in names])
            hits = np.array([sensors.get(f"hit_{key.split('_')[1]}", np.ones(1))[-1] == 1
                             for key in names])
            if angles.shape != ranges.shape:
                raise ValueError("Recorded lidar angles must match the range columns.")
            for selection, label, color, marker in ((hits, "Hit", GREEN, "o"),
                                                    (~hits, "No return", SLATE, "x")):
                if np.any(selection):
                    axes[0].scatter(ranges[selection] * np.cos(angles[selection]),
                                    ranges[selection] * np.sin(angles[selection]),
                                    label=label, color=color, marker=marker, s=24)
            _xy_labels(axes[0], "Latest scan in the sensor frame")
            counts = np.sum([sensors[f"hit_{key.split('_')[1]}"] == 1 for key in names], axis=0)
            _line(axes[2], sensors["time_s"], counts, "Hits", GREEN, step=True)
            axes[2].set(title="Returns per scan", xlabel="Time [s]", ylabel="Ray count")
        _front_range(axes[1], sensors)
        _series(axes[3], sensors, [("gyro_z_rad_s", "Measured", TEAL),
                                  ("true_gyro_z_rad_s", "Ideal", SLATE)],
                title="Yaw rate", ylabel="Angular speed [rad/s]")
        return
    if task == "sensor_imu":
        _series(axes[0], sensors, [("gyro_z_rad_s", "Measured", TEAL),
                                  ("true_gyro_z_rad_s", "Ideal", SLATE)],
                title="Yaw rate", ylabel="Angular speed [rad/s]")
        for ax, axis, color in ((axes[1], "x", TEAL), (axes[2], "y", ORANGE)):
            _series(ax, sensors, [(f"accel_{axis}_m_s2", "Measured", color),
                                  (f"true_accel_{axis}_m_s2", "Ideal", SLATE)],
                    title=f"Body {axis} acceleration", ylabel="Acceleration [m/s²]")
        encoder_axis = axes[3]
    else:
        encoder_axis = axes[0]
        _front_range(axes[1], sensors)
        _series(axes[2], sensors, [("gyro_z_rad_s", "Measured", TEAL),
                                  ("true_gyro_z_rad_s", "Ideal", SLATE)],
                title="Yaw rate", ylabel="Angular speed [rad/s]")
        _series(axes[3], sensors, [("accel_x_m_s2", "Body x", TEAL),
                                  ("accel_y_m_s2", "Body y", ORANGE)],
                title="Body acceleration", ylabel="Acceleration [m/s²]")
    _series(encoder_axis, sensors, [("left_delta_rad", "Left measured", TEAL),
                                   ("right_delta_rad", "Right measured", ORANGE)],
            title="Encoder increments per sensor interval", ylabel="Angle increment [rad]")
    for key, label, color in (("true_left_delta_rad", "Left ideal", TEAL),
                               ("true_right_delta_rad", "Right ideal", ORANGE)):
        _line(encoder_axis, sensors["time_s"], sensors.get(key), label, color, style="--")


def _avoidance(axes, data, snapshot):
    _trajectory(axes[0], data)
    goals = snapshot.get("goals", ())
    if not goals and "target_x_m" in data:
        goals = ((data["target_x_m"][-1], data["target_y_m"][-1]),)
    _goals(axes[0], goals)
    _xy_labels(axes[0])
    _series(axes[1], data, [("goal_error_m", "Distance to goal", TEAL)],
            title="Progress toward the goal", ylabel="Distance [m]")
    _series(axes[2], data, [("front_m", "Front", TEAL), ("left_m", "Left", ORANGE),
                           ("right_m", "Right", PURPLE)],
            title="Sector distances used by the supervisor", ylabel="Range [m]", step=True)
    _series(axes[3], data, [("left_rad_s", "Left", TEAL), ("right_rad_s", "Right", ORANGE)],
            title="Wheel angular speeds", ylabel="Angular speed [rad/s]")


def _arm(axes, data, snapshot):
    metadata = snapshot.get("metadata", {})
    target = metadata.get("arm_target")
    if target is None and "target_x_m" in data:
        target = (data["target_x_m"][-1], data["target_y_m"][-1])
    branches = metadata.get("arm_branches", ())
    points = metadata.get("arm_points", ())
    if len(points):
        chain = np.asarray(points)
        axes[0].plot(chain[:, 0], chain[:, 1], "o--", color=SLATE, linewidth=1.2,
                     markersize=4, label="Arm configuration")
    for index, branch in enumerate(branches):
        chain = np.asarray(branch)
        axes[0].plot(chain[:, 0], chain[:, 1], "o-", color=TEAL if index == 0 else PURPLE,
                     linewidth=2, markersize=5, label=f"Branch {index + 1}")
    for index, prediction in enumerate(metadata.get("arm_predictions", ())):
        axes[0].scatter(*prediction[:2], color=PURPLE, marker="x", s=65,
                        label="Model prediction" if index == 0 else None)
    if snapshot["task"] == "arm_control":
        _trajectory(axes[0], data, x="tip_x_m", y="tip_y_m", label="Endpoint", color=TEAL)
    if target is not None:
        _goals(axes[0], (target[:2],))
    _xy_labels(axes[0], "Endpoint and arm geometry")
    if snapshot["task"] == "arm_control":
        _series(axes[1], data, [("q1_rad", "q1", TEAL), ("q2_rad", "q2", ORANGE)],
                title="Joint angles", ylabel="Angle [rad]")
        _series(axes[2], data, [("residual_m", "Endpoint residual", PURPLE)],
                title="Position residual", ylabel="Error [m]")
        _series(axes[3], data, [("q1_dot_rad_s", "Joint 1", TEAL),
                               ("q2_dot_rad_s", "Joint 2", ORANGE)],
                title="Joint angular speeds", ylabel="Angular speed [rad/s]")
        return
    angles, residuals = [], []
    for branch in branches:
        chain = np.asarray(branch)
        first, second = chain[1, :2] - chain[0, :2], chain[2, :2] - chain[1, :2]
        q1 = np.arctan2(first[1], first[0])
        q2 = (np.arctan2(second[1], second[0]) - q1 + np.pi) % (2 * np.pi) - np.pi
        angles.append((q1, q2))
        residuals.append(np.linalg.norm(chain[-1, :2] - np.asarray(target)[:2]))
    if angles:
        x = np.arange(len(angles))
        angles = np.asarray(angles)
        axes[1].bar(x - .17, angles[:, 0], .34, label="q1", color=TEAL)
        axes[1].bar(x + .17, angles[:, 1], .34, label="q2", color=ORANGE)
        axes[1].set_xticks(x, [f"Branch {i + 1}" for i in x])
        axes[2].bar(x, residuals, color=PURPLE, width=.5, label="Endpoint residual")
        axes[2].set_xticks(x, [f"Branch {i + 1}" for i in x])
    else:
        axes[1].bar(["q1", "q2"], [data["q1_rad"][-1], data["q2_rad"][-1]], color=[TEAL, ORANGE])
        key = "model_error_m" if snapshot["task"] == "arm_forward" else "residual_m"
        value = data.get(key, np.array([np.nan]))[-1]
        if np.isfinite(value):
            axes[2].bar(["Recorded residual"], [value], color=PURPLE)
        else:
            axes[2].text(.5, .5, "No finite residual was recorded.", ha="center", va="center",
                         color=SLATE, transform=axes[2].transAxes)
    axes[1].set(title="Joint configurations", ylabel="Angle [rad]")
    axes[2].set(title="Geometric verification", ylabel="Error [m]")
    axes[3].axis("off")
    outcome = snapshot.get("outcome") or "Recorded"
    detail = f"Outcome: {outcome}\n\nBranches returned: {len(branches)}"
    if target is not None:
        detail += f"\nTarget: ({target[0]:.3f}, {target[1]:.3f}) m"
    if outcome in ("UNREACHABLE", "DEGENERATE", "CHECK"):
        detail += "\n\nInspect reachability, branch count\nand the reported model error."
    axes[3].text(.05, .9, detail, transform=axes[3].transAxes, va="top", color=SLATE,
                 fontsize=11, linespacing=1.6)


def create_experiment_figure(snapshot):
    """Return a labeled 2×2 figure for one frozen experiment dictionary.

    Required fields are ``task``, ``columns`` and nonempty ``samples``. Optional
    sensor rows use their own acquisition times, never physics timestamps.
    """
    if not snapshot.get("samples"):
        raise ValueError("Run or step an activity before opening its plots.")
    data = _numeric_table(snapshot["columns"], snapshot["samples"])
    if "time_s" not in data or not np.any(np.isfinite(data["time_s"])):
        raise ValueError("The experiment has no valid recorded timestamps.")
    sensors = _numeric_table(snapshot.get("sensor_columns", ()), snapshot.get("sensor_samples", ()))
    figure = Figure(figsize=(11.2, 7.4), dpi=100, layout="constrained", facecolor="#f5f8fa")
    axes = figure.subplots(2, 2).ravel()
    task = snapshot["task"]
    if task.startswith("arm_"):
        _arm(axes, data, snapshot)
    elif task.startswith("sensor_estimation_"):
        _estimation(axes, data)
    elif task == "sensor_avoidance":
        _avoidance(axes, data, snapshot)
    elif task.startswith("sensor_") and len(sensors.get("time_s", ())):
        _raw_sensors(axes, sensors, snapshot)
    else:
        _base(axes, data, snapshot)
    outcome = snapshot.get("outcome")
    suffix = f" · {outcome}" if outcome else ""
    figure.suptitle(f"{snapshot.get('title', task)}{suffix}", fontsize=14, fontweight="bold", color="#203545")
    for ax in axes:
        ax.set_facecolor("white")
        ax.grid(True, color="#dce5eb", linewidth=.7, alpha=.8)
        ax.set_axisbelow(True)
        ax.tick_params(labelsize=8, colors="#536a7a")
        ax.title.set_fontsize(10)
        ax.title.set_color("#203545")
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("bottom", "left"):
            ax.spines[side].set_color("#bdcbd5")
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(handles, labels, fontsize=8, frameon=False, loc="best")
    return figure
