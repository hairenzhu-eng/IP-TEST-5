"""Generate APF trajectory snapshots from obstacle JSON logs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Ellipse, Patch
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOGS_DIR = PROJECT_ROOT / "logs"
DEFAULT_OUTPUT_DIR = DEFAULT_LOGS_DIR / "generated_figures"
DEFAULT_K_GOAL = 1.0
DEFAULT_K_OBSTACLE = 150.0


def point(value):
    try:
        value = np.asarray(value, dtype=float).reshape(2)
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value).all() else None


def point_cloud(value):
    try:
        cloud = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return np.empty((0, 2), dtype=float)
    if cloud.ndim != 2 or cloud.shape[1] < 2:
        return np.empty((0, 2), dtype=float)
    cloud = cloud[:, :2]
    return cloud[np.isfinite(cloud).all(axis=1)]


def positive(value, fallback):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return float(fallback)
    return value if np.isfinite(value) and value > 0.0 else float(fallback)


def load_snapshots(run_dir):
    snapshots = []
    for path in sorted(Path(run_dir).glob("obstacle_*.json")):
        with path.open(encoding="utf-8") as stream:
            payload = json.load(stream)
        try:
            time_s = float(payload.get("t"))
        except (TypeError, ValueError):
            continue
        if np.isfinite(time_s) and point(payload.get("robot_pos")) is not None:
            snapshots.append({"path": path, "time_s": time_s, "payload": payload})

    snapshots.sort(key=lambda snapshot: snapshot["time_s"])
    if not snapshots:
        raise ValueError(f"No valid obstacle_*.json snapshots found in {run_dir}")

    start_s = snapshots[0]["time_s"]
    for snapshot in snapshots:
        snapshot["relative_time_s"] = snapshot["time_s"] - start_s
    return snapshots


def select_snapshot_indices(snapshots, interval_s):
    interval_s = positive(interval_s, 5.0)
    duration_s = snapshots[-1]["relative_time_s"]
    targets = np.arange(0.0, duration_s + 1e-9, interval_s)
    times = np.asarray(
        [snapshot["relative_time_s"] for snapshot in snapshots],
        dtype=float,
    )
    indices = []
    for target_s in targets:
        index = int(np.argmin(np.abs(times - target_s)))
        if not indices or index != indices[-1]:
            indices.append(index)
    return indices


def obstacle_dimensions(obstacle, settings):
    minimum_pc1_m = positive(settings.get("minimum_pc1_m"), 0.30)
    minimum_pc2_m = positive(settings.get("minimum_pc2_m"), 0.16)
    equivalent_radius_m = positive(
        obstacle.get("equivalent_radius_m"),
        0.5 * minimum_pc1_m,
    )
    pc1_m = positive(obstacle.get("pc1_m"), 2.0 * equivalent_radius_m)
    pc2_m = positive(obstacle.get("pc2_m"), min(pc1_m, 2.0 * equivalent_radius_m))
    return max(pc1_m, minimum_pc1_m), max(min(pc2_m, pc1_m), minimum_pc2_m)


def normalized_axis(value):
    axis = point(value)
    if axis is None:
        return np.array([1.0, 0.0], dtype=float)
    norm = float(np.linalg.norm(axis))
    return axis / norm if norm >= 1e-9 else np.array([1.0, 0.0], dtype=float)


def obstacle_ellipse_geometry(obstacle, settings):
    centre = point(obstacle.get("centre_ne"))
    if centre is None:
        return None
    pc1_m, pc2_m = obstacle_dimensions(obstacle, settings)
    axis_ne = normalized_axis(obstacle.get("length_axis_ne"))
    angle_deg = float(np.degrees(np.arctan2(axis_ne[0], axis_ne[1])))
    return centre, pc1_m, pc2_m, angle_deg


def attractive_potential(north_grid, east_grid, target_ne, k_goal):
    target_ne = point(target_ne)
    if target_ne is None:
        return np.zeros_like(north_grid)
    return 0.5 * float(k_goal) * (
        (north_grid - target_ne[0]) ** 2
        + (east_grid - target_ne[1]) ** 2
    )


def classic_repulsive_potential(
    distance_grid,
    influence_distance_m,
    repulsive_gain,
):
    influence_distance_m = positive(influence_distance_m, 3.0)
    distance_grid = np.asarray(distance_grid, dtype=float)
    safe_distance = np.maximum(distance_grid, 1e-3)
    potential = np.zeros_like(safe_distance)
    active = safe_distance <= influence_distance_m
    potential[active] = 0.5 * float(repulsive_gain) * (
        1.0 / safe_distance[active] - 1.0 / influence_distance_m
    ) ** 2
    return potential


def ellipse_potential(
    north_grid,
    east_grid,
    obstacle,
    settings,
    k_obstacle=DEFAULT_K_OBSTACLE,
):
    centre = point(obstacle.get("centre_ne"))
    if centre is None:
        return np.zeros_like(north_grid)

    delta_n = north_grid - centre[0]
    delta_e = east_grid - centre[1]
    if settings.get("cluster_range_enabled", True) is False:
        return classic_repulsive_potential(
            np.hypot(delta_n, delta_e),
            settings.get("classic_influence_distance_m"),
            k_obstacle,
        )

    pc1_m, pc2_m = obstacle_dimensions(obstacle, settings)
    scale = positive(
        settings.get("avoidance_pc_scale"),
        settings.get("cluster_influence_scale", 6.0),
    )
    semi_length_m = max(0.5 * scale * pc1_m, 1e-6)
    semi_width_m = max(0.5 * scale * pc2_m, 1e-6)
    axis = normalized_axis(obstacle.get("length_axis_ne"))
    width_axis = np.array([-axis[1], axis[0]], dtype=float)

    along = delta_n * axis[0] + delta_e * axis[1]
    across = delta_n * width_axis[0] + delta_e * width_axis[1]
    # Notebook repulsive potential:
    # Uo = ko * exp(-((x-xo)^4 + (y-yo)^4) / ko).
    # The local coordinates are normalized by the pc1/pc2 APF domain so the
    # same equation represents an oriented obstacle ship instead of a point.
    exponent = -(
        (along / semi_length_m) ** 4
        + (across / semi_width_m) ** 4
    )
    return float(k_obstacle) * np.exp(exponent)


def segment_potential(
    north_grid,
    east_grid,
    obstacle,
    settings,
    k_obstacle=DEFAULT_K_OBSTACLE,
):
    start = point(obstacle.get("segment_start_ne"))
    end = point(obstacle.get("segment_end_ne"))
    if start is None or end is None:
        return np.zeros_like(north_grid)

    pc1_m, pc2_m = obstacle_dimensions(obstacle, settings)
    scale = positive(
        settings.get("virtual_pc_scale"),
        settings.get("avoidance_pc_scale", 6.0),
    )
    segment = end - start
    segment_length_m = float(np.linalg.norm(segment))
    cluster_range_enabled = settings.get("cluster_range_enabled", True) is not False
    if cluster_range_enabled and segment_length_m >= 1e-9:
        axis = segment / segment_length_m
        extension_m = max(0.5 * scale * pc1_m, 1e-6)
        start = start - extension_m * axis
        end = end + extension_m * axis
        segment = end - start

    segment_length_sq = max(float(np.dot(segment, segment)), 1e-12)
    delta_n = north_grid - start[0]
    delta_e = east_grid - start[1]
    ratio = np.clip(
        (delta_n * segment[0] + delta_e * segment[1]) / segment_length_sq,
        0.0,
        1.0,
    )
    closest_n = start[0] + ratio * segment[0]
    closest_e = start[1] + ratio * segment[1]
    distance_grid = np.hypot(
        north_grid - closest_n,
        east_grid - closest_e,
    )
    if not cluster_range_enabled:
        return classic_repulsive_potential(
            distance_grid,
            settings.get("classic_influence_distance_m"),
            k_obstacle,
        )

    corridor_radius_m = max(0.5 * scale * pc2_m, 1e-6)
    level = distance_grid / corridor_radius_m
    return float(k_obstacle) * np.exp(-(level ** 4))


def potential_components(
    north_grid,
    east_grid,
    payload,
    default_target_ne,
    k_goal,
    k_obstacle,
):
    settings = payload.get("apf_settings", {})
    settings = settings if isinstance(settings, dict) else {}
    apf = payload.get("apf", {})
    apf = apf if isinstance(apf, dict) else {}
    target_ne = point(apf.get("target_ne"))
    if target_ne is None:
        target_ne = point(default_target_ne)
    attractive = attractive_potential(
        north_grid,
        east_grid,
        target_ne,
        k_goal,
    )
    real = np.zeros_like(north_grid)
    virtual = np.zeros_like(north_grid)

    for obstacle in payload.get("clusters", []):
        if isinstance(obstacle, dict):
            real += ellipse_potential(
                north_grid,
                east_grid,
                obstacle,
                settings,
                k_obstacle,
            )

    for obstacle in payload.get("virtual_obstacles", []):
        if not isinstance(obstacle, dict):
            continue
        if "segment_start_ne" in obstacle and "segment_end_ne" in obstacle:
            virtual += segment_potential(
                north_grid,
                east_grid,
                obstacle,
                settings,
                k_obstacle,
            )
        else:
            virtual += ellipse_potential(
                north_grid,
                east_grid,
                obstacle,
                settings,
                k_obstacle,
            )

    return attractive, real, virtual, target_ne


def all_run_points(snapshots):
    points = []
    for snapshot in snapshots:
        payload = snapshot["payload"]
        candidate = point(payload.get("robot_pos"))
        if candidate is not None:
            points.append(candidate)
        cloud = point_cloud(payload.get("cloud", []))
        if len(cloud):
            points.extend(cloud)
        for collection_name in ("clusters", "tracks"):
            for item in payload.get(collection_name, []):
                if not isinstance(item, dict):
                    continue
                candidate = point(
                    item.get("centre_ne", item.get("position_ne"))
                )
                if candidate is not None:
                    points.append(candidate)
        for virtual in payload.get("virtual_obstacles", []):
            if not isinstance(virtual, dict):
                continue
            for key in ("segment_start_ne", "segment_end_ne", "centre_ne"):
                candidate = point(virtual.get(key))
                if candidate is not None:
                    points.append(candidate)
    return np.asarray(points, dtype=float)


def run_bounds(snapshots, map_size_m=20.0):
    points = all_run_points(snapshots)
    if len(points) == 0:
        return (-2.0, 2.0, -2.0, 2.0)
    north_min, east_min = np.min(points, axis=0)
    north_max, east_max = np.max(points, axis=0)
    north_centre = 0.5 * (north_min + north_max)
    east_centre = 0.5 * (east_min + east_max)
    half_span_m = 0.5 * positive(map_size_m, 20.0)
    return (
        north_centre - half_span_m,
        north_centre + half_span_m,
        east_centre - half_span_m,
        east_centre + half_span_m,
    )


def prediction_points(track):
    for key in ("prediction_ne", "predicted_trajectory_ne"):
        try:
            values = np.asarray(track.get(key, []), dtype=float)
        except (TypeError, ValueError):
            continue
        if values.ndim == 2 and values.shape[1] >= 2:
            values = values[:, :2]
            return values[np.isfinite(values).all(axis=1)]
    return np.empty((0, 2), dtype=float)


def plot_snapshot(
    snapshot,
    robot_history,
    bounds,
    output_path,
    grid_size,
    target_time_s,
    default_target_ne,
    k_goal,
    k_obstacle,
    quiver_step,
):
    payload = snapshot["payload"]
    north_min, north_max, east_min, east_max = bounds
    north_axis = np.linspace(north_min, north_max, grid_size)
    east_axis = np.linspace(east_min, east_max, grid_size)
    east_grid, north_grid = np.meshgrid(east_axis, north_axis)
    (
        attractive_potential_map,
        real_potential,
        virtual_potential,
        target_ne,
    ) = potential_components(
        north_grid,
        east_grid,
        payload,
        default_target_ne,
        k_goal,
        k_obstacle,
    )
    total_potential = (
        attractive_potential_map
        + real_potential
        + virtual_potential
    )

    fig, ax = plt.subplots(figsize=(10, 8), dpi=160)
    minimum = float(np.min(total_potential))
    maximum = float(np.max(total_potential))
    if maximum - minimum > 1e-9:
        contour = ax.contourf(
            east_grid,
            north_grid,
            total_potential,
            levels=np.linspace(minimum, maximum, 32),
            cmap="coolwarm",
            alpha=0.72,
            vmin=minimum,
            vmax=maximum,
        )
        colorbar = fig.colorbar(contour, ax=ax, pad=0.02)
        colorbar.set_label("Total APF potential, U")

    virtual_max = float(np.max(virtual_potential))
    if virtual_max > 1e-9:
        normalized_virtual = np.clip(virtual_potential / virtual_max, 0.0, 1.0)
        ax.contourf(
            east_grid,
            north_grid,
            normalized_virtual,
            levels=[0.05, 0.2, 0.4, 0.6, 0.8, 1.01],
            colors=["#00d5ff"],
            alpha=0.18,
            zorder=2,
        )
        ax.contour(
            east_grid,
            north_grid,
            normalized_virtual,
            levels=[0.2, 0.4, 0.6, 0.8],
            colors="#00a8cc",
            linewidths=0.8,
            alpha=0.75,
            zorder=2,
        )

    settings = payload.get("apf_settings", {})
    settings = settings if isinstance(settings, dict) else {}
    prediction_horizon_s = positive(
        settings.get("obstacle_prediction_horizon_s"),
        30.0,
    )
    domain_level = float(k_obstacle) / np.e
    for obstacle in payload.get("clusters", []):
        if not isinstance(obstacle, dict):
            continue
        field = ellipse_potential(
            north_grid,
            east_grid,
            obstacle,
            settings,
            k_obstacle,
        )
        if float(np.min(field)) <= domain_level <= float(np.max(field)):
            ax.contour(
                east_grid,
                north_grid,
                field,
                levels=[domain_level],
                colors="black",
                linewidths=1.1,
            )

    for obstacle in payload.get("virtual_obstacles", []):
        if not isinstance(obstacle, dict):
            continue
        if "segment_start_ne" in obstacle and "segment_end_ne" in obstacle:
            field = segment_potential(
                north_grid,
                east_grid,
                obstacle,
                settings,
                k_obstacle,
            )
        else:
            field = ellipse_potential(
                north_grid,
                east_grid,
                obstacle,
                settings,
                k_obstacle,
            )
        if float(np.min(field)) <= domain_level <= float(np.max(field)):
            ax.contour(
                east_grid,
                north_grid,
                field,
                levels=[domain_level],
                colors="#00d5ff",
                linewidths=1.5,
                linestyles="--",
            )

    gradient_north, gradient_east = np.gradient(
        total_potential,
        north_axis,
        east_axis,
    )
    gradient_norm = np.hypot(gradient_east, gradient_north)
    finite_gradient = gradient_norm > 1e-9
    descent_east = np.zeros_like(gradient_east)
    descent_north = np.zeros_like(gradient_north)
    descent_east[finite_gradient] = (
        -gradient_east[finite_gradient] / gradient_norm[finite_gradient]
    )
    descent_north[finite_gradient] = (
        -gradient_north[finite_gradient] / gradient_norm[finite_gradient]
    )
    step = max(int(quiver_step), 1)
    ax.quiver(
        east_grid[::step, ::step],
        north_grid[::step, ::step],
        descent_east[::step, ::step],
        descent_north[::step, ::step],
        color="black",
        alpha=0.28,
        pivot="mid",
        scale=35,
        width=0.002,
        zorder=3,
    )

    history = np.asarray(robot_history, dtype=float)
    ax.plot(
        history[:, 1],
        history[:, 0],
        color="#1f77b4",
        linewidth=2.2,
        label="OS trajectory",
        zorder=5,
    )
    robot_position = point(payload.get("robot_pos"))
    ax.scatter(
        [robot_position[1]],
        [robot_position[0]],
        marker="*",
        s=180,
        color="#1f77b4",
        edgecolor="white",
        linewidth=0.8,
        label="OS current position",
        zorder=8,
    )
    cloud = point_cloud(payload.get("cloud", []))
    if len(cloud):
        ax.scatter(
            cloud[:, 1],
            cloud[:, 0],
            marker=".",
            s=8,
            color="#7f7f7f",
            alpha=0.65,
            label="LiDAR point cloud (earth frame)",
            zorder=4,
        )
    if target_ne is not None:
        ax.scatter(
            [target_ne[1]],
            [target_ne[0]],
            marker="X",
            s=90,
            color="#39ff14",
            edgecolor="black",
            linewidth=0.7,
            label="APF attractive target",
            zorder=9,
        )

    clusters = [
        obstacle
        for obstacle in payload.get("clusters", [])
        if isinstance(obstacle, dict) and point(obstacle.get("centre_ne")) is not None
    ]
    if clusters:
        centres = np.asarray(
            [point(obstacle.get("centre_ne")) for obstacle in clusters],
            dtype=float,
        )
        ax.scatter(
            centres[:, 1],
            centres[:, 0],
            marker="s",
            s=75,
            color="#d62728",
            edgecolor="white",
            linewidth=0.7,
            label="Obstacle ship current position",
            zorder=8,
        )
        for cluster_index, obstacle in enumerate(clusters):
            geometry = obstacle_ellipse_geometry(obstacle, settings)
            if geometry is None:
                continue
            centre_ne, pc1_m, pc2_m, angle_deg = geometry
            ax.add_patch(
                Ellipse(
                    xy=(centre_ne[1], centre_ne[0]),
                    width=pc1_m,
                    height=pc2_m,
                    angle=angle_deg,
                    facecolor="#d62728",
                    edgecolor="white",
                    linewidth=1.2,
                    alpha=0.35,
                    label=(
                        "LiDAR clustered obstacle size (pc1 × pc2)"
                        if cluster_index == 0
                        else None
                    ),
                    zorder=7,
                )
            )
            ax.annotate(
                f"pc1={pc1_m:.2f} m\npc2={pc2_m:.2f} m",
                xy=(centre_ne[1], centre_ne[0]),
                xytext=(7, 7),
                textcoords="offset points",
                fontsize=7,
                color="#7f0000",
                bbox={
                    "boxstyle": "round,pad=0.2",
                    "facecolor": "white",
                    "alpha": 0.72,
                    "edgecolor": "none",
                },
                zorder=10,
            )

    for track_index, track in enumerate(payload.get("tracks", [])):
        if not isinstance(track, dict):
            continue
        position = point(track.get("position_ne"))
        if position is not None:
            ax.scatter(
                [position[1]],
                [position[0]],
                marker="o",
                s=55,
                facecolors="none",
                edgecolors="#2ca02c",
                linewidth=1.4,
                label="Obstacle EKF position" if track_index == 0 else None,
                zorder=8,
            )
        prediction = prediction_points(track)
        if len(prediction) >= 2:
            ax.plot(
                prediction[:, 1],
                prediction[:, 0],
                color="#ff7f0e",
                linestyle="--",
                linewidth=2.0,
                marker=".",
                markersize=4,
                label=(
                    f"EKF predicted trajectory ({prediction_horizon_s:g} s)"
                    if track_index == 0
                    else None
                ),
                zorder=7,
            )
            ax.scatter(
                [prediction[-1, 1]],
                [prediction[-1, 0]],
                marker="X",
                s=65,
                color="#ff7f0e",
                edgecolor="black",
                linewidth=0.6,
                label=(
                    f"Obstacle position at +{prediction_horizon_s:g} s"
                    if track_index == 0
                    else None
                ),
                zorder=8,
            )

    for virtual_index, virtual in enumerate(payload.get("virtual_obstacles", [])):
        if not isinstance(virtual, dict):
            continue
        start = point(virtual.get("segment_start_ne"))
        end = point(virtual.get("segment_end_ne"))
        if start is None or end is None:
            continue
        ax.plot(
            [start[1], end[1]],
            [start[0], end[0]],
            color="#00d5ff",
            linestyle="--",
            linewidth=2.0,
            label="Virtual potential segment" if virtual_index == 0 else None,
            zorder=6,
        )
        ax.scatter(
            [end[1]],
            [end[0]],
            marker="D",
            s=55,
            color="#00d5ff",
            edgecolor="black",
            linewidth=0.5,
            label="Virtual obstacle position" if virtual_index == 0 else None,
            zorder=8,
        )

    apf = payload.get("apf", {})
    mode = apf.get("navigation_mode", "unknown") if isinstance(apf, dict) else "unknown"
    encounter = apf.get("encounter", "none") if isinstance(apf, dict) else "none"
    sample_time_s = snapshot["relative_time_s"]
    time_text = f"t={target_time_s:.1f} s"
    if abs(sample_time_s - target_time_s) >= 0.05:
        time_text += f" (nearest log sample {sample_time_s:.1f} s)"
    ax.set_title(
        f"APF trajectory snapshot at {time_text}\n"
        f"mode={mode}, encounter={encounter}"
    )
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_xlim(east_min, east_max)
    ax.set_ylim(north_min, north_max)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)

    handles, labels = ax.get_legend_handles_labels()
    if float(np.max(real_potential)) > 1e-9:
        handles.append(Line2D([0], [0], color="black", linewidth=1.0))
        labels.append("Measured-obstacle potential boundary")
    if float(np.max(virtual_potential)) > 1e-9:
        handles.append(
            Patch(
                facecolor="#00d5ff",
                edgecolor="#00a8cc",
                alpha=0.3,
            )
        )
        labels.append("Virtual repulsive potential field")
        handles.append(
            Line2D([0], [0], color="#00d5ff", linestyle="--", linewidth=1.5)
        )
        labels.append("Virtual potential boundary")
    handles.append(
        Line2D(
            [0],
            [0],
            color="black",
            marker=r"$\rightarrow$",
            linestyle="none",
            alpha=0.45,
        )
    )
    labels.append("Negative potential gradient")
    unique = {}
    for handle, label in zip(handles, labels):
        if label and label not in unique:
            unique[label] = handle
    ax.legend(
        unique.values(),
        unique.keys(),
        loc="upper left",
        fontsize=8,
        framealpha=0.9,
    )

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path)
    plt.close(fig)


def latest_run_dir(logs_dir):
    candidates = [
        path
        for path in Path(logs_dir).glob("run_*")
        if path.is_dir() and any(path.glob("obstacle_*.json"))
    ]
    if not candidates:
        raise FileNotFoundError(f"No run directory with obstacle JSON logs in {logs_dir}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def generate_snapshots(
    run_dir,
    output_dir=None,
    interval_s=5.0,
    grid_size=240,
    k_goal=DEFAULT_K_GOAL,
    k_obstacle=DEFAULT_K_OBSTACLE,
    quiver_step=14,
    map_size_m=20.0,
):
    run_dir = Path(run_dir)
    snapshots = load_snapshots(run_dir)
    selected_indices = select_snapshot_indices(snapshots, interval_s)
    output_dir = (
        Path(output_dir)
        if output_dir is not None
        else DEFAULT_OUTPUT_DIR / f"{run_dir.name}_apf_snapshots"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    for old_snapshot in output_dir.glob("apf_snapshot_*.png"):
        old_snapshot.unlink()
    bounds = run_bounds(snapshots, map_size_m)
    default_target_ne = point(snapshots[-1]["payload"].get("robot_pos"))

    robot_history = []
    outputs = []
    selected_targets = {
        index: selected_order * positive(interval_s, 5.0)
        for selected_order, index in enumerate(selected_indices)
    }
    for index, snapshot in enumerate(snapshots):
        robot_position = point(snapshot["payload"].get("robot_pos"))
        if robot_position is not None:
            robot_history.append(robot_position)
        if index not in selected_targets:
            continue
        target_time_s = selected_targets[index]
        output_path = output_dir / f"apf_snapshot_{target_time_s:06.1f}s.png"
        plot_snapshot(
            snapshot=snapshot,
            robot_history=robot_history,
            bounds=bounds,
            output_path=output_path,
            grid_size=max(int(grid_size), 80),
            target_time_s=target_time_s,
            default_target_ne=default_target_ne,
            k_goal=float(k_goal),
            k_obstacle=float(k_obstacle),
            quiver_step=quiver_step,
        )
        outputs.append(output_path)
    return outputs


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Generate APF snapshots containing the earth-frame LiDAR cloud, "
            "OS history, obstacle positions, and EKF predictions."
        )
    )
    parser.add_argument(
        "--run-dir",
        type=Path,
        help="Run directory containing obstacle_*.json; defaults to latest run.",
    )
    parser.add_argument("--logs-dir", type=Path, default=DEFAULT_LOGS_DIR)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--grid-size", type=int, default=240)
    parser.add_argument("--k-goal", type=float, default=DEFAULT_K_GOAL)
    parser.add_argument("--k-obstacle", type=float, default=DEFAULT_K_OBSTACLE)
    parser.add_argument("--quiver-step", type=int, default=14)
    parser.add_argument(
        "--map-size",
        type=float,
        default=20.0,
        help="Square map width and height in metres.",
    )
    args = parser.parse_args()

    run_dir = args.run_dir or latest_run_dir(args.logs_dir)
    outputs = generate_snapshots(
        run_dir=run_dir,
        output_dir=args.output_dir,
        interval_s=args.interval,
        grid_size=args.grid_size,
        k_goal=args.k_goal,
        k_obstacle=args.k_obstacle,
        quiver_step=args.quiver_step,
        map_size_m=args.map_size,
    )
    print(f"Run: {run_dir}")
    print(f"Generated {len(outputs)} snapshots")
    for path in outputs:
        print(path)


if __name__ == "__main__":
    main()
