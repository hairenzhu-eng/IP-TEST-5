"""Overlay avoidance trajectories for large and small obstacle ships.

The script scans recent valid ``run_*`` logs, estimates one obstacle-size
metric per run from the active obstacle snapshots, and automatically labels
each trajectory as large or small from the log data before plotting.
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
import re

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOGS_DIR = PROJECT_ROOT / "logs"
DEFAULT_OUTPUT_DIR = DEFAULT_LOGS_DIR / "generated_figures" / "colreg_size_trajectories"
DEFAULT_LATEST_LIMIT = 5
DEFAULT_RUN_DIRS: list[str] = [
    "run_20260624_003938",
    "run_20260624_004126",
]

TIME_COLUMNS = ("TimeFromStart(s)", "TimeFromStart", "elapsed [s]")
OWN_NORTH_COLUMNS = ("North(m)", "North", "x [m]")
OWN_EAST_COLUMNS = ("East(m)", "East", "y [m]")
@dataclass
class SnapshotSample:
    time_s: float
    obstacle_pc1_m: float
    obstacle_pc2_m: float
    equivalent_radius_m: float


@dataclass
class RunRecord:
    run_dir: Path
    log_path: Path
    trajectory_time_s: np.ndarray
    trajectory_ne_m: np.ndarray
    active_window_s: tuple[float, float]
    median_pc1_m: float
    median_pc2_m: float
    median_equivalent_radius_m: float
    size_metric_m: float
    size_label: str = ""


def parse_float(value):
    if value is None:
        return np.nan
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text or text.lower() in {"none", "nan", "null"}:
        return np.nan
    try:
        return float(text)
    except ValueError:
        return np.nan


def positive_float(value):
    value = parse_float(value)
    return value if np.isfinite(value) and value > 0.0 else np.nan


def first_existing(row, names):
    for name in names:
        if name in row:
            return row[name]
    return None


def point(value):
    if value is None:
        return None
    try:
        point_array = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    if point_array.size < 2:
        return None
    point_array = point_array[:2]
    return point_array if np.isfinite(point_array).all() else None


def first_point(*values):
    for value in values:
        parsed = point(value)
        if parsed is not None:
            return parsed
    return None


def find_primary_csv(run_dir):
    candidates = [
        path
        for path in run_dir.glob("log_*.csv")
        if not path.name.endswith("_pseudo_aruco.csv")
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def read_trajectory(log_path):
    times = []
    points = []
    first_time_s = np.nan

    with log_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"{log_path} has no CSV header")

        for row in reader:
            time_s = parse_float(first_existing(row, TIME_COLUMNS))
            north_m = parse_float(first_existing(row, OWN_NORTH_COLUMNS))
            east_m = parse_float(first_existing(row, OWN_EAST_COLUMNS))
            if not (
                np.isfinite(time_s)
                and np.isfinite(north_m)
                and np.isfinite(east_m)
            ):
                continue
            if not np.isfinite(first_time_s):
                first_time_s = float(time_s)
            times.append(float(time_s))
            points.append([north_m, east_m])

    if not times:
        raise ValueError(f"No valid North/East trajectory samples in {log_path}")

    time_array = np.asarray(times, dtype=float)
    trajectory_ne_m = np.asarray(points, dtype=float)
    order = np.argsort(time_array)
    time_array = time_array[order]
    trajectory_ne_m = trajectory_ne_m[order]
    time_array -= float(first_time_s) if np.isfinite(first_time_s) else time_array[0]
    return time_array, trajectory_ne_m


def obstacle_candidates(payload):
    candidates = []
    for key in ("tracks", "clusters"):
        entries = payload.get(key, [])
        if not isinstance(entries, list):
            continue
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            pc1_m = positive_float(entry.get("pc1_m"))
            if not np.isfinite(pc1_m):
                continue
            pc2_m = positive_float(entry.get("pc2_m"))
            equivalent_radius_m = positive_float(entry.get("equivalent_radius_m"))
            centre_ne = first_point(
                entry.get("position_ne"),
                entry.get("centre_ne"),
                entry.get("measurement_centre_ne"),
                entry.get("virtual_position_ne"),
            )
            candidates.append(
                {
                    "pc1_m": pc1_m,
                    "pc2_m": pc2_m if np.isfinite(pc2_m) else np.nan,
                    "equivalent_radius_m": (
                        equivalent_radius_m if np.isfinite(equivalent_radius_m) else np.nan
                    ),
                    "centre_ne": centre_ne,
                }
            )
    return candidates


def choose_active_obstacle(payload):
    robot_pos = point(payload.get("robot_pos"))
    candidates = obstacle_candidates(payload)
    if not candidates:
        return None
    if robot_pos is None:
        return candidates[0]

    def distance(candidate):
        centre_ne = candidate.get("centre_ne")
        if centre_ne is None:
            return float("inf")
        return float(np.linalg.norm(robot_pos - centre_ne))

    return min(candidates, key=distance)


def collect_snapshot_samples(run_dir):
    samples = []
    for path in sorted(run_dir.glob("obstacle_*.json")):
        try:
            with path.open(encoding="utf-8") as stream:
                payload = json.load(stream)
        except (OSError, json.JSONDecodeError):
            continue

        time_s = parse_float(payload.get("t"))
        if not np.isfinite(time_s):
            continue

        obstacle = choose_active_obstacle(payload)
        if obstacle is None:
            continue

        samples.append(
            SnapshotSample(
                time_s=float(time_s),
                obstacle_pc1_m=float(obstacle["pc1_m"]),
                obstacle_pc2_m=float(obstacle["pc2_m"]),
                equivalent_radius_m=float(obstacle["equivalent_radius_m"]),
            )
        )

    return samples
def median_of(values):
    finite = [float(value) for value in values if np.isfinite(value)]
    if not finite:
        return np.nan
    return float(np.median(np.asarray(finite, dtype=float)))


def build_run_record(run_dir):
    log_path = find_primary_csv(run_dir)
    if log_path is None:
        return None

    samples = collect_snapshot_samples(run_dir)
    if len(samples) < 3:
        return None

    trajectory_time_s, trajectory_ne_m = read_trajectory(log_path)
    active_times = np.asarray([sample.time_s for sample in samples], dtype=float)
    active_window_s = (float(np.min(active_times)), float(np.max(active_times)))

    median_pc1_m = median_of(sample.obstacle_pc1_m for sample in samples)
    median_pc2_m = median_of(sample.obstacle_pc2_m for sample in samples)
    median_equivalent_radius_m = median_of(
        sample.equivalent_radius_m for sample in samples
    )
    size_metric_m = (
        median_pc1_m
        if np.isfinite(median_pc1_m)
        else median_equivalent_radius_m
    )
    if not np.isfinite(size_metric_m):
        return None

    return RunRecord(
        run_dir=run_dir,
        log_path=log_path,
        trajectory_time_s=trajectory_time_s,
        trajectory_ne_m=trajectory_ne_m,
        active_window_s=active_window_s,
        median_pc1_m=median_pc1_m,
        median_pc2_m=median_pc2_m,
        median_equivalent_radius_m=median_equivalent_radius_m,
        size_metric_m=float(size_metric_m),
    )


def latest_run_dirs(logs_dir, latest_limit=DEFAULT_LATEST_LIMIT):
    run_dirs = sorted(
        [
            path
            for path in Path(logs_dir).glob("run_*")
            if path.is_dir() and any(path.glob("obstacle_*.json"))
        ],
        key=lambda path: path.name,
        reverse=True,
    )
    return run_dirs


def selected_run_dirs(logs_dir, run_dirs):
    resolved = []
    for run_dir in run_dirs:
        candidate = Path(run_dir)
        if not candidate.is_absolute():
            candidate = Path(logs_dir) / candidate
        candidate = candidate.resolve()
        if not candidate.is_dir():
            raise FileNotFoundError(f"Run directory not found: {run_dir}")
        resolved.append(candidate)
    return resolved


def size_split_threshold(size_values):
    sorted_values = np.sort(np.asarray(size_values, dtype=float))
    if sorted_values.size < 2:
        return np.nan
    gaps = np.diff(sorted_values)
    if gaps.size == 0:
        return np.nan
    best_index = int(np.argmax(gaps))
    best_gap = float(gaps[best_index])
    median_size = float(np.median(sorted_values))
    minimum_gap = max(0.05, 0.08 * max(median_size, 1e-6))
    if best_gap < minimum_gap:
        return np.nan
    return float(0.5 * (sorted_values[best_index] + sorted_values[best_index + 1]))


def assign_size_labels(records):
    size_values = [record.size_metric_m for record in records if np.isfinite(record.size_metric_m)]
    threshold = size_split_threshold(size_values)
    if not np.isfinite(threshold):
        return False
    has_small = False
    has_large = False
    for record in records:
        if record.size_metric_m <= threshold:
            record.size_label = "Small"
            has_small = True
        else:
            record.size_label = "Large"
            has_large = True
    return has_small and has_large


def segment_for_plot(record, padding_s):
    start_s, end_s = record.active_window_s
    lower_s = max(record.trajectory_time_s[0], start_s - padding_s)
    upper_s = min(record.trajectory_time_s[-1], end_s + padding_s)
    mask = (
        (record.trajectory_time_s >= lower_s)
        & (record.trajectory_time_s <= upper_s)
    )
    if np.count_nonzero(mask) < 2:
        return record.trajectory_ne_m
    return record.trajectory_ne_m[mask]


def merged_output_name(records):
    merged = "__".join(record.run_dir.name for record in records)
    safe_name = re.sub(r"[^A-Za-z0-9._-]+", "_", merged).strip("_")
    return safe_name or "large_vs_small"


def plot_group(records, output_path, padding_s):
    colors = {"Small": "#0072B2", "Large": "#D55E00"}
    fig, ax = plt.subplots(figsize=(9.5, 7.5))

    for record in sorted(records, key=lambda item: (item.size_label, item.size_metric_m, item.run_dir.name)):
        segment_ne_m = segment_for_plot(record, padding_s)
        label = (
            f"{record.size_label} {record.run_dir.name} "
            f"(pc1={record.median_pc1_m:.2f} m)"
        )
        color = colors.get(record.size_label, "#333333")
        ax.plot(
            segment_ne_m[:, 1],
            segment_ne_m[:, 0],
            linewidth=2.0,
            color=color,
            alpha=0.88,
            label=label,
        )
        ax.scatter(
            segment_ne_m[0, 1],
            segment_ne_m[0, 0],
            color=color,
            marker="o",
            s=22,
            alpha=0.8,
        )
        ax.scatter(
            segment_ne_m[-1, 1],
            segment_ne_m[-1, 0],
            color=color,
            marker="x",
            s=42,
            alpha=0.9,
        )
        ax.annotate(
            record.size_label,
            xy=(segment_ne_m[-1, 1], segment_ne_m[-1, 0]),
            xytext=(6, 6),
            textcoords="offset points",
            color=color,
            fontsize=9,
            weight="bold",
        )

    ax.set_title(
        "Avoidance Trajectory Comparison\n"
        "Large vs Small Obstacle Ships"
    )

    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300)
    plt.close(fig)


def collect_grouped_runs(
    logs_dir,
    latest_limit=DEFAULT_LATEST_LIMIT,
    run_dirs=None,
):
    records = []
    collected = 0
    source_run_dirs = (
        selected_run_dirs(logs_dir, run_dirs)
        if run_dirs
        else latest_run_dirs(logs_dir, latest_limit=latest_limit)
    )
    for run_dir in source_run_dirs:
        try:
            record = build_run_record(run_dir)
        except (OSError, ValueError):
            continue
        if record is None:
            continue
        records.append(record)
        collected += 1
        if (
            not run_dirs
            and latest_limit is not None
            and latest_limit > 0
            and collected >= int(latest_limit)
        ):
            break
    return records


def plot_colreg_size_trajectories(
    logs_dir,
    output_dir,
    padding_s=2.0,
    latest_limit=DEFAULT_LATEST_LIMIT,
    run_dirs=None,
):
    records = collect_grouped_runs(
        logs_dir,
        latest_limit=latest_limit,
        run_dirs=run_dirs,
    )
    if not records:
        raise FileNotFoundError("No runs with usable obstacle snapshots were found")

    if len(records) < 2:
        raise ValueError("Need at least two valid runs to compare trajectories")
    if not assign_size_labels(records):
        raise ValueError("Could not split runs into distinct large and small ship cases")

    output_path = output_dir / f"{merged_output_name(records)}.png"
    plot_group(records, output_path, padding_s=padding_s)
    return [(output_path, records)]


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Overlay robot avoidance trajectories under different obstacle-ship "
            "sizes, then label each trajectory as a large-ship or small-ship case."
        )
    )
    parser.add_argument("--logs-dir", type=Path, default=DEFAULT_LOGS_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--padding-s",
        type=float,
        default=2.0,
        help="Trajectory padding before and after the active obstacle window.",
    )
    parser.add_argument(
        "--latest-limit",
        type=int,
        default=DEFAULT_LATEST_LIMIT,
        help="Only scan the latest N run_* directories (default: 5).",
    )
    args = parser.parse_args()

    outputs = plot_colreg_size_trajectories(
        logs_dir=args.logs_dir,
        output_dir=args.output_dir,
        padding_s=float(args.padding_s),
        latest_limit=args.latest_limit,
        run_dirs=DEFAULT_RUN_DIRS,
    )

    for output_path, records in outputs:
        for record in sorted(records, key=lambda item: (item.size_label, item.size_metric_m)):
            print(
                "  "
                f"{record.size_label} {record.run_dir.name} "
                f"pc1={record.median_pc1_m:.3f}m "
                f"pc2={record.median_pc2_m:.3f}m "
                f"radius={record.median_equivalent_radius_m:.3f}m"
            )
        print(f"  Figure: {output_path}")


if __name__ == "__main__":
    main()
