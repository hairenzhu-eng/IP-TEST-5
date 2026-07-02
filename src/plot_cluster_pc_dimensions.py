"""Plot DBSCAN cluster PC1 and PC2 dimensions against time."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt
import numpy as np

from plot_colreg_size_trajectories import (
    collect_grouped_runs,
    collect_snapshot_samples,
    merged_output_name,
    pair_large_small_records,
)
from webots_collision import collision_detected, collision_outcome_text


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOGS_DIR = PROJECT_ROOT / "logs"
DEFAULT_OUTPUT_DIR = DEFAULT_LOGS_DIR / "generated_figures"
DEFAULT_DISTANCE_OUTPUT_DIR = DEFAULT_OUTPUT_DIR / "webots_distance_groups"
DEFAULT_SIZE_COMPARISON_OUTPUT_DIR = DEFAULT_OUTPUT_DIR / "pc_size_comparisons"

TIME_COLUMNS = ("TimeFromStart(s)", "TimeFromStart", "elapsed [s]")
OWN_NORTH_COLUMNS = ("North(m)", "North", "x [m]")
OWN_EAST_COLUMNS = ("East(m)", "East", "y [m]")
OBSTACLE_NORTH_COLUMNS = ("NearestObstacleNorth(m)", "NearestObstacleNorth")
OBSTACLE_EAST_COLUMNS = ("NearestObstacleEast(m)", "NearestObstacleEast")
OBSTACLE_DISTANCE_COLUMNS = ("NearestObstacleDistance(m)", "NearestObstacleDistance")

SWITCH_LABELS = {
    "ekf_on_cluster_on": "EKF on, size on",
    "ekf_on_cluster_off": "EKF on, size off",
    "ekf_off_cluster_on": "EKF off, size on",
    "ekf_off_cluster_off": "EKF off, size off",
}

SWITCH_STYLES = {
    "ekf_on_cluster_on": {"color": "#1f77b4", "linestyle": "-"},
    "ekf_on_cluster_off": {"color": "#ff7f0e", "linestyle": "--"},
    "ekf_off_cluster_on": {"color": "#2ca02c", "linestyle": "-."},
    "ekf_off_cluster_off": {"color": "#d62728", "linestyle": ":"},
}


@dataclass
class DistanceRun:
    run_dir: Path
    webots_environment: str
    switch_combination: str
    time_s: np.ndarray
    distance_m: np.ndarray
    collision_detected: bool | None

    @property
    def minimum_distance_m(self):
        return float(np.nanmin(self.distance_m))


def positive_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value) and value > 0.0 else None


def parse_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return np.nan
    return value if np.isfinite(value) else np.nan


def parse_bool(value):
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on", "enabled"}:
        return True
    if text in {"0", "false", "no", "off", "disabled"}:
        return False
    return None


def first_existing(row, names):
    for name in names:
        if name in row:
            return row[name]
    return None


def normalise_webots_name(value):
    text = str(value or "").strip()
    if text.lower().endswith(".wbt"):
        text = Path(text).stem
    text = text.lower().replace("-", "_").replace(" ", "_")
    token = "".join(char for char in text if char.isalnum() or char == "_")
    return token or "webots_unknown"


def switch_from_flags(ekf_enabled, size_enabled):
    if ekf_enabled is None or size_enabled is None:
        return None
    return (
        f"ekf_{'on' if ekf_enabled else 'off'}_"
        f"cluster_{'on' if size_enabled else 'off'}"
    )


def find_primary_csv(run_dir):
    candidates = [
        path
        for path in Path(run_dir).glob("log_*.csv")
        if not path.name.endswith("_pseudo_aruco.csv")
    ]
    return max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None


def read_log_metadata(log_path):
    with log_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError("CSV file has no header")
        for row in reader:
            webots_environment = normalise_webots_name(row.get("WebotsEnvironment"))
            switch_combination = str(row.get("SwitchCombination", "")).strip()
            if switch_combination not in SWITCH_LABELS:
                switch_combination = switch_from_flags(
                    parse_bool(row.get("EKFPredictionEnabled")),
                    parse_bool(row.get("ClusterSizeAPFEnabled")),
                )
            if (
                webots_environment not in {"webots_unknown", "not_webots"}
                and switch_combination in SWITCH_LABELS
            ):
                return webots_environment, switch_combination
    raise ValueError("No Webots/EKF/size metadata in CSV")


def read_distance_series(log_path):
    times = []
    distances = []
    first_time_s = np.nan

    with log_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError("CSV file has no header")

        for row in reader:
            time_s = parse_float(first_existing(row, TIME_COLUMNS))
            if np.isfinite(time_s) and not np.isfinite(first_time_s):
                first_time_s = time_s
            distance_m = parse_float(first_existing(row, OBSTACLE_DISTANCE_COLUMNS))

            if not np.isfinite(distance_m):
                own_north = parse_float(first_existing(row, OWN_NORTH_COLUMNS))
                own_east = parse_float(first_existing(row, OWN_EAST_COLUMNS))
                obstacle_north = parse_float(first_existing(row, OBSTACLE_NORTH_COLUMNS))
                obstacle_east = parse_float(first_existing(row, OBSTACLE_EAST_COLUMNS))
                if all(
                    np.isfinite(value)
                    for value in (own_north, own_east, obstacle_north, obstacle_east)
                ):
                    distance_m = float(
                        np.hypot(own_north - obstacle_north, own_east - obstacle_east)
                    )

            if np.isfinite(time_s) and np.isfinite(distance_m):
                times.append(float(time_s))
                distances.append(float(distance_m))

    if not times:
        raise ValueError("No valid nearest-obstacle distance samples")

    time_array = np.asarray(times, dtype=float)
    distance_array = np.asarray(distances, dtype=float)
    order = np.argsort(time_array)
    time_array = time_array[order]
    distance_array = distance_array[order]
    time_array -= float(first_time_s) if np.isfinite(first_time_s) else time_array[0]
    return time_array, distance_array, float(first_time_s)


def load_cluster_dimensions(run_dir):
    """Return cluster PC dimensions grouped by track ID."""
    samples = []
    for path in sorted(Path(run_dir).glob("obstacle_*.json")):
        try:
            with path.open(encoding="utf-8") as stream:
                payload = json.load(stream)
        except (OSError, json.JSONDecodeError):
            continue

        try:
            time_s = float(payload.get("t"))
        except (TypeError, ValueError):
            continue
        if not np.isfinite(time_s):
            continue

        clusters = payload.get("clusters", [])
        if not isinstance(clusters, list):
            continue
        for cluster in clusters:
            if not isinstance(cluster, dict):
                continue
            pc1_m = positive_float(cluster.get("pc1_m"))
            pc2_m = positive_float(cluster.get("pc2_m"))
            if pc1_m is None or pc2_m is None:
                continue
            track_id = cluster.get("track_id")
            if track_id is None:
                track_id = f"cluster-{cluster.get('label', 'unknown')}"
            samples.append(
                {
                    "time_s": time_s,
                    "track_id": str(track_id),
                    "pc1_m": pc1_m,
                    "pc2_m": pc2_m,
                }
            )

    if not samples:
        raise ValueError(
            f"No valid cluster pc1_m/pc2_m values found in {run_dir}"
        )

    samples.sort(key=lambda sample: sample["time_s"])
    first_time_s = samples[0]["time_s"]
    grouped = defaultdict(list)
    for sample in samples:
        sample["time_s"] -= first_time_s
        grouped[sample["track_id"]].append(sample)
    return dict(grouped)


def plot_cluster_dimensions(run_dir, output_path=None):
    """Create one figure containing PC1, PC2, and their horizontal means."""
    run_dir = Path(run_dir)
    grouped = load_cluster_dimensions(run_dir)
    all_samples = [
        sample
        for track_samples in grouped.values()
        for sample in track_samples
    ]
    pc1_mean_m = float(np.mean([sample["pc1_m"] for sample in all_samples]))
    pc2_mean_m = float(np.mean([sample["pc2_m"] for sample in all_samples]))

    if output_path is None:
        output_path = (
            DEFAULT_OUTPUT_DIR / f"{run_dir.name}_pc1_pc2_over_time.png"
        )
    output_path = Path(output_path)

    fig, ax = plt.subplots(figsize=(10, 5.5))
    for track_index, track_id in enumerate(
        sorted(grouped, key=lambda value: (not value.isdigit(), value))
    ):
        samples = grouped[track_id]
        time_s = [sample["time_s"] for sample in samples]
        pc1_m = [sample["pc1_m"] for sample in samples]
        pc2_m = [sample["pc2_m"] for sample in samples]
        ax.plot(
            time_s,
            pc1_m,
            color="#0072B2",
            linewidth=1.4,
            marker="o",
            markersize=2.5,
            alpha=0.85,
            label="PC1" if track_index == 0 else "_nolegend_",
        )
        ax.plot(
            time_s,
            pc2_m,
            color="#D55E00",
            linewidth=1.4,
            marker="o",
            markersize=2.5,
            alpha=0.85,
            label="PC2" if track_index == 0 else "_nolegend_",
        )

    ax.axhline(
        pc1_mean_m,
        color="#0072B2",
        linestyle="--",
        linewidth=2.0,
        label=f"PC1 mean = {pc1_mean_m:.3f} m",
    )
    ax.axhline(
        pc2_mean_m,
        color="#D55E00",
        linestyle="--",
        linewidth=2.0,
        label=f"PC2 mean = {pc2_mean_m:.3f} m",
    )

    ax.set_title(f"Cluster principal dimensions over time\n{run_dir.name}")
    ax.set_xlabel("Time from first cluster sample (s)")
    ax.set_ylabel("Size (m)")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300)
    plt.close(fig)
    return output_path, pc1_mean_m, pc2_mean_m, len(all_samples)


def plot_large_small_pc_pair(records, output_path):
    """Plot PC1 and PC2 time series for one matched large/small Webots pair."""
    colors = {"Small": "#0072B2", "Large": "#D55E00"}
    scenario = (
        records[0].webots_pair_key.removeprefix("mr_webots_")
        .replace("_size_ship", "")
        .replace("_", " ")
        .title()
    )
    fig, ax = plt.subplots(figsize=(10, 5.8))

    for record in sorted(records, key=lambda item: item.size_label, reverse=True):
        samples = collect_snapshot_samples(record.run_dir)
        if not samples:
            continue
        first_time_s = samples[0].time_s
        time_s = [sample.time_s - first_time_s for sample in samples]
        color = colors[record.size_label]
        ax.plot(
            time_s,
            [sample.obstacle_pc1_m for sample in samples],
            color=color,
            linewidth=1.8,
            label=f"{record.size_label} ship PC1",
        )
        ax.plot(
            time_s,
            [sample.obstacle_pc2_m for sample in samples],
            color=color,
            linestyle="--",
            linewidth=1.8,
            label=f"{record.size_label} ship PC2",
        )

    ax.set_title(
        "Principal Component Size over Time\n"
        f"{scenario}: Large vs Small"
    )
    ax.set_xlabel("Time from first obstacle sample (s)")
    ax.set_ylabel("Size (m)")
    ax.set_xlim(left=0.0)
    ax.set_ylim(bottom=0.0)
    ax.grid(True, linestyle=":", alpha=0.35)
    ax.legend()
    fig.tight_layout()

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300)
    plt.close(fig)
    return output_path


def plot_large_small_pc_comparisons(logs_dir, output_dir, run_dirs=None):
    """Plot the latest EKF-on/size-on large-small pair for each Webots scene."""
    records = collect_grouped_runs(logs_dir, run_dirs=run_dirs)
    pairs = pair_large_small_records(records)
    if not pairs:
        raise ValueError(
            "No matched large/small Webots runs with EKF and size both enabled"
        )

    outputs = []
    for pair in pairs:
        output_path = Path(output_dir) / f"{merged_output_name(pair)}_pc1_pc2.png"
        outputs.append((plot_large_small_pc_pair(pair, output_path), pair))
    return outputs


def load_distance_run(run_dir):
    log_path = find_primary_csv(run_dir)
    if log_path is None:
        raise FileNotFoundError("No primary log_*.csv file")
    webots_environment, switch_combination = read_log_metadata(log_path)
    time_s, distance_m, _ = read_distance_series(log_path)
    return DistanceRun(
        run_dir=Path(run_dir),
        webots_environment=webots_environment,
        switch_combination=switch_combination,
        time_s=time_s,
        distance_m=distance_m,
        collision_detected=collision_detected(run_dir),
    )


def scan_distance_runs(logs_dir):
    records = []
    skipped = []
    for run_dir in sorted(Path(logs_dir).glob("run_*"), key=lambda path: path.name, reverse=True):
        if not run_dir.is_dir():
            continue
        try:
            records.append(load_distance_run(run_dir))
        except (FileNotFoundError, ValueError) as exc:
            skipped.append((run_dir, str(exc)))
    return records, skipped


def latest_by_webots_and_switch(records):
    grouped = defaultdict(dict)
    for record in sorted(records, key=lambda item: item.run_dir.name, reverse=True):
        grouped[record.webots_environment].setdefault(record.switch_combination, record)
    return {
        webots_environment: group
        for webots_environment, group in grouped.items()
        if len(group) >= 2
    }


def safe_output_name(name):
    return "".join(char if char.isalnum() or char in "._-" else "_" for char in name)


def plot_webots_distance_group(webots_environment, group, output_dir):
    output_path = Path(output_dir) / f"{safe_output_name(webots_environment)}_distance.png"
    fig, ax = plt.subplots(figsize=(10, 5.8))

    for switch in SWITCH_LABELS:
        record = group.get(switch)
        if record is None:
            continue
        style = SWITCH_STYLES[switch]
        label = (
            f"{SWITCH_LABELS[switch]} {record.run_dir.name} "
            f"(min distance {record.minimum_distance_m:.2f} m; "
            f"{collision_outcome_text(record.run_dir)})"
        )
        ax.plot(
            record.time_s,
            record.distance_m,
            linewidth=2.0,
            label=label,
            **style,
        )
    ax.set_title(
        "Distance to nearest obstacle\n"
        f"{webots_environment.replace('_', ' ').title()} "
        "(collision from Webots ShipObstacle contact sensor)"
    )
    ax.set_xlabel("Time from log start (s)")
    ax.set_ylabel("Distance (m)")
    ax.set_xlim(left=0.0)
    ax.set_ylim(bottom=0.0)
    ax.grid(True, linestyle=":", alpha=0.35)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300)
    plt.close(fig)
    return output_path


def plot_webots_distance_groups(logs_dir, output_dir):
    records, skipped = scan_distance_runs(logs_dir)
    grouped = latest_by_webots_and_switch(records)
    if not grouped:
        raise ValueError("No Webots groups with at least two EKF/size states found")

    outputs = []
    for webots_environment, group in sorted(grouped.items()):
        outputs.append((plot_webots_distance_group(webots_environment, group, output_dir), group))
    return outputs, skipped


def plot_webots_distance_group_for_run(run_dir, logs_dir, output_dir):
    selected = load_distance_run(Path(run_dir))
    records, skipped = scan_distance_runs(logs_dir)
    group = {}
    for record in sorted(records + [selected], key=lambda item: item.run_dir.name, reverse=True):
        if record.webots_environment == selected.webots_environment:
            group.setdefault(record.switch_combination, record)
    if not group:
        raise ValueError(f"No runs found for Webots environment {selected.webots_environment}")
    output_path = plot_webots_distance_group(
        selected.webots_environment,
        group,
        output_dir,
    )
    return output_path, group, skipped


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Plot clustered PC1 and PC2 sizes against time, including "
            "a horizontal mean line for each dimension."
        )
    )
    parser.add_argument(
        "--run-dir",
        type=Path,
        help="Run directory used to select one Webots distance group.",
    )
    parser.add_argument("--logs-dir", type=Path, default=DEFAULT_LOGS_DIR)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--pc-dimensions",
        action="store_true",
        help="Plot old PC1/PC2 dimensions for --run-dir instead of distances.",
    )
    parser.add_argument(
        "--distance-output-dir",
        type=Path,
        default=DEFAULT_DISTANCE_OUTPUT_DIR,
        help="Output directory for grouped Webots distance figures.",
    )
    parser.add_argument(
        "--size-comparison",
        action="store_true",
        help="Compare PC1/PC2 for matched large/small EKF-on and size-on runs.",
    )
    parser.add_argument(
        "--size-comparison-output-dir",
        type=Path,
        default=DEFAULT_SIZE_COMPARISON_OUTPUT_DIR,
        help="Output directory for large/small PC1/PC2 figures.",
    )
    args = parser.parse_args()

    if args.size_comparison:
        run_dirs = [args.run_dir] if args.run_dir else None
        outputs = plot_large_small_pc_comparisons(
            args.logs_dir,
            args.size_comparison_output_dir,
            run_dirs=run_dirs,
        )
        for output_path, pair in outputs:
            print(f"Figure: {output_path}")
            for record in pair:
                print(f"  {record.size_label}: {record.run_dir.name}")
        return

    if args.pc_dimensions:
        if not args.run_dir:
            raise SystemExit("--pc-dimensions requires --run-dir")
        output_path, pc1_mean_m, pc2_mean_m, sample_count = (
            plot_cluster_dimensions(args.run_dir, args.output)
        )
        print(f"Run: {args.run_dir}")
        print(f"Samples: {sample_count}")
        print(f"PC1 mean: {pc1_mean_m:.6f} m")
        print(f"PC2 mean: {pc2_mean_m:.6f} m")
        print(f"Figure: {output_path}")
        return

    if args.run_dir:
        output_path, group, skipped = plot_webots_distance_group_for_run(
            args.run_dir,
            args.logs_dir,
            args.distance_output_dir,
        )
        print(f"Figure: {output_path}")
        for switch in SWITCH_LABELS:
            record = group.get(switch)
            if record is not None:
                print(
                    f"  {SWITCH_LABELS[switch]}: {record.run_dir.name}, "
                    f"min distance={record.minimum_distance_m:.3f} m, "
                    f"{collision_outcome_text(record.run_dir)}"
                )
        print(f"Skipped {len(skipped)} unclassifiable runs.")
        return

    outputs, skipped = plot_webots_distance_groups(
        args.logs_dir,
        args.distance_output_dir,
    )
    for output_path, group in outputs:
        print(f"Figure: {output_path}")
        for switch in SWITCH_LABELS:
            record = group.get(switch)
            if record is not None:
                print(
                    f"  {SWITCH_LABELS[switch]}: {record.run_dir.name}, "
                    f"min distance={record.minimum_distance_m:.3f} m, "
                    f"{collision_outcome_text(record.run_dir)}"
                )
    print(f"Skipped {len(skipped)} unclassifiable runs.")


if __name__ == "__main__":
    main()
