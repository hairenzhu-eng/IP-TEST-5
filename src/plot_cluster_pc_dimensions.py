"""Plot DBSCAN cluster PC1 and PC2 dimensions against time."""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOGS_DIR = PROJECT_ROOT / "logs"
DEFAULT_OUTPUT_DIR = DEFAULT_LOGS_DIR / "generated_figures"


def positive_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value) and value > 0.0 else None


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


def latest_run_dir(logs_dir):
    candidates = [
        path
        for path in Path(logs_dir).glob("run_*")
        if path.is_dir() and any(path.glob("obstacle_*.json"))
    ]
    if not candidates:
        raise FileNotFoundError(
            f"No run directory with obstacle JSON logs in {logs_dir}"
        )
    return max(candidates, key=lambda path: path.stat().st_mtime)


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
        help="Run directory containing obstacle_*.json; defaults to latest run.",
    )
    parser.add_argument("--logs-dir", type=Path, default=DEFAULT_LOGS_DIR)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    run_dir = args.run_dir or latest_run_dir(args.logs_dir)
    output_path, pc1_mean_m, pc2_mean_m, sample_count = (
        plot_cluster_dimensions(run_dir, args.output)
    )
    print(f"Run: {run_dir}")
    print(f"Samples: {sample_count}")
    print(f"PC1 mean: {pc1_mean_m:.6f} m")
    print(f"PC2 mean: {pc2_mean_m:.6f} m")
    print(f"Figure: {output_path}")


if __name__ == "__main__":
    main()
