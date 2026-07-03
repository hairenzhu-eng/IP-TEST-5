"""Plot trajectory and distance comparisons for different obstacle speeds."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
import re

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOGS_DIR = PROJECT_ROOT / "logs"
DEFAULT_OUTPUT_DIR = DEFAULT_LOGS_DIR / "generated_figures" / "speed_overtaking"

TIME_COLUMNS = ("TimeFromStart(s)", "TimeFromStart", "elapsed [s]")
OWN_NORTH_COLUMNS = ("North(m)", "North", "x [m]")
OWN_EAST_COLUMNS = ("East(m)", "East", "y [m]")
ARUCO_NORTH_COLUMNS = ("ARUCOSensedNorth(m)", "ARUCOSensedNorth")
ARUCO_EAST_COLUMNS = ("ARUCOSensedEast(m)", "ARUCOSensedEast")
OBSTACLE_NORTH_COLUMNS = ("NearestObstacleNorth(m)", "NearestObstacleNorth")
OBSTACLE_EAST_COLUMNS = ("NearestObstacleEast(m)", "NearestObstacleEast")
SPEED_WORLD_PATTERN = re.compile(
    r"^(?P<group>.+)_(?P<whole>\d+)_(?P<fraction>\d+)_m_s$"
)


@dataclass
class RunRecord:
    run_dir: Path
    world_name: str
    group_name: str
    obstacle_speed_m_s: float
    time_s: np.ndarray
    own_position_ne_m: np.ndarray
    obstacle_position_ne_m: np.ndarray
    distance_m: np.ndarray


def parse_float(value: object) -> float:
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return np.nan


def parse_bool(value: object) -> bool | None:
    if isinstance(value, bool):
        return value
    text = str(value or "").strip().lower()
    if text in {"1", "true", "yes", "on", "enabled"}:
        return True
    if text in {"0", "false", "no", "off", "disabled"}:
        return False
    return None


def first_existing(row: dict[str, str], names: tuple[str, ...]) -> object:
    for name in names:
        if name in row:
            return row[name]
    return None


def parse_speed_world(value: object) -> tuple[str, str, float] | None:
    world_name = Path(str(value or "").strip()).stem.lower()
    match = SPEED_WORLD_PATTERN.fullmatch(world_name)
    if match is None:
        return None
    speed = float(f"{match.group('whole')}.{match.group('fraction')}")
    return world_name, match.group("group"), speed


def is_ekf_cluster_on(row: dict[str, str]) -> bool:
    switch_name = str(row.get("SwitchCombination", "")).strip().lower()
    if switch_name:
        return switch_name == "ekf_on_cluster_on"
    return (
        parse_bool(row.get("EKFPredictionEnabled")) is True
        and parse_bool(row.get("ClusterSizeAPFEnabled")) is True
    )


def find_primary_csv(run_dir: Path) -> Path:
    candidates = [
        path
        for path in run_dir.glob("log_*.csv")
        if not path.name.endswith("_pseudo_aruco.csv")
    ]
    if not candidates:
        raise FileNotFoundError(f"No primary log CSV in {run_dir}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def load_run(run_dir: Path, use_aruco: bool = False) -> RunRecord | None:
    log_path = find_primary_csv(run_dir)
    with log_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        first_row = next(reader, None)
        if first_row is None or not is_ekf_cluster_on(first_row):
            return None

        speed_world = parse_speed_world(first_row.get("WebotsEnvironment"))
        if speed_world is None:
            return None
        world_name, group_name, obstacle_speed_m_s = speed_world

        times = []
        own_positions = []
        obstacle_positions = []
        distances = []
        own_north_columns = ARUCO_NORTH_COLUMNS if use_aruco else OWN_NORTH_COLUMNS
        own_east_columns = ARUCO_EAST_COLUMNS if use_aruco else OWN_EAST_COLUMNS
        for row in (first_row, *reader):
            time_s = parse_float(first_existing(row, TIME_COLUMNS))
            own_north = parse_float(first_existing(row, own_north_columns))
            own_east = parse_float(first_existing(row, own_east_columns))
            obstacle_north = parse_float(
                first_existing(row, OBSTACLE_NORTH_COLUMNS)
            )
            obstacle_east = parse_float(first_existing(row, OBSTACLE_EAST_COLUMNS))
            values = (
                time_s,
                own_north,
                own_east,
                obstacle_north,
                obstacle_east,
            )
            if not all(np.isfinite(value) for value in values):
                continue

            times.append(time_s)
            own_positions.append([own_north, own_east])
            obstacle_positions.append([obstacle_north, obstacle_east])
            distances.append(
                float(np.hypot(own_north - obstacle_north, own_east - obstacle_east))
            )
            if str(row.get("NavigationMode", "")).strip().lower() == "arrived":
                break

    if len(times) < 2:
        raise ValueError(f"Not enough valid position samples in {log_path}")

    time_array = np.asarray(times, dtype=float)
    order = np.argsort(time_array)
    return RunRecord(
        run_dir=run_dir.resolve(),
        world_name=world_name,
        group_name=group_name,
        obstacle_speed_m_s=obstacle_speed_m_s,
        time_s=time_array[order] - float(np.min(time_array)),
        own_position_ne_m=np.asarray(own_positions, dtype=float)[order],
        obstacle_position_ne_m=np.asarray(obstacle_positions, dtype=float)[order],
        distance_m=np.asarray(distances, dtype=float)[order],
    )


def selected_run_dirs(logs_dir: Path, run_dirs: list[Path] | None) -> list[Path]:
    if not run_dirs:
        return sorted(
            (path for path in logs_dir.glob("run_*") if path.is_dir()),
            key=lambda path: path.name,
            reverse=True,
        )

    selected = []
    for path in run_dirs:
        candidate = path if path.is_absolute() else logs_dir / path
        if not candidate.is_dir():
            raise FileNotFoundError(f"Run directory not found: {path}")
        selected.append(candidate)
    return selected


def collect_runs(
    logs_dir: Path,
    run_dirs: list[Path] | None = None,
    use_aruco: bool = False,
) -> dict[str, list[RunRecord]]:
    latest_by_group_speed: dict[tuple[str, float], RunRecord] = {}
    for run_dir in selected_run_dirs(logs_dir, run_dirs):
        try:
            record = load_run(run_dir, use_aruco=use_aruco)
        except (FileNotFoundError, OSError, ValueError):
            continue
        if record is None:
            continue
        latest_by_group_speed.setdefault(
            (record.group_name, record.obstacle_speed_m_s),
            record,
        )

    grouped: dict[str, list[RunRecord]] = {}
    for record in latest_by_group_speed.values():
        grouped.setdefault(record.group_name, []).append(record)
    return {
        group_name: sorted(
            records,
            key=lambda record: record.obstacle_speed_m_s,
        )
        for group_name, records in grouped.items()
        if len(records) >= 2
    }


def plot_group(
    records: list[RunRecord],
    output_path: Path,
    use_aruco: bool = False,
    trajectory_only: bool = False,
    include_obstacle: bool = True,
) -> None:
    colors = plt.get_cmap("viridis")(
        np.linspace(0.08, 0.88, len(records))
    )
    if trajectory_only:
        figure, trajectory_ax = plt.subplots(
            1,
            1,
            figsize=(8.0, 6.2),
            dpi=180,
        )
        distance_ax = None
    else:
        figure, (trajectory_ax, distance_ax) = plt.subplots(
            1,
            2,
            figsize=(14.0, 6.2),
            dpi=180,
        )

    speed_handles = []
    for record, color in zip(records, colors):
        speed_label = f"{record.obstacle_speed_m_s:g} m/s"
        (own_line,) = trajectory_ax.plot(
            record.own_position_ne_m[:, 1],
            record.own_position_ne_m[:, 0],
            color=color,
            linewidth=2.2,
            label=speed_label,
        )
        if include_obstacle:
            trajectory_ax.plot(
                record.obstacle_position_ne_m[:, 1],
                record.obstacle_position_ne_m[:, 0],
                color=color,
                linestyle="--",
                linewidth=1.5,
                alpha=0.8,
            )
        speed_handles.append(own_line)

        if distance_ax is not None:
            minimum_distance_m = float(np.min(record.distance_m))
            distance_ax.plot(
                record.time_s,
                record.distance_m,
                color=color,
                linewidth=2.0,
                label=f"{speed_label} (min {minimum_distance_m:.2f} m)",
            )

    own_ship_source = "ArUco position" if use_aruco else "own ship state"
    trajectory_ax.set_title(
        "Trajectories "
        f"(solid: {own_ship_source}; dashed: obstacle)"
    )
    trajectory_ax.set_xlabel("East position (m)")
    trajectory_ax.set_ylabel("North position (m)")
    trajectory_ax.grid(True, linestyle=":", alpha=0.35)
    trajectory_ax.set_aspect("equal", adjustable="datalim")
    speed_legend = trajectory_ax.legend(
        handles=speed_handles,
        title="Obstacle speed",
        loc="best",
    )
    trajectory_ax.add_artist(speed_legend)
    legend_handles = [
        Line2D([0], [0], color="black", linewidth=2.2, label="Own ship"),
    ]
    if include_obstacle:
        legend_handles.append(
            Line2D(
                [0],
                [0],
                color="black",
                linestyle="--",
                linewidth=1.5,
                label="Obstacle ship",
            )
        )
    trajectory_ax.legend(handles=legend_handles, loc="lower right")

    if distance_ax is not None:
        distance_ax.set_title("Distance Between Own Ship and Obstacle")
        distance_ax.set_xlabel("Motion time (s)")
        distance_ax.set_ylabel("Distance (m)")
        distance_ax.set_xlim(left=0.0)
        distance_ax.set_ylim(bottom=0.0)
        distance_ax.grid(True, linestyle=":", alpha=0.35)
        distance_ax.legend(loc="best", title="Obstacle speed")

    display_name = records[0].group_name.replace("_", " ").title()
    source_name = "ArUco trajectory" if use_aruco else "trajectory"
    figure.suptitle(
        f"{display_name}: Obstacle-Speed Comparison ({source_name})\n"
        "EKF prediction on, cluster-size APF on"
    )
    figure.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def plot_obstacle_speed_comparisons(
    logs_dir: Path,
    output_dir: Path,
    run_dirs: list[Path] | None = None,
    use_aruco: bool = False,
    trajectory_only: bool = False,
    include_obstacle: bool = True,
) -> list[tuple[Path, list[RunRecord]]]:
    groups = collect_runs(logs_dir, run_dirs, use_aruco=use_aruco)
    if not groups:
        raise ValueError(
            "No environment has at least two obstacle speeds with "
            "EKF and cluster both on"
        )

    outputs = []
    for group_name, records in sorted(groups.items()):
        suffix = (
            "speed_aruco_trajectory"
            if use_aruco and trajectory_only
            else "speed_distance_aruco_trajectory"
            if use_aruco
            else "speed_trajectory"
            if trajectory_only
            else "speed_distance_trajectory"
        )
        if not include_obstacle:
            suffix += "_own_only"
        output_path = output_dir / f"{group_name}_{suffix}.png"
        plot_group(
            records,
            output_path,
            use_aruco=use_aruco,
            trajectory_only=trajectory_only,
            include_obstacle=include_obstacle,
        )
        outputs.append((output_path, records))
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Plot obstacle-speed distance and trajectory comparisons using only "
            "runs where EKF and cluster-size APF are both on."
        )
    )
    parser.add_argument("--logs-dir", type=Path, default=DEFAULT_LOGS_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--run-dir",
        action="append",
        type=Path,
        dest="run_dirs",
        help="Explicit run_* directory to include; repeat for multiple runs.",
    )
    parser.add_argument(
        "--use-aruco",
        action="store_true",
        help="Plot the own-ship trajectory from ARUCOSensedNorth/East columns.",
    )
    parser.add_argument(
        "--trajectory-only",
        action="store_true",
        help="Plot only trajectories and skip the distance subplot.",
    )
    parser.add_argument(
        "--no-obstacle",
        action="store_true",
        help="Do not draw obstacle-ship trajectories.",
    )
    args = parser.parse_args()

    outputs = plot_obstacle_speed_comparisons(
        args.logs_dir,
        args.output_dir,
        args.run_dirs,
        use_aruco=args.use_aruco,
        trajectory_only=args.trajectory_only,
        include_obstacle=not args.no_obstacle,
    )
    for output_path, records in outputs:
        speeds = ", ".join(
            f"{record.obstacle_speed_m_s:g} m/s" for record in records
        )
        print(f"{records[0].group_name}: {speeds}")
        print(f"  Figure: {output_path.resolve()}")


if __name__ == "__main__":
    main()
