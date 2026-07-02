"""Plot robot trajectories for all EKF/cluster switch combinations."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt
import numpy as np

from webots_collision import avoidance_succeeded


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOGS_DIR = PROJECT_ROOT / "logs"
DEFAULT_OUTPUT_DIR = DEFAULT_LOGS_DIR / "generated_figures"
TIME_COLUMNS = ("TimeFromStart(s)", "TimeFromStart", "elapsed [s]")
OWN_NORTH_COLUMNS = ("North(m)", "North", "x [m]")
OWN_EAST_COLUMNS = ("East(m)", "East", "y [m]")
OBSTACLE_NORTH_COLUMNS = ("NearestObstacleNorth(m)", "NearestObstacleNorth")
OBSTACLE_EAST_COLUMNS = ("NearestObstacleEast(m)", "NearestObstacleEast")

SWITCH_COMBINATIONS = {
    "ekf_on_cluster_on": (True, True),
    "ekf_on_cluster_off": (True, False),
    "ekf_off_cluster_on": (False, True),
    "ekf_off_cluster_off": (False, False),
}

SWITCH_LABELS = {
    "ekf_on_cluster_on": "EKF on, cluster APF on",
    "ekf_on_cluster_off": "EKF on, cluster APF off",
    "ekf_off_cluster_on": "EKF off, cluster APF on",
    "ekf_off_cluster_off": "EKF off, cluster APF off",
}

SWITCH_STYLES = {
    "ekf_on_cluster_on": {"color": "#1f77b4", "linestyle": "-"},
    "ekf_on_cluster_off": {"color": "#ff7f0e", "linestyle": "--"},
    "ekf_off_cluster_on": {"color": "#2ca02c", "linestyle": "-."},
    "ekf_off_cluster_off": {"color": "#d62728", "linestyle": ":"},
}


@dataclass
class RunRecord:
    run_dir: Path
    log_path: Path
    run_time: datetime
    switch_combination: str
    colreg_rule: str
    webots_environment: str
    avoidance_succeeded: bool | None
    time_s: np.ndarray
    own_position_ne_m: np.ndarray
    obstacle_position_ne_m: np.ndarray

    @property
    def initial_own_position_ne_m(self) -> np.ndarray:
        return self.own_position_ne_m[0]


@dataclass
class EnvironmentMatch:
    median_error_m: float
    percentile_90_error_m: float
    overlap_duration_s: float


def parse_float(value: object) -> float:
    if value is None:
        return np.nan
    text = str(value).strip()
    if not text or text.lower() in {"none", "nan", "null"}:
        return np.nan
    try:
        return float(text)
    except (TypeError, ValueError):
        return np.nan


def first_existing(row: dict[str, str], names: tuple[str, ...]) -> object:
    for name in names:
        if name in row:
            return row[name]
    return None


def find_csv_log(run_dir: Path) -> Path:
    candidates = [
        path
        for path in run_dir.glob("log_*.csv")
        if not path.name.endswith("_pseudo_aruco.csv")
    ]
    if not candidates:
        raise FileNotFoundError("No primary log_*.csv file")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def parse_run_time(run_dir: Path) -> datetime:
    try:
        return datetime.strptime(run_dir.name, "run_%Y%m%d_%H%M%S")
    except ValueError:
        return datetime.fromtimestamp(run_dir.stat().st_mtime)


def switch_combination_from_flags(ekf_enabled: bool, cluster_enabled: bool) -> str:
    for name, flags in SWITCH_COMBINATIONS.items():
        if flags == (bool(ekf_enabled), bool(cluster_enabled)):
            return name
    raise ValueError(
        f"Unknown switch flags: ekf={ekf_enabled}, cluster={cluster_enabled}"
    )


def normalise_colreg_rule(value: object) -> str | None:
    if value is None:
        return None
    text = str(value).strip().lower().replace("-", "_").replace(" ", "_")
    if not text or text in {"0", "none", "nan", "null", "unknown"}:
        return None
    return "".join(char for char in text if char.isalnum() or char == "_")


def normalise_name_token(value: object, fallback: str) -> str:
    if value is None:
        return fallback
    text = str(value).strip()
    if not text:
        return fallback
    if text.lower().endswith(".wbt"):
        text = Path(text).stem
    text = text.lower().replace("-", "_").replace(" ", "_")
    token = "".join(char for char in text if char.isalnum() or char == "_")
    return token or fallback


def most_common_colreg_rule(values: list[str]) -> str:
    if not values:
        return "unknown_colreg"
    return Counter(values).most_common(1)[0][0]


def read_csv_colreg_rule(log_path: Path) -> str | None:
    with log_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or "APFEncounter" not in reader.fieldnames:
            return None
        rules = []
        for row in reader:
            rule = normalise_colreg_rule(row.get("APFEncounter"))
            if rule is not None:
                rules.append(rule)
    return most_common_colreg_rule(rules) if rules else None


def read_json_colreg_rule(run_dir: Path) -> str | None:
    rules = []
    active_profiles = []
    for snapshot_path in sorted(run_dir.glob("obstacle_*.json")):
        with snapshot_path.open(encoding="utf-8") as stream:
            payload = json.load(stream)
        apf = payload.get("apf", {})
        if not isinstance(apf, dict):
            continue
        for key in ("colreg_rule", "encounter"):
            rule = normalise_colreg_rule(apf.get(key))
            if rule is not None:
                rules.append(rule)
        profile = normalise_colreg_rule(apf.get("active_profile"))
        if profile is not None and profile != "default_apf":
            active_profiles.append(profile)
    return (
        most_common_colreg_rule(rules)
        if rules
        else most_common_colreg_rule(active_profiles)
        if active_profiles
        else None
    )


def read_colreg_rule(run_dir: Path, log_path: Path) -> str:
    return (
        read_csv_colreg_rule(log_path)
        or read_json_colreg_rule(run_dir)
        or "unknown_colreg"
    )


def read_csv_webots_environment(log_path: Path) -> str | None:
    with log_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or "WebotsEnvironment" not in reader.fieldnames:
            return None
        names = []
        for row in reader:
            name = normalise_name_token(row.get("WebotsEnvironment"), "")
            if name and name not in {"webots_unknown", "not_webots"}:
                names.append(name)
    return Counter(names).most_common(1)[0][0] if names else None


def read_json_webots_environment(run_dir: Path) -> str | None:
    names = []
    for snapshot_path in sorted(run_dir.glob("obstacle_*.json")):
        with snapshot_path.open(encoding="utf-8") as stream:
            payload = json.load(stream)
        for key in ("webots_environment", "WebotsEnvironment", "world", "world_file"):
            name = normalise_name_token(payload.get(key), "")
            if name and name not in {"webots_unknown", "not_webots"}:
                names.append(name)
    return Counter(names).most_common(1)[0][0] if names else None


def read_webots_environment(run_dir: Path, log_path: Path) -> str:
    return (
        read_csv_webots_environment(log_path)
        or read_json_webots_environment(run_dir)
        or "webots_unknown"
    )


def read_avoidance_outcome(run_dir: Path) -> bool | None:
    return avoidance_succeeded(run_dir)


def parse_bool(value: object) -> bool | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on", "enabled"}:
        return True
    if text in {"0", "false", "no", "off", "disabled"}:
        return False
    return None


def read_csv_switch_combination(log_path: Path) -> str | None:
    with log_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            return None

        has_switch_name = "SwitchCombination" in reader.fieldnames
        has_switch_flags = (
            "EKFPredictionEnabled" in reader.fieldnames
            and "ClusterSizeAPFEnabled" in reader.fieldnames
        )
        if not has_switch_name and not has_switch_flags:
            return None

        for row in reader:
            if has_switch_name:
                switch_name = str(row.get("SwitchCombination", "")).strip()
                if switch_name in SWITCH_COMBINATIONS:
                    return switch_name
            if has_switch_flags:
                ekf_enabled = parse_bool(row.get("EKFPredictionEnabled"))
                cluster_enabled = parse_bool(row.get("ClusterSizeAPFEnabled"))
                if ekf_enabled is not None and cluster_enabled is not None:
                    return switch_combination_from_flags(
                        ekf_enabled,
                        cluster_enabled,
                    )
    return None


def read_switch_combination(run_dir: Path, log_path: Path) -> str:
    csv_switch = read_csv_switch_combination(log_path)
    if csv_switch is not None:
        return csv_switch

    snapshot_paths = sorted(run_dir.glob("obstacle_*.json"))
    if not snapshot_paths:
        raise ValueError("No obstacle_*.json snapshots")

    sample_indices = sorted({0, len(snapshot_paths) // 2, len(snapshot_paths) - 1})
    flags = []
    for index in sample_indices:
        with snapshot_paths[index].open(encoding="utf-8") as stream:
            payload = json.load(stream)
        settings = payload.get("apf_settings", {})
        if not isinstance(settings, dict):
            continue
        if (
            "obstacle_ekf_prediction_enabled" not in settings
            or "cluster_range_enabled" not in settings
        ):
            continue
        flags.append(
            (
                bool(settings["obstacle_ekf_prediction_enabled"]),
                bool(settings["cluster_range_enabled"]),
            )
        )

    if not flags:
        raise ValueError("No EKF/cluster switch metadata")
    if any(flag != flags[0] for flag in flags[1:]):
        raise ValueError("Switch metadata changes within the run")
    return switch_combination_from_flags(*flags[0])


def read_trajectory_series(
    log_path: Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    times = []
    own_positions = []
    obstacle_positions = []
    first_log_time_s = np.nan

    with log_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError("CSV file has no header")

        required_groups = (
            TIME_COLUMNS,
            OWN_NORTH_COLUMNS,
            OWN_EAST_COLUMNS,
        )
        missing = [
            names[0]
            for names in required_groups
            if not any(name in reader.fieldnames for name in names)
        ]
        if missing:
            raise ValueError(
                "Missing columns required for trajectory plotting: "
                + ", ".join(missing)
            )

        for row in reader:
            time_s = parse_float(first_existing(row, TIME_COLUMNS))
            own_north = parse_float(first_existing(row, OWN_NORTH_COLUMNS))
            own_east = parse_float(first_existing(row, OWN_EAST_COLUMNS))
            navigation_mode = str(row.get("NavigationMode", "")).strip().lower()
            if np.isfinite(time_s) and not np.isfinite(first_log_time_s):
                first_log_time_s = time_s
            if not all(np.isfinite(value) for value in (time_s, own_north, own_east)):
                continue

            obstacle_north = parse_float(first_existing(row, OBSTACLE_NORTH_COLUMNS))
            obstacle_east = parse_float(first_existing(row, OBSTACLE_EAST_COLUMNS))
            times.append(float(time_s))
            own_positions.append([float(own_north), float(own_east)])
            if np.isfinite(obstacle_north) and np.isfinite(obstacle_east):
                obstacle_positions.append([float(obstacle_north), float(obstacle_east)])
            else:
                obstacle_positions.append([np.nan, np.nan])
            if navigation_mode == "arrived":
                break

    if not times:
        raise ValueError("No valid robot trajectory samples")

    time_array = np.asarray(times, dtype=float)
    own_array = np.asarray(own_positions, dtype=float)
    obstacle_array = np.asarray(obstacle_positions, dtype=float)
    order = np.argsort(time_array)
    time_array = time_array[order]
    own_array = own_array[order]
    obstacle_array = obstacle_array[order]
    time_array -= (
        float(first_log_time_s)
        if np.isfinite(first_log_time_s)
        else float(time_array[0])
    )
    return time_array, own_array, obstacle_array


def load_run(run_dir: Path, forced_switch_combination: str | None = None) -> RunRecord:
    log_path = find_csv_log(run_dir)
    switch_combination = (
        forced_switch_combination
        if forced_switch_combination is not None
        else read_switch_combination(run_dir, log_path)
    )
    colreg_rule = read_colreg_rule(run_dir, log_path)
    webots_environment = read_webots_environment(run_dir, log_path)
    avoidance_succeeded = read_avoidance_outcome(run_dir)
    time_s, own_position_ne_m, obstacle_position_ne_m = read_trajectory_series(log_path)
    return RunRecord(
        run_dir=run_dir.resolve(),
        log_path=log_path.resolve(),
        run_time=parse_run_time(run_dir),
        switch_combination=switch_combination,
        colreg_rule=colreg_rule,
        webots_environment=webots_environment,
        avoidance_succeeded=avoidance_succeeded,
        time_s=time_s,
        own_position_ne_m=own_position_ne_m,
        obstacle_position_ne_m=obstacle_position_ne_m,
    )


def scan_runs(logs_dir: Path) -> tuple[list[RunRecord], list[tuple[Path, str]]]:
    records = []
    skipped = []
    run_dirs = sorted(
        (path for path in logs_dir.glob("run_*") if path.is_dir()),
        key=parse_run_time,
        reverse=True,
    )
    for run_dir in run_dirs:
        try:
            records.append(load_run(run_dir))
        except (FileNotFoundError, ValueError, json.JSONDecodeError) as exc:
            skipped.append((run_dir, str(exc)))
    return records, skipped


def compare_environments(
    left: RunRecord,
    right: RunRecord,
    minimum_overlap_s: float,
) -> EnvironmentMatch | None:
    if (
        left.webots_environment == "webots_unknown"
        or right.webots_environment == "webots_unknown"
        or left.webots_environment != right.webots_environment
    ):
        return None

    if np.linalg.norm(left.initial_own_position_ne_m - right.initial_own_position_ne_m) > 0.5:
        return None

    left_valid = np.isfinite(left.obstacle_position_ne_m).all(axis=1)
    right_valid = np.isfinite(right.obstacle_position_ne_m).all(axis=1)
    if np.count_nonzero(left_valid) < 2 or np.count_nonzero(right_valid) < 2:
        return None

    left_time = left.time_s[left_valid]
    right_time = right.time_s[right_valid]
    overlap_start_s = max(float(left_time[0]), float(right_time[0]))
    overlap_end_s = min(float(left_time[-1]), float(right_time[-1]))
    overlap_duration_s = overlap_end_s - overlap_start_s
    if overlap_duration_s < minimum_overlap_s:
        return None

    sample_count = max(50, min(300, len(left_time), len(right_time)))
    common_time_s = np.linspace(overlap_start_s, overlap_end_s, sample_count)
    left_positions = np.column_stack(
        [
            np.interp(
                common_time_s,
                left_time,
                left.obstacle_position_ne_m[left_valid, axis],
            )
            for axis in range(2)
        ]
    )
    right_positions = np.column_stack(
        [
            np.interp(
                common_time_s,
                right_time,
                right.obstacle_position_ne_m[right_valid, axis],
            )
            for axis in range(2)
        ]
    )
    errors_m = np.linalg.norm(left_positions - right_positions, axis=1)
    return EnvironmentMatch(
        median_error_m=float(np.median(errors_m)),
        percentile_90_error_m=float(np.percentile(errors_m, 90.0)),
        overlap_duration_s=float(overlap_duration_s),
    )


def select_latest_compatible_group(
    records: list[RunRecord],
    minimum_overlap_s: float,
    median_tolerance_m: float,
    percentile_90_tolerance_m: float,
) -> dict[str, RunRecord]:
    missing = [
        switch
        for switch in SWITCH_COMBINATIONS
        if not any(record.switch_combination == switch for record in records)
    ]
    if missing:
        raise ValueError("Missing switch combinations: " + ", ".join(missing))

    sorted_records = sorted(records, key=lambda record: record.run_time, reverse=True)
    for reference in sorted_records:
        group = {reference.switch_combination: reference}
        for switch in SWITCH_COMBINATIONS:
            if switch in group:
                continue
            compatible = []
            for candidate in sorted_records:
                if candidate.switch_combination != switch:
                    continue
                match = compare_environments(
                    reference,
                    candidate,
                    minimum_overlap_s=minimum_overlap_s,
                )
                if match is None:
                    continue
                if match.median_error_m > median_tolerance_m:
                    continue
                if match.percentile_90_error_m > percentile_90_tolerance_m:
                    continue
                compatible.append(candidate)
            if compatible:
                group[switch] = compatible[0]
        if set(group) == set(SWITCH_COMBINATIONS):
            return group

    raise ValueError(
        "No compatible four-switch group was found. Run the same Webots "
        "world once for each switch combination, keeping the obstacle setup "
        "unchanged."
    )


def select_latest_group(records: list[RunRecord]) -> dict[str, RunRecord]:
    group = {}
    for record in sorted(records, key=lambda item: item.run_time, reverse=True):
        group.setdefault(record.switch_combination, record)
    missing = [switch for switch in SWITCH_COMBINATIONS if switch not in group]
    if missing:
        raise ValueError("Missing switch combinations: " + ", ".join(missing))
    return group


def load_explicit_group(args: argparse.Namespace) -> dict[str, RunRecord] | None:
    explicit_paths = {
        "ekf_on_cluster_on": args.ekf_on_cluster_on,
        "ekf_on_cluster_off": args.ekf_on_cluster_off,
        "ekf_off_cluster_on": args.ekf_off_cluster_on,
        "ekf_off_cluster_off": args.ekf_off_cluster_off,
    }
    provided = {switch: path for switch, path in explicit_paths.items() if path}
    if not provided:
        return None
    missing = [switch for switch in SWITCH_COMBINATIONS if switch not in provided]
    if missing:
        raise ValueError(
            "Explicit run selection requires all four switch paths. Missing: "
            + ", ".join(missing)
        )
    return {
        switch: load_run(Path(path), forced_switch_combination=switch)
        for switch, path in provided.items()
    }


def set_equal_axis_with_padding(ax) -> None:
    x_min, x_max = ax.get_xlim()
    y_min, y_max = ax.get_ylim()
    x_mid = 0.5 * (x_min + x_max)
    y_mid = 0.5 * (y_min + y_max)
    span = max(x_max - x_min, y_max - y_min, 1.0)
    pad = 0.08 * span
    half = 0.5 * span + pad
    ax.set_xlim(x_mid - half, x_mid + half)
    ax.set_ylim(y_mid - half, y_mid + half)
    ax.set_aspect("equal", adjustable="box")


def plot_switch_trajectories(
    group: dict[str, RunRecord],
    output_path: Path,
    webots_environment: str,
) -> None:
    fig, ax = plt.subplots(figsize=(8.5, 8.0), dpi=180)

    for switch in SWITCH_COMBINATIONS:
        record = group[switch]
        positions = record.own_position_ne_m
        style = SWITCH_STYLES[switch]
        ax.plot(
            positions[:, 1],
            positions[:, 0],
            linewidth=2.1,
            label=f"{SWITCH_LABELS[switch]} ({avoidance_outcome_text(record)})",
            **style,
        )
        ax.scatter(
            positions[0, 1],
            positions[0, 0],
            s=26,
            marker="o",
            color=style["color"],
            edgecolors="white",
            linewidths=0.6,
            zorder=3,
        )
        ax.scatter(
            positions[-1, 1],
            positions[-1, 0],
            s=34,
            marker="s",
            color=style["color"],
            edgecolors="white",
            linewidths=0.6,
            zorder=3,
        )

    title_name = webots_environment.replace("_", " ").title()
    ax.set_title(
        "Robot Trajectory Comparison: "
        f"{title_name}\nAvoidance result: {group_avoidance_outcome_text(group)} "
        "(Webots contact sensor)"
    )
    ax.set_xlabel("East position (m)")
    ax.set_ylabel("North position (m)")
    ax.grid(True, linestyle=":", alpha=0.35)
    ax.legend(loc="best", framealpha=0.95)
    set_equal_axis_with_padding(ax)
    fig.tight_layout()

    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Scan Webots logs and plot robot trajectories for all four "
            "EKF/cluster switch combinations."
        )
    )
    parser.add_argument(
        "--logs-dir",
        type=Path,
        default=DEFAULT_LOGS_DIR,
        help="Directory containing run_* log directories.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help=(
            "Output PNG path. Defaults to "
            "<webots_name>_four_switch_robot_trajectories.png."
        ),
    )
    parser.add_argument(
        "--minimum-overlap",
        type=float,
        default=10.0,
        help="Minimum nearest-obstacle trajectory overlap in seconds.",
    )
    parser.add_argument(
        "--environment-median-tolerance",
        type=float,
        default=0.75,
        help="Maximum median nearest-obstacle trajectory difference in metres.",
    )
    parser.add_argument(
        "--environment-p90-tolerance",
        type=float,
        default=1.5,
        help="Maximum 90th-percentile nearest-obstacle trajectory difference in metres.",
    )
    parser.add_argument(
        "--allow-different-environments",
        action="store_true",
        help="Use the latest run for each switch even when obstacle trajectories differ.",
    )
    parser.add_argument(
        "--ekf-on-cluster-on",
        type=Path,
        help="Explicit run directory for EKF on and cluster APF on.",
    )
    parser.add_argument(
        "--ekf-on-cluster-off",
        type=Path,
        help="Explicit run directory for EKF on and cluster APF off.",
    )
    parser.add_argument(
        "--ekf-off-cluster-on",
        type=Path,
        help="Explicit run directory for EKF off and cluster APF on.",
    )
    parser.add_argument(
        "--ekf-off-cluster-off",
        type=Path,
        help="Explicit run directory for EKF off and cluster APF off.",
    )
    return parser


def group_colreg_rule(group: dict[str, RunRecord]) -> str:
    rules = [
        record.colreg_rule
        for record in group.values()
        if record.colreg_rule != "unknown_colreg"
    ]
    return most_common_colreg_rule(rules)


def avoidance_outcome_text(record: RunRecord) -> str:
    if record.avoidance_succeeded is None:
        return "avoidance unknown"
    return "avoidance succeeded" if record.avoidance_succeeded else "avoidance failed"


def group_avoidance_outcome_text(group: dict[str, RunRecord]) -> str:
    outcomes = [record.avoidance_succeeded for record in group.values()]
    known = [outcome for outcome in outcomes if outcome is not None]
    unknown = len(outcomes) - len(known)
    if not known:
        return "unknown"
    if all(known) and unknown == 0:
        return "all selected runs succeeded"
    failed = sum(not outcome for outcome in known)
    succeeded = sum(bool(outcome) for outcome in known)
    suffix = f", {unknown} unknown" if unknown else ""
    return f"{succeeded} succeeded, {failed} failed{suffix}"


def group_webots_environment(group: dict[str, RunRecord]) -> str:
    names = [
        record.webots_environment
        for record in group.values()
        if record.webots_environment != "webots_unknown"
    ]
    return Counter(names).most_common(1)[0][0] if names else "webots_unknown"


def default_output_path(webots_environment: str) -> Path:
    return DEFAULT_OUTPUT_DIR / f"{webots_environment}_four_switch_robot_trajectories.png"


def main() -> None:
    args = build_parser().parse_args()

    try:
        explicit_group = load_explicit_group(args)
        if explicit_group is not None:
            group = explicit_group
            skipped = []
        else:
            records, skipped = scan_runs(args.logs_dir)
            if not records:
                raise SystemExit("No classified runs were found.")
            if args.allow_different_environments:
                group = select_latest_group(records)
            else:
                group = select_latest_compatible_group(
                    records=records,
                    minimum_overlap_s=args.minimum_overlap,
                    median_tolerance_m=args.environment_median_tolerance,
                    percentile_90_tolerance_m=args.environment_p90_tolerance,
                )
    except ValueError as exc:
        raise SystemExit(f"Selection stopped: {exc}") from None

    print("Selected switch-combination runs:")
    for switch in SWITCH_COMBINATIONS:
        record = group[switch]
        print(
            f"  {SWITCH_LABELS[switch]}: {record.run_dir.name}, "
            f"{avoidance_outcome_text(record)}"
        )
    print(f"Skipped {len(skipped)} unclassifiable runs.")

    webots_environment = group_webots_environment(group)
    output_path = (
        args.output if args.output is not None else default_output_path(webots_environment)
    )
    plot_switch_trajectories(group, output_path, webots_environment)
    print(f"Webots environment: {webots_environment}")
    print(f"COLREG rule: {group_colreg_rule(group)}")
    print(f"Figure saved to: {output_path.resolve()}")


if __name__ == "__main__":
    main()
