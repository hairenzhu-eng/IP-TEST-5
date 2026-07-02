from __future__ import annotations

from collections import defaultdict
import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def main() -> None:
    switch = load_module(
        "switch_plot", ROOT / "src" / "plot_switch_combination_trajectories.py"
    )
    size = load_module(
        "size_plot", ROOT / "src" / "plot_colreg_size_trajectories.py"
    )

    records, skipped = switch.scan_runs(ROOT / "logs")
    grouped = defaultdict(list)
    for record in records:
        if record.webots_environment.startswith("mr_webots_"):
            grouped[record.webots_environment].append(record)

    print("SWITCH RESULTS")
    selected_records = []
    for environment in sorted(grouped):
        latest = switch.select_latest_group(grouped[environment])
        print(environment)
        for combination in switch.SWITCH_COMBINATIONS:
            record = latest[combination]
            selected_records.append(record)
            status = "success" if record.avoidance_succeeded else "failed"
            print(
                f"  {combination}: {record.run_dir.name}; {status}; "
                f"clearance={record.minimum_collision_clearance_m:.6f}; "
                f"rule={record.colreg_rule}"
            )

    print("\nAGGREGATE")
    for combination in switch.SWITCH_COMBINATIONS:
        subset = [r for r in selected_records if r.switch_combination == combination]
        clearances = [r.minimum_collision_clearance_m for r in subset]
        successes = sum(bool(r.avoidance_succeeded) for r in subset)
        print(
            f"  {combination}: {successes}/{len(subset)}; "
            f"mean={sum(clearances)/len(clearances):.6f}; "
            f"min={min(clearances):.6f}; max={max(clearances):.6f}"
        )

    print("\nSIZE RESULTS")
    figure_runs = [
        "run_20260625_163234", "run_20260625_185755",
        "run_20260625_160520", "run_20260625_162046",
        "run_20260625_191930", "run_20260625_191229",
        "run_20260625_193234", "run_20260625_192948",
    ]
    for run_name in figure_runs:
        record = size.build_run_record(ROOT / "logs" / run_name)
        print(
            f"  {record.webots_environment}: {record.run_dir.name}; "
            f"size={record.size_label}; pc1_median={record.median_pc1_m:.6f}; "
            f"switch={record.switch_combination}"
        )

    print(f"\nSkipped switch runs: {len(skipped)}")


if __name__ == "__main__":
    main()
