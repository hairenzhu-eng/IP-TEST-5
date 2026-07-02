"""Run every Webots world with all four EKF/cluster switch combinations."""

import argparse
import csv
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
WORLDS_DIR = ROOT / "webots" / "worlds"
LOGS_DIR = ROOT / "logs"
WEBOTS_EXE = Path(
    os.environ.get(
        "WEBOTS_EXE",
        r"F:\webots2025a\Webots\msys64\mingw64\bin\webots.exe",
    )
)
COMBINATIONS = (
    "ekf_on_cluster_on",
    "ekf_on_cluster_off",
    "ekf_off_cluster_on",
    "ekf_off_cluster_off",
)
FLAGS = {
    "ekf_on_cluster_on": ("1", "1"),
    "ekf_on_cluster_off": ("1", "0"),
    "ekf_off_cluster_on": ("0", "1"),
    "ekf_off_cluster_off": ("0", "0"),
}


class RobotTimeout(TimeoutError):
    pass


def save_manifest(path, manifest):
    path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")


def terminate_tree(process, grace_s=10):
    if process is None or process.poll() is not None:
        return
    subprocess.run(
        ["taskkill", "/PID", str(process.pid), "/T"],
        capture_output=True,
        text=True,
    )
    try:
        process.wait(timeout=grace_s)
        return
    except subprocess.TimeoutExpired:
        subprocess.run(
            ["taskkill", "/PID", str(process.pid), "/T", "/F"],
            capture_output=True,
            text=True,
        )
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass


def wait_for_run_dir(existing, show_process, timeout_s=60):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        candidates = [
            path
            for path in LOGS_DIR.glob("run_*")
            if path.is_dir() and path not in existing
        ]
        if candidates:
            run_dir = max(candidates, key=lambda path: path.stat().st_mtime)
            csv_paths = sorted(run_dir.glob("log_*.csv"))
            if csv_paths:
                return run_dir, csv_paths[0]
        if show_process.poll() is not None:
            raise RuntimeError(
                f"show_laptop exited before creating a run log "
                f"(exit code {show_process.returncode})"
            )
        time.sleep(1)
    raise TimeoutError("show_laptop did not create a run log within 60 seconds")


def read_rows(csv_path):
    try:
        with csv_path.open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))
    except (OSError, csv.Error, UnicodeDecodeError):
        return []


def wait_until_stopped(csv_path, show_process, webots_process, timeout_s):
    deadline = time.monotonic() + timeout_s
    seen_rows = 0
    moved = False
    zero_streak = 0
    while time.monotonic() < deadline:
        rows = read_rows(csv_path)
        if len(rows) < seen_rows:
            seen_rows = 0
            zero_streak = 0
        for row in rows[seen_rows:]:
            try:
                right = float(row["right_prop_rate(rad/s)"])
                left = float(row["left_prop_rate(rad/s)"])
            except (KeyError, TypeError, ValueError):
                continue
            if abs(right) > 1.0 or abs(left) > 1.0:
                moved = True
                zero_streak = 0
            elif moved and abs(right) < 0.01 and abs(left) < 0.01:
                zero_streak += 1
            else:
                zero_streak = 0
        seen_rows = len(rows)
        if moved and zero_streak >= 20:
            return rows
        if show_process.poll() is not None:
            raise RuntimeError(
                f"show_laptop exited before the robot stopped "
                f"(exit code {show_process.returncode})"
            )
        if webots_process.poll() is not None:
            raise RuntimeError(
                f"Webots exited before the robot stopped "
                f"(exit code {webots_process.returncode})"
            )
        time.sleep(1)
    raise RobotTimeout(f"robot did not stop within {timeout_s} seconds")


def verify_rows(rows, world, combination):
    expected_ekf, expected_cluster = FLAGS[combination]
    metadata_rows = [
        row
        for row in rows
        if row.get("WebotsEnvironment") and row.get("SwitchCombination")
    ]
    if not metadata_rows:
        raise RuntimeError("run log has no Webots/switch metadata")
    row = metadata_rows[-1]
    actual = (
        row.get("WebotsEnvironment"),
        row.get("SwitchCombination"),
        row.get("EKFPredictionEnabled"),
        row.get("ClusterSizeAPFEnabled"),
    )
    expected = (world.name, combination, expected_ekf, expected_cluster)
    if actual != expected:
        raise RuntimeError(f"metadata mismatch: expected {expected}, got {actual}")


def run_once(world, combination, output_dir, timeout_s):
    existing_runs = set(LOGS_DIR.glob("run_*"))
    label = f"{world.stem}__{combination}"
    webots_output = (output_dir / f"{label}__webots.txt").open(
        "w", encoding="utf-8"
    )
    laptop_output = (output_dir / f"{label}__show_laptop.txt").open(
        "w", encoding="utf-8"
    )
    webots_process = None
    show_process = None
    try:
        webots_process = subprocess.Popen(
            [
                str(WEBOTS_EXE),
                "--batch",
                "--minimize",
                "--mode=realtime",
                "--stdout",
                "--stderr",
                str(world),
            ],
            cwd=ROOT,
            stdout=webots_output,
            stderr=subprocess.STDOUT,
        )
        time.sleep(5)
        if webots_process.poll() is not None:
            raise RuntimeError(
                f"Webots exited during startup (exit code {webots_process.returncode})"
            )

        env = os.environ.copy()
        env["SWITCH_COMBINATION"] = combination
        env["WEBOTS_WORLD"] = str(world)
        show_process = subprocess.Popen(
            [sys.executable, str(ROOT / "src" / "show_laptop.py"), "--simulation"],
            cwd=ROOT,
            env=env,
            stdout=laptop_output,
            stderr=subprocess.STDOUT,
        )
        run_dir, csv_path = wait_for_run_dir(existing_runs, show_process)
        rows = wait_until_stopped(
            csv_path, show_process, webots_process, timeout_s
        )
        verify_rows(rows, world, combination)
        return run_dir
    finally:
        terminate_tree(show_process)
        terminate_tree(webots_process)
        laptop_output.close()
        webots_output.close()
        time.sleep(3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=int, default=500)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--skip-failed", action="store_true")
    parser.add_argument("--world-pattern", default="*.wbt")
    args = parser.parse_args()

    if args.skip_failed and not args.resume:
        parser.error("--skip-failed requires --resume")
    if not WEBOTS_EXE.is_file():
        parser.error(f"Webots executable not found: {WEBOTS_EXE}")

    if args.resume:
        manifest_path = args.resume.resolve()
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        output_dir = manifest_path.parent
        world_names = manifest.get("worlds")
        worlds = (
            [WORLDS_DIR / name for name in world_names]
            if world_names
            else sorted(WORLDS_DIR.glob(args.world_pattern))
        )
        if args.skip_failed:
            failure = manifest.pop("failure", None)
            if not failure:
                parser.error("manifest has no failed item to skip")
            manifest.setdefault("skipped", []).append(
                {
                    "world": failure["world"],
                    "combination": failure["combination"],
                    "reason": failure["error"],
                }
            )
        manifest["status"] = "running"
        manifest.pop("completed", None)
    else:
        worlds = sorted(WORLDS_DIR.glob(args.world_pattern))
        started = datetime.now().astimezone()
        output_dir = LOGS_DIR / f"matrix_{started:%Y%m%d_%H%M%S}"
        output_dir.mkdir(parents=True)
        manifest_path = args.manifest or output_dir / "manifest.json"
        manifest = {
            "started": started.isoformat(),
            "status": "running",
            "expected_runs": len(worlds) * len(COMBINATIONS),
            "worlds": [world.name for world in worlds],
            "runs": [],
        }
    if not worlds or any(not world.is_file() for world in worlds):
        parser.error(f"no valid worlds matched: {args.world_pattern}")
    total_runs = len(worlds) * len(COMBINATIONS)
    save_manifest(manifest_path, manifest)

    completed = {
        (run["world"], run["combination"]) for run in manifest["runs"]
    }
    skipped = {
        (run["world"], run["combination"])
        for run in manifest.get("skipped", [])
    }
    for world in worlds:
        for combination in COMBINATIONS:
            if (world.name, combination) in completed | skipped:
                continue
            index = len(completed) + len(skipped) + 1
            print(
                f"[{index}/{total_runs}] START {world.name} {combination}",
                flush=True,
            )
            run_dir = None
            for attempt in (1, 2):
                try:
                    run_dir = run_once(
                        world, combination, output_dir, args.timeout
                    )
                    break
                except RobotTimeout as exc:
                    manifest.setdefault("skipped", []).append(
                        {
                            "world": world.name,
                            "combination": combination,
                            "reason": str(exc),
                        }
                    )
                    skipped.add((world.name, combination))
                    save_manifest(manifest_path, manifest)
                    print(f"[{index}/{total_runs}] SKIPPED {exc}", flush=True)
                    break
                except Exception as exc:
                    print(
                        f"[{index}/{total_runs}] attempt {attempt} failed: {exc}",
                        flush=True,
                    )
                    if attempt == 2:
                        manifest["status"] = "failed"
                        manifest["failure"] = {
                            "world": world.name,
                            "combination": combination,
                            "error": str(exc),
                        }
                        save_manifest(manifest_path, manifest)
                        raise
            if run_dir is None:
                continue
            manifest["runs"].append(
                {
                    "world": world.name,
                    "combination": combination,
                    "run_dir": str(run_dir.relative_to(ROOT)),
                }
            )
            completed.add((world.name, combination))
            save_manifest(manifest_path, manifest)
            print(
                f"[{index}/{total_runs}] DONE {run_dir.relative_to(ROOT)}",
                flush=True,
            )

    manifest["status"] = "complete"
    manifest["completed"] = datetime.now().astimezone().isoformat()
    save_manifest(manifest_path, manifest)
    print(f"COMPLETE: {manifest_path}", flush=True)


if __name__ == "__main__":
    main()
