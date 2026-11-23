#!/usr/bin/env python3
"""Run paired shelf trials in fresh Gazebo worlds and save failure bags."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

STRATEGIES = ("fixed", "multi", "precheck")


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def trial(directory, seed, width, depth, strategy, timeout, env):
    directory.mkdir()
    processes = []
    result = {"success": False, "stage": "startup", "grasped": False, "planning_s": None}
    with (directory / "launch.log").open("w") as launch_log, (
            directory / "record.log").open("w") as record_log, (
            directory / "execution.log").open("w") as execution:
        def start(args, output):
            proc = subprocess.Popen(args, env=env, start_new_session=True,
                                    stdout=output, stderr=subprocess.STDOUT)
            processes.append(proc)
            return proc
        try:
            launch = start(["ros2", "launch", "shelf_scene", "baseline.launch.py",
                            f"seed:={seed}", f"width:={width}", f"depth:={depth}"], launch_log)
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline and launch.poll() is None:
                log = (directory / "launch.log").read_text(errors="replace")
                if "Shelf geometry applied" in log and "Configured and activated" in log and (
                        "Configuring controller 'panda_right_finger_controller'" in log):
                    break
                time.sleep(.2)
            else:
                raise RuntimeError("baseline launch failed to become ready")
            record = start(["ros2", "bag", "record", "-o", str(directory / "replay"),
                            "/clock", "/joint_states", "/tf", "/tf_static", "/gazebo/model_states",
                            "/monitored_planning_scene", "/shelf/markers"], record_log)
            runner = start(["ros2", "run", "shelf_scene", "fixed_grasp",
                            "--ros-args", "-p", f"strategy:={strategy}"], execution)
            try:
                runner.wait(timeout)
            except subprocess.TimeoutExpired:
                result.update(stage="timeout", error="trial deadline exceeded")
            if record.poll() is not None:
                raise RuntimeError("rosbag recorder exited before trial ended")
        except RuntimeError as exc:
            result["error"] = str(exc)
        finally:
            for process in reversed(processes):
                stop(process)
    for line in (directory / "execution.log").read_text(errors="replace").splitlines():
        if line.startswith("TRIAL_RESULT "):
            result = json.loads(line.removeprefix("TRIAL_RESULT "))
    result.update(seed=seed, width=width, depth=depth, strategy=strategy)
    return result


def summarize(rows):
    groups = {}
    for row in rows:
        groups.setdefault((row["width"], row["depth"], row["strategy"]), []).append(row)
    stats = []
    for (width, depth, strategy), runs in groups.items():
        held = [r for r in runs if r["grasped"]]
        times = sorted(r["planning_s"] for r in runs if r["planning_s"] is not None)
        stats.append(dict(width=width, depth=depth, strategy=strategy, trials=len(runs),
                          success_rate=sum(r["success"] for r in runs) / len(runs),
                          verified_grasps=len(held),
                          withdraw_failure_rate=sum(r["stage"] == "withdraw" for r in held)
                          / len(held) if held else None,
                          planning_p95_s=times[math.ceil(.95*len(times))-1] if times else None,
                          timed_trials=len(times)))
    return stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seeds", nargs="+", type=int, default=[7, 8])
    parser.add_argument("--widths", nargs="+", type=float, default=[.30, .36])
    parser.add_argument("--depths", nargs="+", type=float, default=[.28, .32])
    parser.add_argument("--timeout", type=float, default=120.)
    parser.add_argument("--domain", type=int, default=95)
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    config = dict(vars(args), output=str(root), strategies=STRATEGIES)
    (root / "config.json").write_text(json.dumps(config, indent=2))
    rows = []
    with tempfile.TemporaryDirectory() as logs:
        for width in args.widths:
            for depth in args.depths:
                for seed in args.seeds:
                    for strategy in STRATEGIES:
                        with socket.socket() as sock:
                            sock.bind(("127.0.0.1", 0))
                            port = sock.getsockname()[1]
                        env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOG_DIR=logs,
                                   GAZEBO_MASTER_URI=f"http://localhost:{port}", GAZEBO_MODEL_DATABASE_URI="")
                        name = f"w{width}-d{depth}-s{seed}-{strategy}"
                        result = trial(root/name, seed, width, depth, strategy, args.timeout, env)
                        rows.append(result)
                        with (root/"trials.jsonl").open("a") as output:
                            print(json.dumps(result), file=output)
                        (root/"summary.json").write_text(json.dumps(summarize(rows), indent=2))
                        print(name, result["stage"], flush=True)


if __name__ == "__main__":
    main()
