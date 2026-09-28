#!/usr/bin/env python3
"""Run independent Gazebo duels and pair both robots' measured results."""

import argparse
import hashlib
import json
import os
import subprocess
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from scenarios import SCENARIOS, scenario_environment


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"


def revision():
    commit = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"],
                                     cwd=ROOT, text=True).strip()
    status = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=all"],
                                     cwd=ROOT)
    if not status:
        return commit
    digest = hashlib.sha256(subprocess.check_output(
        ["git", "diff", "--binary", "HEAD"], cwd=ROOT))
    for line in sorted(status.splitlines()):
        if line.startswith(b"?? "):
            path = ROOT / os.fsdecode(line[3:])
            digest.update(line)
            if path.is_file():
                digest.update(path.read_bytes())
    return f"{commit}+dirty.{digest.hexdigest()[:12]}"


def command(args, env, timeout=None, log=None):
    result = subprocess.run(args, cwd=ROOT, env=env, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            timeout=timeout, check=False)
    if log is not None:
        Path(log).write_text(result.stdout)
    if result.returncode:
        raise RuntimeError(f"{' '.join(args)} exited {result.returncode}: "
                           f"{result.stdout[-1200:]}")
    return result.stdout


def read_report(path, run_id, stopped=False):
    try:
        report = json.loads(path.read_text())
        return report if report.get("run_id") == run_id and (not stopped or
               report.get("active") is False) else None
    except (OSError, ValueError):
        return None


def ready_topic(container, topic, env):
    shell = ("source /autoware/install/setup.bash && timeout 7 "
             f"ros2 topic echo {topic} --no-daemon --once --field pose.pose.position")
    try:
        result = subprocess.run(["docker", "exec", container, "bash", "-lc", shell],
                                cwd=ROOT, env=env, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, timeout=12, check=False)
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def wait_ready(env, deadline):
    topics = (("docker-hsl-decision-1", "/navigation/self"),
              ("docker-hsl-opponent-decision-1", "/opponent/navigation/self"))
    started = time.monotonic()
    while time.monotonic() < deadline:
        if time.monotonic() - started > 15:
            gazebo = subprocess.run(["docker", "exec", "docker-gazebo-duel-1",
                                     "pgrep", "-x", "gzserver"], cwd=ROOT,
                                    env=env, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL, check=False)
            if gazebo.returncode:
                raise TimeoutError("gzserver exited before both pose streams were ready")
        if all(ready_topic(container, topic, env) for container, topic in topics):
            time.sleep(2)
            return
        time.sleep(2)
    raise TimeoutError("both navigation pose streams did not become ready")


def wait_report(path, run_id, deadline, stopped=False):
    while time.monotonic() < deadline:
        report = read_report(path, run_id, stopped)
        if report is not None:
            return report
        time.sleep(2)
    raise TimeoutError(f"result {path} for {run_id} did not appear")


def start_traces(series_dir, index, env):
    command(["docker", "cp", str(ROOT / "benchmarks" / "trace_motion.py"),
             "docker-hsl-adapter-1:/tmp/hsl_trace_motion.py"], env, timeout=15)
    traces = []
    for role, namespace in (("first", ""), ("second", " --namespace opponent")):
        destination = series_dir / f"{index:02d}-trace-{role}.json"
        output = destination.open("w")
        script = ("source /autoware/install/setup.bash && python3 "
                  "/tmp/hsl_trace_motion.py --wall-seconds 1200" + namespace)
        process = subprocess.Popen(
            ["docker", "exec", "docker-hsl-adapter-1", "bash", "-lc", script],
            cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT)
        traces.append((process, output, destination))
    time.sleep(2)
    return traces


def finish_traces(traces):
    for process, output, _ in traces:
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        finally:
            output.close()


def summary(runs):
    complete = [run for run in runs if "outcome" in run]
    counts = Counter(run["outcome"]["event"] for run in complete)
    result = {"runs_completed": len(complete), "events": dict(counts),
              "runs_failed": len(runs) - len(complete),
              "explorer_goals": sum(run["outcome"]["explorer_reached_goal"]
                                    for run in complete),
              "guardian_captures": sum(run["outcome"]["guardian_captured"]
                                       for run in complete)}
    for role in ("explorer", "guardian"):
        reports = [report for run in complete for report in run["robots"]
                   if report["role"] == role]
        result[role] = {
            "mean_speed_mps": round(sum(r["mean_speed_mps"] for r in reports)
                                      / len(reports), 3) if reports else None,
            "runs_below_0_3_mps": sum(r["mean_speed_mps"] < 0.3
                                        for r in reports),
            "runs_below_0_2_mps": sum(r["mean_speed_mps"] < 0.2
                                        for r in reports),
            "mean_planner_ok_fraction": round(sum(r["planner_ok_fraction"]
                                                  for r in reports) / len(reports), 3)
                                        if reports else None,
            "mean_moving_fraction": round(sum(r["moving_fraction"] for r in reports)
                                          / len(reports), 3) if reports else None,
            "mean_turning_fraction": round(sum(r["turning_fraction"]
                                               for r in reports) / len(reports), 3)
                                     if reports and all("turning_fraction" in r
                                                        for r in reports) else None,
            "mean_active_motion_fraction": round(
                sum(r["active_motion_fraction"] for r in reports) / len(reports), 3)
                if reports and all("active_motion_fraction" in r for r in reports)
                else None,
            "mean_collisions": round(sum(r["collisions"] for r in reports)
                                     / len(reports), 3) if reports else None,
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=20)
    parser.add_argument("--start-seed", type=int, default=0)
    parser.add_argument("--active-s", type=float, default=360.0)
    parser.add_argument("--wall-timeout-s", type=float, default=1200.0)
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--trace", action="store_true",
                        help="record active cmd/path diagnostics for each robot")
    parser.add_argument("--scenario", type=int, choices=SCENARIOS, default=1)
    args = parser.parse_args()
    if args.runs < 1 or args.active_s <= 0:
        parser.error("runs and active-s must be positive")
    series_id = datetime.now(timezone.utc).strftime("series-%Y%m%dT%H%M%SZ")
    series_dir = RESULTS / series_id
    series_dir.mkdir(parents=True, exist_ok=True)
    code_revision = revision()
    runs = []
    if args.build:
        command(["helm", "build", "duel"], os.environ.copy(),
                timeout=1800, log=series_dir / "build.log")
    for index in range(args.runs):
        seed = args.start_seed + index
        first_role = "explorer" if index % 2 == 0 else "guardian"
        second_role = "guardian" if first_role == "explorer" else "explorer"
        run_id = f"{series_id}-{index:02d}"
        env = os.environ.copy()
        env.update({"GAZEBO_HEADLESS": "true", "HSL_ROLE": first_role,
                    "HSL_RVIZ_ENABLED": "false",
                    "HSL_OPPONENT_ROLE": second_role,
                    "DUEL_RUN_ID": run_id, "DUEL_SEED": str(seed),
                    "DUEL_OPPONENT_SEED": str(seed + 1000003),
                    "DUEL_REVISION": code_revision,
                    "DUEL_MAX_ACTIVE_S": str(args.active_s)})
        env.update(scenario_environment(args.scenario))
        record = {"run_id": run_id, "seed": seed,
                  "roles": [first_role, second_role], "scenario": args.scenario}
        print(f"[{index + 1}/{args.runs}] {run_id} "
              f"{first_role}/{second_role} seed={seed}", flush=True)
        traces = []
        try:
            for attempt in range(3):
                suffix = f"{index:02d}" if attempt == 0 else f"{index:02d}-retry{attempt}"
                command(["helm", "clean", "duel"], env, timeout=120,
                        log=series_dir / f"{suffix}-clean.log")
                command(["helm", "up", "duel"], env, timeout=120,
                        log=series_dir / f"{suffix}-up.log")
                try:
                    wait_ready(env, time.monotonic() + 180)
                    break
                except TimeoutError:
                    if attempt == 2:
                        raise
            if args.trace:
                traces = start_traces(series_dir, index, env)
            command(["helm", "start_match"], env, timeout=45,
                    log=series_dir / f"{index:02d}-start.log")
            outcome = wait_report(RESULTS / "latest_outcome.json", run_id,
                                  time.monotonic() + args.wall_timeout_s)
            first = wait_report(RESULTS / "latest.json", run_id,
                                time.monotonic() + 45, stopped=True)
            second = wait_report(RESULTS / "opponent/latest.json", run_id,
                                 time.monotonic() + 45, stopped=True)
            record.update(outcome=outcome, robots=[first, second])
            if traces:
                finish_traces(traces)
                traces = []
            print(f"  {outcome['event']} at {outcome['duration_s']} sim s; "
                  f"speeds {first['mean_speed_mps']}/{second['mean_speed_mps']} m/s",
                  flush=True)
        except (RuntimeError, TimeoutError, subprocess.TimeoutExpired) as error:
            record["error"] = str(error)
            print(f"  ERROR: {error}", flush=True)
            try:
                command(["helm", "stop_match"], env, timeout=30,
                        log=series_dir / f"{index:02d}-stop-after-error.log")
            except (RuntimeError, subprocess.TimeoutExpired):
                pass
            runs.append(record)
            (series_dir / "index.json").write_text(json.dumps(
                runs, indent=2, sort_keys=True) + "\n")
            break
        finally:
            if traces:
                finish_traces(traces)
        runs.append(record)
        (series_dir / "index.json").write_text(json.dumps(
            runs, indent=2, sort_keys=True) + "\n")
    aggregate = summary(runs)
    aggregate.update(series_id=series_id, code_revision=code_revision,
                     active_limit_s=args.active_s, scenario=args.scenario)
    (series_dir / "summary.json").write_text(json.dumps(
        aggregate, indent=2, sort_keys=True) + "\n")
    print(json.dumps(aggregate, indent=2, sort_keys=True), flush=True)
    if aggregate["runs_failed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
