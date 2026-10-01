#!/usr/bin/env python3
"""Run independent Gazebo duels and pair both robots' measured results."""

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from scenarios import SCENARIOS, scenario_environment


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "hsl_planning"))
from hsl_planning.backend import resolve_backend
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


def ready_topic(container, topic, env, field="pose.pose.position", expected=None):
    shell = ("source /autoware/install/setup.bash && timeout 7 "
             f"ros2 topic echo {topic} --no-daemon --once --field {field}")
    try:
        result = subprocess.run(["docker", "exec", container, "bash", "-lc", shell],
                                cwd=ROOT, env=env, stdout=subprocess.PIPE, text=True,
                                stderr=subprocess.DEVNULL, timeout=12, check=False)
        return result.returncode == 0 and (expected is None or
               expected in [line.strip().lower() for line in result.stdout.splitlines()])
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
            if env.get("HSL_LOCAL_BACKEND", "python") == "nav2_cpp":
                native_ready = all(ready_topic(container, prefix + "navigation/native_ready",
                                               env, "data", "true")
                                   for container, prefix in (("docker-hsl-planning-1", "/"),
                                      ("docker-hsl-opponent-planning-1", "/opponent/")))
                if not native_ready:
                    time.sleep(2)
                    continue
            time.sleep(2)
            return
        time.sleep(2)
    raise TimeoutError("autonomous navigation streams did not become ready")


def classify_runtime(expected, observed):
    """Container identity changes are terminal; an observation timeout is not."""
    if observed is None:
        return "unknown"
    if set(observed) != set(expected) or any(
            observed[name]["id"] != value["container_id"]
            for name, value in expected.items()):
        return "replaced"
    return "running" if all(value["running"] for value in observed.values()) else "stopped"


def runtime_state(runtime, env):
    expected = runtime["containers"]
    fmt = '{"name":"{{.Name}}","id":"{{.Id}}","running":{{.State.Running}}}'
    try:
        result = subprocess.run(["docker", "inspect", "--format", fmt, *expected],
                                cwd=ROOT, env=env, text=True, capture_output=True, timeout=5)
    except subprocess.TimeoutExpired:
        return "unknown"
    if result.returncode:
        return "replaced" if "No such object" in result.stderr else "unknown"
    observed = {value["name"].lstrip("/"): value
                for value in (json.loads(line) for line in result.stdout.splitlines())}
    return classify_runtime(expected, observed)


def wait_report(path, run_id, deadline, stopped=False, runtime=None, env=None):
    next_health_check = 0.0
    while time.monotonic() < deadline:
        report = read_report(path, run_id, stopped)
        if report is not None:
            return report
        if runtime is not None and time.monotonic() >= next_health_check:
            state = runtime_state(runtime, env)
            if state in ("stopped", "replaced"):
                raise RuntimeError(f"evaluation containers {state}; no outcome accepted for {run_id}")
            next_health_check = time.monotonic() + 5.0
        time.sleep(2)
    raise TimeoutError(f"result {path} for {run_id} did not appear")


def validate_runtime_metadata(runtime, env):
    params = runtime["effective_parameters"]
    expected = {
        "run_id": env["DUEL_RUN_ID"], "seed": int(env["DUEL_SEED"]),
        "code_revision": env["DUEL_REVISION"],
        "max_active_s": float(env["DUEL_MAX_ACTIVE_S"]),
        "first_role": env["HSL_ROLE"], "second_role": env["HSL_OPPONENT_ROLE"],
        "spawn_x": float(env["SPAWN_X"]), "spawn_y": float(env["DUEL_SPAWN_Y"]),
        "second_start": json.loads(env["DUEL_SECOND_START"]),
    }
    for name, value in expected.items():
        actual = params["/duel_referee"].get(name)
        if actual != value:
            raise RuntimeError(f"referee {name}={actual!r}, expected {value!r}; runtime does not belong to this evaluation")
    for prefix, role, seed in (("/", env["HSL_ROLE"], int(env["DUEL_SEED"])),
                              ("/opponent/", env["HSL_OPPONENT_ROLE"], int(env["DUEL_OPPONENT_SEED"]))):
        checks = {"trajectory_planner": {"role": role, "random_seed": seed,
                  "local_backend": env["HSL_LOCAL_BACKEND"],
                  "arena_bounds": json.loads(env["DUEL_ARENA_BOUNDS"])},
                  "decision_manager": {"role": role}}
        if env["HSL_LOCAL_BACKEND"] == "nav2_cpp":
            checks["native_mppi"] = {"role": role, "random_seed": seed}
        for node, values in checks.items():
            for name, value in values.items():
                actual = params[prefix + node].get(name)
                if actual != value:
                    raise RuntimeError(f"{prefix + node} {name}={actual!r}, expected {value!r}; runtime does not belong to this evaluation")


def runtime_snapshot(env, scenario):
    containers = ["docker-" + name + "-1" for name in (
        "hsl-planning", "hsl-opponent-planning", "hsl-control", "hsl-opponent-control",
        "hsl-decision", "hsl-opponent-decision", "hsl-adapter", "hsl-opponent-adapter",
        "hsl-referee", "hsl-metrics", "hsl-opponent-metrics", "gazebo-duel")]
    images = command(["docker", "inspect", "--format", "{{.Name}} {{.Id}} {{.Image}}",
                      *containers], env, timeout=15)
    runtime = {"scenario_environment": scenario_environment(scenario),
               "containers": {name.lstrip("/"): {"container_id": cid, "image_id": image}
                              for name, cid, image in (line.split() for line in images.splitlines())}}
    paths = []
    for package in ("hsl_planning", "hsl_decision", "hsl_sim_adapter", "hsl_debug_control",
                    "hsl_interfaces", "hsl_nav2_control", "sim_kobuki", "jr_map",
                    "mpc_motion_control/workspace/src/swarm_controller",
                    "mpc_motion_control/workspace/src/swarm_msgs"):
        paths.extend(path for path in (ROOT / "src" / package).rglob("*")
                     if path.is_file() and path.suffix in
                     (".py", ".yaml", ".cpp", ".hpp", ".msg", ".world", ".xacro"))
    paths = sorted(paths)
    expected = {"/autoware/" + str(path.relative_to(ROOT)):
                hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    for container in ("docker-hsl-planning-1", "docker-hsl-opponent-planning-1"):
        actual = command(["docker", "exec", container, "sha256sum", *expected],
                         env, timeout=15)
        actual = {name: digest for digest, name in (line.split(maxsplit=1)
                  for line in actual.splitlines())}
        if actual != expected:
            raise RuntimeError(f"navigation sources in {container} differ from the worktree; rebuild duel")
    runtime["verified_source_sha256"] = expected
    nodes = [prefix + name for prefix in ("/", "/opponent/") for name in
             ("trajectory_planner", "decision_manager", "hsl_cc_mpc", "hsl_lat_mpc",
              "hsl_mpc_gate")]
    nodes.extend(("/duel_referee", "/gazebo"))
    if env.get("HSL_LOCAL_BACKEND", "python") == "nav2_cpp":
        nodes.extend(prefix + name for prefix in ("/", "/opponent/")
                     for name in ("native_mppi", "native_mppi/native_costmap"))
    # Read only parameter services, before movement or trace collection begins.
    script = """import json, subprocess, yaml
nodes = %r
result = {}
for node in nodes:
    output = subprocess.run(['ros2', 'param', 'dump', node, '--no-daemon'],
                            text=True, capture_output=True, timeout=15, check=True)
    parsed = yaml.safe_load(output.stdout)
    params = next((value.get('ros__parameters') for value in (parsed or {}).values()
                   if isinstance(value, dict)), None)
    if not isinstance(params, dict):
        raise RuntimeError('No parameter snapshot for ' + node)
    result[node] = params
print(json.dumps(result))
""" % nodes
    output = command(["docker", "exec", "docker-hsl-control-1", "bash", "-lc",
                      "source /autoware/install/setup.bash && python3 - <<'PY'\n" + script + "PY"],
                     env, timeout=180)
    runtime["effective_parameters"] = json.loads(output)
    validate_runtime_metadata(runtime, env)
    if env.get("HSL_LOCAL_BACKEND", "python") == "nav2_cpp":
        runtime["nav2_package_versions"] = command(["docker", "exec",
            "docker-hsl-planning-1", "dpkg-query", "-W",
            "ros-humble-nav2-mppi-controller", "ros-humble-nav2-controller",
            "ros-humble-nav2-costmap-2d"], env, timeout=15)
    return runtime


def start_traces(series_dir, index, env):
    command(["docker", "cp", str(ROOT / "benchmarks" / "trace_motion.py"),
             "docker-hsl-adapter-1:/tmp/hsl_trace_motion.py"], env, timeout=15)
    traces = []
    spawn_x = float(env.get("SPAWN_X", "-0.34"))
    spawn_y = float(env.get("DUEL_SPAWN_Y", "0.4"))
    control_mode = env.get("HSL_CONTROL_MODE", "mppi")
    if control_mode not in ("mpc", "mppi"):
        raise ValueError("HSL_CONTROL_MODE must be mpc or mppi")
    path_source = ("local" if control_mode == "mppi" else
                   env.get("HSL_MPC_PATH_SOURCE", "local"))
    for role, namespace in (("first", ""), ("second", " --namespace opponent")):
        destination = series_dir / f"{index:02d}-trace-{role}.json"
        output = destination.open("w")
        script = ("source /autoware/install/setup.bash && python3 "
                  f"/tmp/hsl_trace_motion.py --wall-seconds 1200 "
                  f"--spawn-x {spawn_x} --spawn-y {spawn_y} "
                  f"--control-mode {control_mode} "
                  f"--control-path-source {path_source} --timeseries" + namespace)
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


def start_probes(series_dir, index, status, env):
    command(["docker", "cp", str(ROOT / "benchmarks" / "planner_probe.py"),
             "docker-hsl-adapter-1:/tmp/hsl_planner_probe.py"], env, timeout=15)
    probes = []
    for role, namespace in (("first", ""), ("second", " --namespace opponent")):
        destination = series_dir / f"{index:02d}-probe-{role}.json"
        output = destination.open("w")
        script = ("source /autoware/install/setup.bash && python3 "
                  f"/tmp/hsl_planner_probe.py --on-status {status} --wait-s 1200" +
                  namespace)
        process = subprocess.Popen(
            ["docker", "exec", "docker-hsl-adapter-1", "bash", "-lc", script],
            cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT)
        probes.append((process, output, destination))
    return probes


def finish_probes(probes):
    for process, output, _ in probes:
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
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
    parser.add_argument("--probe-status", default="",
                        help="save the first planner snapshot with this status")
    parser.add_argument("--scenario", type=int, choices=SCENARIOS, default=1)
    args = parser.parse_args()
    if args.runs < 1 or args.active_s <= 0:
        parser.error("runs and active-s must be positive")
    if args.probe_status and not re.fullmatch(r"[A-Z_]+", args.probe_status):
        parser.error("probe-status must be an uppercase planner status")
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
        env["HSL_LOCAL_BACKEND"] = resolve_backend(
            env.get("HSL_CONTROL_MODE", "mppi"), env.get("HSL_LOCAL_BACKEND", "auto"))
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
        probes = []
        runtime = None
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
            runtime = runtime_snapshot(env, args.scenario)
            (series_dir / f"{index:02d}-runtime.json").write_text(
                json.dumps(runtime, indent=2, sort_keys=True) + "\n")
            record["runtime_snapshot"] = f"{index:02d}-runtime.json"
            if args.trace:
                traces = start_traces(series_dir, index, env)
            if args.probe_status:
                probes = start_probes(series_dir, index, args.probe_status, env)
            command(["helm", "start_match"], env, timeout=45,
                    log=series_dir / f"{index:02d}-start.log")
            outcome = wait_report(RESULTS / "latest_outcome.json", run_id,
                                  time.monotonic() + args.wall_timeout_s, runtime=runtime, env=env)
            first = wait_report(RESULTS / "latest.json", run_id,
                                time.monotonic() + 45, stopped=True)
            second = wait_report(RESULTS / "opponent/latest.json", run_id,
                                 time.monotonic() + 45, stopped=True)
            for report in (first, second):
                if (report.get("window_source") != "referee" or
                        report.get("window_start_sim_s") != outcome.get("started_at_sim_s") or
                        report.get("window_end_sim_s") != outcome.get("finished_at_sim_s")):
                    raise RuntimeError("robot metrics do not share the referee interval")
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
                if runtime is not None and runtime_state(runtime, env) in ("running", "stopped"):
                    command(["helm", "stop_match"], env, timeout=30,
                            log=series_dir / f"{index:02d}-stop-after-error.log")
                else:
                    record["cleanup_skipped"] = "runtime ownership unconfirmed; leave manual match untouched"
            except (RuntimeError, subprocess.TimeoutExpired):
                pass
            runs.append(record)
            (series_dir / "index.json").write_text(json.dumps(
                runs, indent=2, sort_keys=True) + "\n")
            break
        finally:
            if traces:
                finish_traces(traces)
            if probes:
                finish_probes(probes)
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
