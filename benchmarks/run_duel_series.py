#!/usr/bin/env python3
"""Run independent Gazebo duels and pair both robots' measured results."""

import argparse
import hashlib
import json
import math
import os
import re
import shlex
import subprocess
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from match_config import DEFAULT_CONFIG, load_config, configuration_environment


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"


def roles_for_run(index, first_role="alternate"):
    if first_role == "alternate":
        first_role = "explorer" if index % 2 == 0 else "guardian"
    if first_role not in ("explorer", "guardian"):
        raise ValueError("first_role must be alternate, explorer or guardian")
    return first_role, "guardian" if first_role == "explorer" else "explorer"


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


def container_name(name, env):
    project = env.get("COMPOSE_PROJECT_NAME", "docker")
    if name.startswith(("docker-hsl-", "docker-gazebo-")):
        return project + name[len("docker"):]
    return name



def command(args, env, timeout=None, log=None):
    args = [container_name(arg, env) for arg in args]
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
        result = subprocess.run(["docker", "exec", container_name(container, env), "bash", "-lc", shell],
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
    seen_gazebo = False
    while time.monotonic() < deadline:
        if time.monotonic() - started > 15:
            gazebo = subprocess.run(["docker", "exec", container_name("docker-gazebo-duel-1", env),
                                     "pgrep", "-x", "gzserver"], cwd=ROOT,
                                    env=env, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL, check=False)
            if gazebo.returncode == 0:
                seen_gazebo = True
            elif seen_gazebo:
                raise TimeoutError("gzserver exited after starting, before both pose streams were ready")
            else:
                state = command(["docker", "inspect", "--format", "{{.State.Running}}",
                                 "docker-gazebo-duel-1"], env, timeout=5).strip()
                if state != "true":
                    raise TimeoutError("Gazebo launch container exited before readiness")
        if all(ready_topic(container, topic, env) for container, topic in topics):
            native_ready = all(ready_topic(container, prefix + "navigation/native_ready",
                                           env, "data", "true")
                               for container, prefix in (("docker-hsl-planning-1", "/"),
                                  ("docker-hsl-opponent-planning-1", "/opponent/")))
            if not native_ready or not ready_topic("docker-gazebo-duel-1",
                    "/simulation/obstacles_ready", env, "data", "true"):
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


def parameter_value(parameters, name):
    if name in parameters:
        return parameters[name]
    value = parameters
    for part in name.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def validate_runtime_metadata(runtime, env):
    params = runtime["effective_parameters"]
    expected = {
        "run_id": env["DUEL_RUN_ID"], "seed": int(env["DUEL_SEED"]),
        "code_revision": env["DUEL_REVISION"],
        "max_active_s": float(env["DUEL_MAX_ACTIVE_S"]),
        "first_role": env["HSL_ROLE"], "second_role": env["HSL_OPPONENT_ROLE"],
        "spawn_x": float(env.get("MAP_ORIGIN_X", env["SPAWN_X"])), "spawn_y": float(env.get("MAP_ORIGIN_Y", env["DUEL_SPAWN_Y"])),
        "second_start": json.loads(env["DUEL_SECOND_START"]),
        **({"first_start": json.loads(env["DUEL_FIRST_START"])} if "DUEL_FIRST_START" in env else {}),
    }
    for name, value in expected.items():
        actual = params["/duel_referee"].get(name)
        if actual != value:
            raise RuntimeError(f"referee {name}={actual!r}, expected {value!r}; runtime does not belong to this evaluation")
    for prefix, role, seed in (("/", env["HSL_ROLE"], int(env["DUEL_SEED"])),
                              ("/opponent/", env["HSL_OPPONENT_ROLE"], int(env["DUEL_OPPONENT_SEED"]))):
        checks = {"trajectory_planner": {"role": role, "random_seed": seed,
                  "arena_bounds": json.loads(env["DUEL_ARENA_BOUNDS"]),
                  "require_match_active": True},
                  "decision_manager": {"role": role,
                      "own_max_speed": float(env.get("HSL_MAX_SPEED", "0.5")),
                      **({"own_start": json.loads(env["DUEL_FIRST_START"] if prefix == "/" else env["DUEL_SECOND_START"]),
                          "opponent_start": json.loads(env["DUEL_SECOND_START"] if prefix == "/" else env["DUEL_FIRST_START"])} if "DUEL_FIRST_START" in env else {})},
                  "hsl_motion_gate": {"require_match_active": True},
                  "opponent_detector": {"use_sim_time": True,
                      "opponent_max_height": float(env.get("HSL_OPPONENT_MAX_HEIGHT", "0.46")),
                      "robot.max_gap_share": 0.12, "robot.line_ratio": 0.35, "strong_arc_min_span_deg": 90.0, "allow_merged_strong": False, "strong_min_inlier_fraction": 0.95}}
        allow_reverse = role == "explorer" and env.get("HSL_ALLOW_REVERSE", "true") == "true"
        checks["native_mppi"] = {
            "role": role, "random_seed": seed,
            "MPPI.PathAngleCritic.forward_preference": not allow_reverse,
            "MPPI.PreferForwardCritic.enabled": False,
            "MPPI.GoalAngleCritic.enabled": False,
            "MPPI.wz_max": float(env.get("HSL_MAX_ANGULAR_SPEED", "1.5")),
            "MPPI.vx_max": float(env.get("HSL_MAX_SPEED", "0.5")),
            "MPPI.vx_min": -float(env.get("HSL_MAX_SPEED", "0.5")) if allow_reverse else 0.0}
        for node, values in checks.items():
            for name, value in values.items():
                actual = parameter_value(params[prefix + node], name)
                if actual != value:
                    raise RuntimeError(f"{prefix + node} {name}={actual!r}, expected {value!r}; runtime does not belong to this evaluation")


def runtime_snapshot(env, config):
    containers = ["docker-" + name + "-1" for name in (
        "hsl-planning", "hsl-opponent-planning", "hsl-control", "hsl-opponent-control",
        "hsl-decision", "hsl-opponent-decision", "hsl-adapter", "hsl-opponent-adapter",
        "hsl-referee", "hsl-metrics", "hsl-opponent-metrics", "gazebo-duel")]
    images = command(["docker", "inspect", "--format", "{{.Name}} {{.Id}} {{.Image}}",
                      *containers], env, timeout=15)
    runtime = {"match_config": config, "configuration_environment": configuration_environment(config),
               "containers": {name.lstrip("/"): {"container_id": cid, "image_id": image}
                              for name, cid, image in (line.split() for line in images.splitlines())}}
    paths = []
    for package in ("hsl_planning", "hsl_decision", "hsl_sim_adapter", "hsl_debug_control",
                    "hsl_interfaces", "hsl_nav2_control", "hsl_perception", "sim_kobuki", "jr_map"):
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
    obstacle_file = ROOT / "config/simulation_obstacles.yaml"
    obstacle_hash = hashlib.sha256(obstacle_file.read_bytes()).hexdigest()
    mounted_hash = command(["docker", "exec", "docker-gazebo-duel-1", "sha256sum",
        "/autoware/simulation_obstacles.yaml"], env, timeout=10).split()[0]
    if mounted_hash != obstacle_hash:
        raise RuntimeError("simulation obstacle configuration differs from mounted file")
    runtime["simulation_obstacles"] = __import__('yaml').safe_load(obstacle_file.read_text())
    runtime["simulation_obstacles_sha256"] = obstacle_hash
    nodes = [prefix + name for prefix in ("/", "/opponent/") for name in
             ("trajectory_planner", "decision_manager", "hsl_motion_gate", "opponent_detector")]
    nodes.extend(("/duel_referee", "/gazebo"))
    nodes.extend(prefix + name for prefix in ("/", "/opponent/")
                 for name in ("native_mppi", "native_mppi/native_costmap"))
    # Read only parameter services, before movement or trace collection begins.
    script = (ROOT / "benchmarks/ros_parameter_snapshot.py").read_text()
    output = command(["docker", "exec", "docker-hsl-control-1", "bash", "-lc",
                      "source /autoware/install/setup.bash && python3 - " +
                      shlex.quote(json.dumps(nodes)) + " <<'PY'\n" + script + "\nPY"],
                     env, timeout=30)
    runtime["effective_parameters"] = json.loads(output)
    validate_runtime_metadata(runtime, env)
    runtime["nav2_package_versions"] = command(["docker", "exec",
        "docker-hsl-planning-1", "dpkg-query", "-W",
        "ros-humble-nav2-mppi-controller", "ros-humble-nav2-controller",
        "ros-humble-nav2-costmap-2d"], env, timeout=15)
    return runtime


def start_traces(series_dir, index, env):
    command(["docker", "cp", str(ROOT / "benchmarks" / "trace_motion.py"),
             "docker-hsl-adapter-1:/tmp/hsl_trace_motion.py"], env, timeout=15)
    traces = []
    spawn_x = float(env.get("MAP_ORIGIN_X", "-0.468"))
    spawn_y = float(env.get("MAP_ORIGIN_Y", "-0.582"))
    for role, namespace in (("first", ""), ("second", " --namespace opponent")):
        destination = series_dir / f"{index:02d}-trace-{role}.json"
        output = destination.open("w")
        script = ("source /autoware/install/setup.bash && python3 "
                  f"/tmp/hsl_trace_motion.py --wall-seconds 1200 "
                  f"--spawn-x {spawn_x} --spawn-y {spawn_y} "
                  "--timeseries" + namespace)
        process = subprocess.Popen(
            ["docker", "exec", container_name("docker-hsl-adapter-1", env), "bash", "-lc", script],
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


def start_obstacle_trial(series_dir, index, run_id, config, env, record_scans=False, obstacle_height=0.8):
    command(["docker", "cp", str(ROOT / "benchmarks" / "unknown_obstacle_trial.py"),
             "docker-hsl-adapter-1:/tmp/hsl_unknown_obstacle_trial.py"], env, timeout=15)
    remote = f"/tmp/hsl_obstacle_{index:02d}.json"
    log = (series_dir / f"{index:02d}-obstacle-node.log").open("w")
    origin = config["simulation"]["map_origin_world"]
    bounds = " ".join(str(value) for value in config["simulation"]["arena_bounds"])
    script = ("source /autoware/install/setup.bash && python3 /tmp/hsl_unknown_obstacle_trial.py "
              f"--run-id {run_id} --output {remote} --origin-x {origin[0]} "
              f"--origin-y {origin[1]} --arena-bounds {bounds} --wall-seconds 1200 --height {obstacle_height}"
              + (" --record-scans" if record_scans else ""))
    process = subprocess.Popen(
        ["docker", "exec", container_name("docker-hsl-adapter-1", env), "bash", "-lc", script],
        cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
    time.sleep(2)
    return process, log, remote


def finish_obstacle_trial(trial, series_dir, index, env):
    process, log, remote = trial
    try:
        process.wait(timeout=20)
        destination = series_dir / f"{index:02d}-obstacle.json"
        command(["docker", "cp", f"docker-hsl-adapter-1:{remote}", str(destination)],
                env, timeout=15)
        report = json.loads(destination.read_text())
        if process.returncode or not report.get("fixture_spawned"):
            raise RuntimeError("unmapped obstacle trial did not spawn the required fixture")
        return {key: value for key, value in report.items()
                if key not in ("samples", "original_route", "offline_alternative", "cloud_samples", "replay_grid")}
    finally:
        if process.poll() is None:
            process.terminate()
        log.close()


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
            ["docker", "exec", container_name("docker-hsl-adapter-1", env), "bash", "-lc", script],
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


def allow_motion(env, log=None):
    """Grant both permissions on the already prepared world; never reset it."""
    for container, service in (("docker-hsl-decision-1", "/match/allow_motion"),
                               ("docker-hsl-opponent-decision-1", "/opponent/match/allow_motion")):
        command(["docker", "exec", container, "bash", "-lc",
                 "source /autoware/install/setup.bash && ros2 service call " + service +
                 " std_srvs/srv/SetBool '{data: true}'"], env, timeout=45,
                log=log if service == "/match/allow_motion" else
                    log.with_name(log.stem + "-opponent" + log.suffix) if log else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=20)
    parser.add_argument("--start-seed", type=int, default=None)
    parser.add_argument("--first-role", choices=("config", "alternate", "explorer", "guardian"),
                        default="config", help="fix physical role assignment or alternate it")
    parser.add_argument("--active-s", type=float, default=None)
    parser.add_argument("--wall-timeout-s", type=float, default=1200.0)
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--trace", action="store_true",
                        help="record active cmd/path diagnostics for each robot")
    parser.add_argument("--rviz", action="store_true",
                        help="show RViz when DISPLAY is available; Gazebo remains headless")
    parser.add_argument("--audit-start", action="store_true",
                        help="verify both gates before and after granting only the first permission")
    parser.add_argument("--unknown-obstacle", action="store_true",
                        help="spawn an unmapped box on the first robot's actual route; requires isolation")
    parser.add_argument("--obstacle-height", type=float, default=0.8,
                        help="unmapped fixture height in metres, at least 0.15; requires unknown-obstacle")
    parser.add_argument("--record-detector-scans", action="store_true",
                        help="save clouds of both observers for offline detector replay; requires unknown-obstacle")
    parser.add_argument("--probe-status", default="",
                        help="save the first planner snapshot with this status")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--isolated-project", default="",
                        help="separate Compose project, ROS domain, Gazebo port and result directory")
    parser.add_argument("--ros-domain-id", type=int, default=73)
    parser.add_argument("--gazebo-port", type=int, default=11418)
    args = parser.parse_args()
    if not math.isfinite(args.obstacle_height) or args.obstacle_height < 0.15:
        parser.error("obstacle-height must be finite and at least 0.15 m")
    if args.obstacle_height != 0.8 and not args.unknown_obstacle:
        parser.error("obstacle-height requires unknown-obstacle")
    if args.record_detector_scans and not args.unknown_obstacle:
        parser.error("record-detector-scans requires unknown-obstacle")
    if args.unknown_obstacle and not args.isolated_project:
        parser.error("unknown-obstacle requires an isolated evaluation project")
    config = load_config(args.config)
    global RESULTS
    if args.isolated_project:
        if not re.fullmatch(r"[a-z][a-z0-9_-]+", args.isolated_project) or args.isolated_project == "docker":
            parser.error("isolated-project must be a separate lower-case Compose project")
        if not 1 <= args.ros_domain_id <= 100 or not 1024 <= args.gazebo_port <= 65535 or args.gazebo_port == 11345:
            parser.error("isolation requires a nonzero ROS domain and a separate Gazebo port")
        RESULTS = ROOT / "results" / "isolated" / args.isolated_project
        RESULTS.mkdir(parents=True, exist_ok=True)
    args.start_seed = config["match"]["seed"] if args.start_seed is None else args.start_seed
    args.active_s = config["match"]["active_seconds"] if args.active_s is None else args.active_s
    args.first_role = config["robot"]["role"] if args.first_role == "config" else args.first_role
    if args.runs < 1 or args.active_s <= 0:
        parser.error("runs and active-s must be positive")
    if args.probe_status and not re.fullmatch(r"[A-Z_]+", args.probe_status):
        parser.error("probe-status must be an uppercase planner status")
    series_id = datetime.now(timezone.utc).strftime("series-%Y%m%dT%H%M%SZ")
    series_dir = RESULTS / series_id
    series_dir.mkdir(parents=True, exist_ok=True)
    (series_dir / "match.yaml").write_text(__import__("yaml").safe_dump(config, sort_keys=False))
    code_revision = revision()
    runs = []
    if args.build:
        command(["helm", "build", "duel"], os.environ.copy(),
                timeout=1800, log=series_dir / "build.log")
    for index in range(args.runs):
        seed = args.start_seed + index
        first_role, second_role = roles_for_run(index, args.first_role)
        run_id = f"{series_id}-{index:02d}"
        env = os.environ.copy()
        if args.isolated_project:
            env.update(COMPOSE_PROJECT_NAME=args.isolated_project, ROS_DOMAIN_ID=str(args.ros_domain_id),
                       GAZEBO_MASTER_URI=f"http://127.0.0.1:{args.gazebo_port}", HSL_RESULTS_DIR=str(RESULTS))
        env.update(configuration_environment(config))
        env.update({"GAZEBO_HEADLESS": "true", "HSL_ROLE": first_role,
                    "HSL_RVIZ_ENABLED": "auto" if args.rviz else "false",
                    "HSL_OPPONENT_ROLE": second_role,
                    "DUEL_RUN_ID": run_id, "DUEL_SEED": str(seed),
                    "DUEL_OPPONENT_SEED": str(seed + 1000003),
                    "DUEL_REVISION": code_revision,
                    "DUEL_MAX_ACTIVE_S": str(args.active_s)})
        record = {"run_id": run_id, "seed": seed,
                  "roles": [first_role, second_role], "match_config": config,
                  "rviz_requested": args.rviz,
                  "effective_environment": {key: env[key] for key in list(configuration_environment(config)) +
                      [k for k in ("COMPOSE_PROJECT_NAME", "ROS_DOMAIN_ID", "GAZEBO_MASTER_URI", "HSL_RESULTS_DIR") if k in env]}}
        print(f"[{index + 1}/{args.runs}] {run_id} "
              f"{first_role}/{second_role} seed={seed}", flush=True)
        traces = []
        probes = []
        obstacle_trial = None
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
            runtime = runtime_snapshot(env, config)
            (series_dir / f"{index:02d}-runtime.json").write_text(
                json.dumps(runtime, indent=2, sort_keys=True) + "\n")
            record["runtime_snapshot"] = f"{index:02d}-runtime.json"
            if args.trace:
                traces = start_traces(series_dir, index, env)
            if args.probe_status:
                probes = start_probes(series_dir, index, args.probe_status, env)
            if args.audit_start:
                command(["docker", "cp", str(ROOT / "benchmarks" / "audit_motion_gate.py"),
                         "docker-hsl-control-1:/tmp/audit_motion_gate.py"], env, timeout=15)
                audit = ["docker", "exec", "docker-hsl-control-1", "bash", "-lc",
                         "source /autoware/install/setup.bash && python3 /tmp/audit_motion_gate.py"]
                command(audit, env, timeout=25, log=series_dir / f"{index:02d}-gate-before-start.json")
                command(["docker", "exec", "docker-hsl-decision-1", "bash", "-lc",
                         "source /autoware/install/setup.bash && ros2 service call /match/allow_motion "
                         "std_srvs/srv/SetBool '{data: true}'"], env, timeout=30)
                command(audit[:-1] + [audit[-1] + " --partial-start"], env, timeout=25,
                        log=series_dir / f"{index:02d}-gate-partial-start.json")
            if args.unknown_obstacle:
                obstacle_trial = start_obstacle_trial(series_dir, index, run_id, config, env, args.record_detector_scans, args.obstacle_height)
            allow_motion(env, series_dir / f"{index:02d}-start.log")
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
            if args.audit_start:
                command(audit, env, timeout=25,
                        log=series_dir / f"{index:02d}-gate-after-finish.json")
            if obstacle_trial:
                trial = obstacle_trial
                obstacle_trial = None
                record["unmapped_obstacle"] = finish_obstacle_trial(trial, series_dir, index, env)
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
            if obstacle_trial:
                process, output, _ = obstacle_trial
                if process.poll() is None:
                    process.terminate()
                output.close()
            if traces:
                finish_traces(traces)
            if probes:
                finish_probes(probes)
            # Completed private worlds still run physics and sensors. Release
            # only the exact runtime we measured; never remove a replacement.
            if args.isolated_project and runtime is not None:
                try:
                    if runtime_state(runtime, env) in ("running", "stopped"):
                        command(["helm", "clean", "duel"], env, timeout=120,
                                log=series_dir / f"{index:02d}-cleanup-after-run.log")
                        record["isolated_runtime_cleaned"] = True
                    else:
                        record["cleanup_skipped"] = "runtime ownership unconfirmed"
                except (RuntimeError, subprocess.TimeoutExpired) as error:
                    record["cleanup_error"] = str(error)
        runs.append(record)
        (series_dir / "index.json").write_text(json.dumps(
            runs, indent=2, sort_keys=True) + "\n")
    aggregate = summary(runs)
    aggregate.update(series_id=series_id, code_revision=code_revision,
                     active_limit_s=args.active_s, match_config=config,
                     role_assignment=args.first_role)
    (series_dir / "summary.json").write_text(json.dumps(
        aggregate, indent=2, sort_keys=True) + "\n")
    print(json.dumps(aggregate, indent=2, sort_keys=True), flush=True)
    if aggregate["runs_failed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
