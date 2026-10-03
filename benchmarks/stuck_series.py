#!/usr/bin/env python3
"""Серия прогонов stuck_probe с подстановкой параметров planning.yaml.

    python3 benchmarks/stuck_series.py --label base --runs 2 --seconds 90 \
        [--set local.MPPI.time_steps=40 --set global.robot_radius=0.19] \
        [--obstacles config/simulation_obstacles.yaml|none]

Параметры подставляются во временную копию planning.yaml: репозиторный файл
восстанавливается после серии. Результаты -- results/stuck/<label>.jsonl.
"""
import argparse
import copy
import json
import os
import subprocess
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PLANNING = ROOT / "config/planning.yaml"
ADAPTER = "docker-hsl-adapter-1"


def sh(cmd, **kw):
    kw.setdefault("cwd", ROOT)
    return subprocess.run(cmd, text=True, capture_output=True, **kw)


def apply(original, overrides):
    cfg = copy.deepcopy(original)
    for item in overrides:
        key, value = item.split("=", 1)
        section, name = key.split(".", 1)
        cfg[section][name] = yaml.safe_load(value)
    return cfg


def patch_decision(args):
    """Подменить decision.yaml в контейнере решения и перезапустить его."""
    path = ROOT / "src/hsl_decision/config/decision.yaml"
    cfg = yaml.safe_load(path.read_text())
    params = cfg["decision_manager"]["ros__parameters"]
    for item in args.decision:
        key, value = item.split("=", 1)
        params[key] = yaml.safe_load(value)
    patched = ROOT / "results/stuck/decision_patched.yaml"
    patched.write_text(yaml.safe_dump(cfg, sort_keys=False))
    sh(["docker", "cp", str(patched), "docker-hsl-decision-1:/autoware/src/hsl_decision/config/decision.yaml"])
    sh(["docker", "restart", "docker-hsl-decision-1"])
    time.sleep(8)


def one_run(args, env):
    sh(["python3", "benchmarks/start_match.py", "--prepare-only"], env=env, timeout=400)
    if args.decision:
        patch_decision(args)
    sh(["docker", "cp", "benchmarks/stuck_probe.py", f"{ADAPTER}:/tmp/stuck_probe.py"])
    probe = subprocess.Popen(
        ["docker", "exec", ADAPTER, "bash", "-lc",
         f"source /autoware/install/setup.bash && python3 /tmp/stuck_probe.py --seconds {args.seconds} --diag-out /tmp/diag.json"],
        cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(3)
    # матч активен только при обоих разрешениях, поэтому стража не отключаем от
    # разрешений, а останавливаем его управление: команды до колёс не доходят
    sh(["docker", "stop", "-t", "1", "docker-hsl-opponent-control-1"])
    for container, service in (("docker-hsl-decision-1", "/match/allow_motion"),
                               ("docker-hsl-opponent-decision-1", "/opponent/match/allow_motion")):
        sh(["docker", "exec", container, "bash", "-lc",
            "source /autoware/install/setup.bash && ros2 service call " + service +
            " std_srvs/srv/SetBool '{data: true}'"])
    try:
        out, err = probe.communicate(timeout=args.seconds * 1.6 + 60)
        line = [l for l in out.splitlines() if l.startswith("{")][-1]
        result = json.loads(line)
        sh(["docker", "cp", f"{ADAPTER}:/tmp/diag.json", str(ROOT / "results/stuck" / f"{args.label}_diag{args.index}.json")])
        return result
    except Exception as error:  # noqa: BLE001
        probe.kill()
        return {"error": repr(error)}
    finally:
        sh(["helm", "down", "duel"], cwd=ROOT / "docker", timeout=200)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--runs", type=int, default=2)
    parser.add_argument("--seconds", type=float, default=90.0)
    parser.add_argument("--set", action="append", default=[])
    parser.add_argument("--decision", action="append", default=[], help="decision.yaml: key=value")
    parser.add_argument("--obstacles", default="")
    args = parser.parse_args()
    original = PLANNING.read_text()
    env = os.environ.copy()
    if args.obstacles == "none":
        empty = ROOT / "results/stuck/no_boxes.yaml"
        empty.parent.mkdir(parents=True, exist_ok=True)
        empty.write_text("enabled: false\nboxes: []\n")
        env["HSL_SIM_OBSTACLES_FILE"] = str(empty)
    elif args.obstacles:
        env["HSL_SIM_OBSTACLES_FILE"] = str(Path(args.obstacles).resolve())
    out = ROOT / "results/stuck" / f"{args.label}.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        PLANNING.write_text(yaml.safe_dump(apply(yaml.safe_load(original), args.set), sort_keys=False))
        for index in range(args.runs):
            args.index = index
            result = one_run(args, env)
            result.update(label=args.label, run=index, overrides=args.set, decision=args.decision, obstacles=args.obstacles or "default")
            with out.open("a") as f:
                f.write(json.dumps(result, ensure_ascii=False) + "\n")
            print(json.dumps({k: result.get(k) for k in ("label", "run", "distance_m", "mean_speed", "reached",
                  "longest_stall_s", "contacts", "min_opponent_dist", "error")}, ensure_ascii=False), flush=True)
    finally:
        PLANNING.write_text(original)


if __name__ == "__main__":
    main()
