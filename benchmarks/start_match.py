#!/usr/bin/env python3
"""Prepare a fresh match using external YAML, then grant both movement permissions."""
import argparse
from datetime import datetime, timezone
import json
import os
import subprocess
import time
from match_config import DEFAULT_CONFIG, load_config, configuration_environment
from run_duel_series import ROOT, RESULTS, wait_ready, revision, allow_motion, runtime_snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=os.environ.get('HSL_MATCH_CONFIG', str(DEFAULT_CONFIG)))
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--print-env', action='store_true', help='validate and print effective settings without starting')
    args = parser.parse_args()
    config = load_config(args.config)
    settings = configuration_environment(config)
    if args.print_env:
        print(json.dumps(settings, indent=2))
        return
    env = os.environ.copy()
    env.update(settings)
    if not env.get("DISPLAY"):
        env["GAZEBO_HEADLESS"] = "true"
        env["HSL_RVIZ_ENABLED"] = "false"
    env.update(DUEL_RUN_ID=datetime.now(timezone.utc).strftime('manual-%Y%m%dT%H%M%S%fZ'),
               DUEL_REVISION=revision(), HSL_LOCAL_BACKEND='nav2_cpp')
    run_dir = RESULTS / env['DUEL_RUN_ID']
    run_dir.mkdir(parents=True)
    (run_dir / 'match.yaml').write_text(__import__('yaml').safe_dump(config, sort_keys=False))
    (run_dir / 'environment.json').write_text(json.dumps({k: v for k, v in env.items()
        if k in settings or k in ('DUEL_RUN_ID', 'DUEL_REVISION', 'HSL_LOCAL_BACKEND')}, indent=2))
    for action in ('clean', 'up'):
        subprocess.run(['helm', action, 'duel'], cwd=ROOT, env=env, check=True)
    wait_ready(env, time.monotonic()+180)
    runtime = runtime_snapshot(env, config)
    (run_dir / "runtime.json").write_text(json.dumps(runtime, indent=2, sort_keys=True))
    if not args.prepare_only:
        allow_motion(env, run_dir / 'start.log')
    print(f"{'Prepared' if args.prepare_only else 'Started'} {env['DUEL_RUN_ID']} using {args.config}")


if __name__ == '__main__':
    main()
