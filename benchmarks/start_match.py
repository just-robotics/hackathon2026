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
               DUEL_REVISION=revision())
    run_dir = RESULTS / env['DUEL_RUN_ID']
    run_dir.mkdir(parents=True)
    (run_dir / 'match.yaml').write_text(__import__('yaml').safe_dump(config, sort_keys=False))
    (run_dir / 'environment.json').write_text(json.dumps({k: v for k, v in env.items()
        if k in settings or k in ('DUEL_RUN_ID', 'DUEL_REVISION')}, indent=2))
    started = time.monotonic()
    timings = {}

    def phase(message):
        print(f"[{time.monotonic() - started:.1f}s] {message}", flush=True)

    for action in ('clean', 'up'):
        phase("Пересоздание мира: " + action)
        subprocess.run(['helm', action, 'duel'], cwd=ROOT, env=env, check=True)
    timings['world_launch_s'] = time.monotonic() - started
    phase("Ожидание поз и готовности MPPI обоих роботов; движение закрыто")
    wait_ready(env, time.monotonic()+180)
    timings['navigation_ready_s'] = time.monotonic() - started
    phase("Оба контура готовы. Проверка образа и параметров перед разрешением")
    runtime = runtime_snapshot(env, config)
    (run_dir / "runtime.json").write_text(json.dumps(runtime, indent=2, sort_keys=True))
    timings['runtime_verified_s'] = time.monotonic() - started
    if not args.prepare_only:
        phase("Проверка пройдена. Разрешение движения обоим роботам")
        allow_motion(env, run_dir / 'start.log')
    timings['permissions_granted_s' if not args.prepare_only else 'prepared_s'] = time.monotonic() - started
    (run_dir / 'startup-timing.json').write_text(json.dumps(timings, indent=2))
    phase("Матч запущен" if not args.prepare_only else "Мир подготовлен; разрешения не выдавались")
    print(f"{'Prepared' if args.prepare_only else 'Started'} {env['DUEL_RUN_ID']} using {args.config}")


if __name__ == '__main__':
    main()
