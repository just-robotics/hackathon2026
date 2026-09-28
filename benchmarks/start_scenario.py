#!/usr/bin/env python3
"""Reset the duel world and start a selected maze scenario."""

import argparse
import os
import subprocess
import sys
import time

from run_duel_series import ROOT, wait_ready
from scenarios import SCENARIOS, scenario_environment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", type=int, choices=SCENARIOS)
    parser.add_argument("--prepare-only", action="store_true",
                        help="reset the world and wait for poses, but keep motion disabled")
    args = parser.parse_args()
    env = os.environ.copy()
    env.update(scenario_environment(args.scenario))
    # Every invocation starts from fresh poses, including after a prior match.
    for action in ("clean", "up"):
        subprocess.run(["helm", action, "duel"], cwd=ROOT, env=env, check=True)
    wait_ready(env, time.monotonic() + 180)
    if not args.prepare_only:
        subprocess.run(["helm", "start_match"], cwd=ROOT, env=env, check=True)
    print(f"{'Prepared' if args.prepare_only else 'Started'} match {args.scenario}: guardian at world "
          f"{SCENARIOS[args.scenario]}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (subprocess.CalledProcessError, TimeoutError) as error:
        print(f"Could not start scenario: {error}", file=sys.stderr)
        raise SystemExit(1)
