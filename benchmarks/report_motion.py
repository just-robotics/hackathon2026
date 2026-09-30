#!/usr/bin/env python3
"""Compare measured motion in paired duel series, preserving every run."""

import argparse
import json
from math import sqrt
from pathlib import Path
from statistics import mean


def quantile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))]


def trace_error(path, key):
    if not path.is_file():
        return None, None, 0
    samples = json.loads(path.read_text()).get("time_series", [])
    values = [abs(sample[key]) for sample in samples
              if sample.get(key) is not None]
    if not values:
        return None, None, 0
    return sqrt(mean(value * value for value in values)), quantile(values, 0.95), len(values)


def rows(series):
    records = json.loads((series / "index.json").read_text())
    for index, record in enumerate(records):
        if "error" in record or "robots" not in record:
            continue
        for side, robot in zip(("first", "second"), record["robots"]):
            rms, p95, count = trace_error(
                series / f"{index:02d}-trace-{side}.json", "lateral_global_m")
            yield {
                "seed": record["seed"],
                "event": record["outcome"]["event"],
                "role": robot["role"],
                "speed": robot["mean_speed_mps"],
                "error_rms": rms,
                "error_p95": p95,
                "error_samples": count,
                "angular_accel_rms": robot.get("angular_accel_rms_radps2"),
                "stop_go": robot.get("stop_go_events"),
                "collisions": robot.get("collisions"),
            }


def number(value, digits=3):
    return f"{value:.{digits}f}" if value is not None else "—"


def report(series):
    data = list(rows(series))
    print(f"\n{series} ({len(data) // 2} paired runs)")
    print("seed | role | event | speed m/s | global RMS / p95 m | angular accel RMS rad/s² | stop-go | contacts")
    for row in data:
        print(f"{row['seed']} | {row['role']} | {row['event']} | "
              f"{number(row['speed'])} | {number(row['error_rms'])} / "
              f"{number(row['error_p95'])} | "
              f"{number(row['angular_accel_rms'])} | "
              f"{row['stop_go']} | {row['collisions']}")
    for role in ("explorer", "guardian"):
        group = [row for row in data if row["role"] == role]
        if not group:
            continue
        def average(key):
            values = [row[key] for row in group if row[key] is not None]
            return mean(values) if values else None
        print(f"{role}: speed mean/min {number(average('speed'))}/"
              f"{number(min(row['speed'] for row in group))} m/s; "
              f"below 0.2: {sum(row['speed'] < 0.2 for row in group)}/{len(group)}; "
              f"global RMS mean {number(average('error_rms'))} m; "
              f"angular accel RMS mean {number(average('angular_accel_rms'))} rad/s²; "
              f"stop-go mean {number(average('stop_go'), 1)}; "
              f"contacts {sum(row['collisions'] for row in group)}")
    if any(row["error_samples"] == 0 for row in data):
        print("Lateral error unavailable for runs without valid --trace samples.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("series", nargs="+", type=Path)
    args = parser.parse_args()
    for series in args.series:
        report(series)


if __name__ == "__main__":
    main()
