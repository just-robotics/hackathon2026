#!/usr/bin/env python3
"""Compare cached own-pose snapshots by simulation sample time; no LOS proof."""

import argparse
from bisect import bisect_left
import json
from math import atan2, degrees, hypot, pi
from pathlib import Path


def positions(path):
    samples = json.loads(path.read_text()).get("time_series", [])
    # Repeated receipt samples at the same simulation timestamp add no evidence.
    unique = {s["sim_t_s"]: s for s in samples
              if all(s.get(k) is not None for k in
                     ("sim_t_s", "own_x_m", "own_y_m", "own_yaw_rad"))}
    return sorted(unique.items())


def interpolate(points, times, stamp, max_gap):
    index = bisect_left(times, stamp)
    if index < len(times) and times[index] == stamp:
        s = points[index][1]
        return s["own_x_m"], s["own_y_m"]
    if index == 0 or index == len(times):
        return None
    left_t, left = points[index - 1]
    right_t, right = points[index]
    if right_t - left_t > max_gap:
        return None
    weight = (stamp - left_t) / (right_t - left_t)
    return tuple(left[k] + weight * (right[k] - left[k])
                 for k in ("own_x_m", "own_y_m"))


def analyze(series, max_gap=0.3):
    records = json.loads((series / "index.json").read_text())
    output = []
    for index, record in enumerate(records):
        if "error" in record or len(record.get("robots", [])) != 2:
            continue
        traces = {}
        for side, robot in zip(("first", "second"), record["robots"]):
            path = series / f"{index:02d}-trace-{side}.json"
            if path.is_file():
                traces[robot["role"]] = (robot, positions(path))
        if set(traces) != {"guardian", "explorer"}:
            continue
        guardian, own = traces["guardian"]
        _, peer = traces["explorer"]
        peer_times = [t for t, _ in peer]
        start, end = guardian["window_start_sim_s"], guardian["window_end_sim_s"]
        samples = []
        missing = 0
        for stamp, sample in own:
            if not start <= stamp <= end:
                continue
            target = interpolate(peer, peer_times, stamp, max_gap)
            if target is None:
                missing += 1
                continue
            dx = target[0] - sample["own_x_m"]
            dy = target[1] - sample["own_y_m"]
            heading = (atan2(dy, dx) - sample["own_yaw_rad"] + pi) % (2 * pi) - pi
            samples.append({"active_s": stamp - start,
                            "distance_m": hypot(dx, dy),
                            "heading_error_deg": abs(degrees(heading)),
                            "behavior": sample.get("behavior"),
                            "capture_strategy": sample.get("planning", {}).get("capture_strategy"),
                            "cmd_speed_mps": sample.get("cmd_speed_mps")})
        output.append({"seed": record["seed"],
                       "event": record["outcome"]["event"],
                       "paired_samples": len(samples), "unpaired_samples": missing,
                       "closest": min(samples, key=lambda s: s["distance_m"]) if samples else None,
                       "samples_with_distance_and_heading": sum(
                           s["distance_m"] < 0.45 and s["heading_error_deg"] <= 45
                           for s in samples),
                       "last_two_seconds": [s for s in samples if s["active_s"] >= end - start - 2]})
    return {"series": str(series), "max_interpolation_gap_s": max_gap,
            "limitation": "Cached own poses paired by trace simulation sample time, "
                          "not original odometry stamps. No LOS check or extrapolation. "
                          "May miss capture between samples; referee outcome remains authoritative.",
            "runs": output}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("series", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.series), indent=2))


if __name__ == "__main__":
    main()
