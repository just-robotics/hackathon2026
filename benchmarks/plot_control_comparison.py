#!/usr/bin/env python3
"""Plot measured speed and signed global-route offset for matched duel series."""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def traces(series):
    series = Path(series)
    runs = json.loads((series / "index.json").read_text())
    if len(runs) != 1:
        raise ValueError("Use a one-run series with a fixed physical start")
    result = {}
    for index, physical in enumerate(("first", "second")):
        role = runs[0]["robots"][index]["role"]
        data = json.loads((series / f"00-trace-{physical}.json").read_text())
        samples = data.get("time_series")
        if not samples:
            raise ValueError(f"No time_series in {series} {physical}")
        result[role] = samples
    return result, runs[0]["outcome"]["event"]


def smooth(t, values, window_s=1.0):
    values = np.asarray(values, dtype=float)
    result = np.full(values.shape, np.nan)
    for i in range(len(values)):
        indices = (t >= t[i] - window_s / 2) & (t <= t[i] + window_s / 2)
        sample = values[indices]
        if np.isfinite(sample).any():
            result[i] = np.nanmean(sample)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mppi-series", required=True)
    parser.add_argument("--mpc-series", required=True)
    parser.add_argument("--raw-series")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    direct, direct_outcome = traces(args.mppi_series)
    mpc, mpc_outcome = traces(args.mpc_series)
    modes = [("Direct MPPI", direct, direct_outcome),
             ("MPC + smoothed global", mpc, mpc_outcome)]
    if args.raw_series:
        raw, raw_outcome = traces(args.raw_series)
        modes.append(("MPC + raw global", raw, raw_outcome))
    colors = {"Direct MPPI": "#0077b6",
              "MPC + smoothed global": "#d55e00",
              "MPC + raw global": "#5e548e"}
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex="col")
    for col, role in enumerate(("explorer", "guardian")):
        speed_ax = axes[0, col]
        lateral_ax = axes[1, col]
        for name, sample_by_role, outcome in modes:
            samples = sample_by_role[role]
            t = np.asarray([sample["t_s"] for sample in samples])
            speed = np.asarray([sample["speed_mps"] for sample in samples])
            lateral = np.asarray([
                np.nan if sample["lateral_global_m"] is None else
                sample["lateral_global_m"] for sample in samples])
            color = colors[name]
            speed_ax.plot(t, speed, color=color, alpha=0.18, linewidth=0.8)
            speed_ax.plot(t, smooth(t, speed), color=color, linewidth=1.8,
                          label=f"{name}: {np.mean(speed):.3f} m/s, {outcome}")
            lateral_ax.plot(t, lateral, color=color, alpha=0.18, linewidth=0.8)
            lateral_ax.plot(t, smooth(t, lateral), color=color, linewidth=1.8,
                            label=f"{name}: RMS {np.sqrt(np.nanmean(lateral ** 2)):.3f} m")
        speed_ax.set_title(role.capitalize())
        speed_ax.set_ylabel("Measured speed, m/s")
        speed_ax.set_ylim(bottom=0)
        speed_ax.grid(alpha=0.25)
        speed_ax.legend(fontsize=8)
        lateral_ax.axhline(0, color="black", linewidth=0.7)
        lateral_ax.set_ylabel("Signed offset to global route, m")
        lateral_ax.set_xlabel("Active simulation time, s")
        lateral_ax.grid(alpha=0.25)
        lateral_ax.legend(fontsize=8)
    fig.suptitle("Same start and seed: measured motion vs global route")
    fig.tight_layout()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    fig.savefig(output.with_suffix(".svg"))
    print(output)


if __name__ == "__main__":
    main()
