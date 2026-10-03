#!/usr/bin/env python3

"""Графики телеметрии и анимация движения по карте.

Строит три графика (скорость, боковое отклонение, ошибка курса) и анимацию
движения обоих роботов с путями планировщика поверх карты лабиринта.

    python3 plot_telemetry.py telemetry.json префикс [мир]
"""

import json
import math
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402

from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

sys.path.insert(0, "/home/user/hackathon2026/src/jr_map")

from jr_map.sdf_map_server import collect_boxes, rasterize  # noqa: E402


DATA = sys.argv[1] if len(sys.argv) > 1 else "/tmp/telemetry.json"
PREFIX = sys.argv[2] if len(sys.argv) > 2 else "/tmp/run"
WORLD = (
    sys.argv[3]
    if len(sys.argv) > 3
    else "/home/user/hackathon2026/src/sim_kobuki/worlds/maze_duel.world"
)

# синяя зона: центр и полуразмеры
ZONE = (2.7, 2.7, 0.3, 0.45)
COLORS = {"attacker": "#d32f2f", "defender": "#1976d2"}

frames = json.load(open(DATA))


def series(robot, key):
    """Временной ряд одного поля, пропуски как NaN

    :robot имя робота
    :key имя поля

    :return (время, значения)
    """
    moments, values = [], []

    for frame in frames:
        value = frame.get(robot, {}).get(key)
        moments.append(frame["t"])
        values.append(np.nan if value is None else value)

    return np.array(moments), np.array(values, dtype=float)


figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

for robot, color in COLORS.items():
    moments, speed = series(robot, "v_meas")
    axes[0].plot(moments, speed, color=color, label=f"{robot} факт")

    moments, command = series(robot, "v_cmd")
    axes[0].plot(
        moments, command, color=color, ls="--", alpha=0.5,
        label=f"{robot} команда",
    )

    moments, lateral = series(robot, "e_lat")
    axes[1].plot(moments, lateral, color=color, label=robot)

    moments, heading = series(robot, "e_theta")
    axes[2].plot(moments, np.degrees(heading), color=color, label=robot)

axes[0].set_ylabel("скорость, м/с")
axes[0].set_title("Продольная скорость")
axes[0].axhline(0, color="gray", lw=0.5)
axes[1].set_ylabel("e_lat, м")
axes[1].set_title("Боковое отклонение от пути")
axes[1].axhline(0, color="gray", lw=0.5)
axes[2].set_ylabel("e_theta, градусы")
axes[2].set_title("Ошибка курса")
axes[2].axhline(0, color="gray", lw=0.5)
axes[2].set_xlabel("время, с")

for axis in axes:
    axis.grid(alpha=0.3)
    axis.legend(fontsize=8, ncol=2)

figure.tight_layout()
figure.savefig(f"{PREFIX}_telemetry.png", dpi=110)
print(f"{PREFIX}_telemetry.png")

boxes = collect_boxes(WORLD, 0.25)
grid, origin_x, origin_y = rasterize(boxes, 0.05, 0.4, (0, 0))
extent = [
    origin_x,
    origin_x + grid.shape[1] * 0.05,
    origin_y,
    origin_y + grid.shape[0] * 0.05,
]

figure2, axis = plt.subplots(figsize=(8, 8))
axis.imshow(
    grid == 100, origin="lower", extent=extent,
    cmap="gray_r", alpha=0.85, interpolation="nearest",
)
axis.add_patch(
    plt.Rectangle(
        (ZONE[0] - ZONE[2], ZONE[1] - ZONE[3]),
        2 * ZONE[2], 2 * ZONE[3],
        facecolor="#1976d2", alpha=0.25, edgecolor="#1976d2",
    )
)
axis.set_xlabel("x, м")
axis.set_ylabel("y, м")
axis.set_aspect("equal")

artists = {}
for robot, color in COLORS.items():
    artists[robot] = {
        "body": axis.plot([], [], "o", color=color, ms=11, label=robot)[0],
        "path": axis.plot([], [], "-", color=color, lw=1.4, alpha=0.8)[0],
        "trail": axis.plot([], [], ":", color=color, lw=1, alpha=0.5)[0],
    }

axis.legend(loc="upper left", fontsize=9)
title = axis.set_title("")
trails = {robot: ([], []) for robot in COLORS}


def update(index):
    """Нарисовать один кадр анимации

    :index номер кадра

    :return пустой список, обновление идёт на месте
    """
    frame = frames[index]

    for robot in COLORS:
        state = frame.get(robot, {})
        x, y = state.get("x"), state.get("y")

        if x is not None:
            artists[robot]["body"].set_data([x], [y])
            trails[robot][0].append(x)
            trails[robot][1].append(y)
            artists[robot]["trail"].set_data(*trails[robot])

        path = state.get("path") or []
        if path:
            artists[robot]["path"].set_data(
                [point[0] for point in path], [point[1] for point in path]
            )

    title.set_text(f"t = {frame['t']:.1f} с")
    return []


animation = FuncAnimation(figure2, update, frames=len(frames), interval=100)
animation.save(f"{PREFIX}_motion.gif", writer=PillowWriter(fps=10))
print(f"{PREFIX}_motion.gif")
