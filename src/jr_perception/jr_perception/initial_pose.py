#!/usr/bin/env python3
"""Начальная поза робота на карте по первым сканам бэга.

Для сырых бэгов, где старт неизвестен и AMCL сам не сходится: точки стен
(срез по высоте над base_footprint) из первых сканов перебором
сопоставляются с занятыми клетками карты -- позиция по свободным клеткам,
курс через 2 градуса, потом уточнение вокруг лучшего.

    ros2 run jr_perception initial_pose.py <бэг> <карта.yaml>  ->  печатает x y yaw
"""

import argparse
import math
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

from jr_perception.record_background import chain_to
from jr_perception.robot_detector import cloud_to_xyz, quaternion_matrix


def read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    tokens, position = [], 0
    while len(tokens) < 4:
        while data[position:position + 1].isspace():
            position += 1
        if data[position:position + 1] == b"#":
            position = data.index(b"\n", position)
            continue
        end = position
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[position:end])
        position = end
    width, height = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data, np.uint8, width * height, position + 1).reshape(height, width)


def grow(mask: np.ndarray, cells: int) -> np.ndarray:
    for _ in range(cells):
        grown = mask.copy()
        grown[1:] |= mask[:-1]
        grown[:-1] |= mask[1:]
        grown[:, 1:] |= mask[:, :-1]
        grown[:, :-1] |= mask[:, 1:]
        mask = grown
    return mask


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bag")
    parser.add_argument("map")
    parser.add_argument("--scans", type=int, default=3)
    parser.add_argument("--min-height", type=float, default=0.10)
    parser.add_argument("--max-height", type=float, default=0.40)
    parser.add_argument("--min-range", type=float, default=0.5)
    parser.add_argument("--points", type=int, default=600)
    arguments = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=arguments.bag, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/livox/lidar", "/tf_static"]))
    edges, scans, frame = {}, [], None
    while reader.has_next() and len(scans) < arguments.scans:
        topic, data, _ = reader.read_next()
        if topic == "/tf_static":
            for t in deserialize_message(data, TFMessage).transforms:
                tr, r = t.transform.translation, t.transform.rotation
                edges[t.child_frame_id] = (t.header.frame_id, quaternion_matrix([r.x, r.y, r.z, r.w]),
                                           np.array([tr.x, tr.y, tr.z]))
        elif edges:
            message = deserialize_message(data, PointCloud2)
            frame = message.header.frame_id
            scans.append(cloud_to_xyz(message))

    rotation, translation = chain_to(frame, "base_footprint", edges)
    points = np.concatenate(scans) @ rotation.T + translation
    distance = np.hypot(points[:, 0], points[:, 1])
    walls = points[(points[:, 2] > arguments.min_height) & (points[:, 2] < arguments.max_height)
                   & (distance > arguments.min_range)][:, :2]
    rng = np.random.default_rng(0)
    if len(walls) > arguments.points:
        walls = walls[rng.choice(len(walls), arguments.points, replace=False)]

    meta = yaml.safe_load(Path(arguments.map).read_text())
    grid = read_pgm(Path(arguments.map).parent / meta["image"])[::-1]
    resolution = float(meta["resolution"])
    origin = np.array(meta["origin"][:2], dtype=float)
    occupancy = (255 - grid.astype(float)) / 255.0
    occupied = occupancy > float(meta.get("occupied_thresh", 0.65))
    free = occupancy < float(meta.get("free_thresh", 0.196))
    height, width = grid.shape

    def score(candidates: np.ndarray, yaws: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Доля точек стен в клетках target для поз (позиции x курсы)"""
        result = np.zeros((len(candidates), len(yaws)))
        for j, yaw in enumerate(yaws):
            c, s = math.cos(yaw), math.sin(yaw)
            rotated = walls @ np.array([[c, s], [-s, c]])
            world = candidates[:, None, :] + rotated[None, :, :]
            cells = np.floor((world - origin) / resolution).astype(int)
            inside = (cells[..., 0] >= 0) & (cells[..., 0] < width) & (cells[..., 1] >= 0) & (cells[..., 1] < height)
            hit = np.zeros(cells.shape[:2], dtype=bool)
            hit[inside] = target[cells[..., 1][inside], cells[..., 0][inside]]
            result[:, j] = hit.mean(axis=1)
        return result

    def score_cells(rows: np.ndarray, cols: np.ndarray, yaws: np.ndarray, target: np.ndarray) -> np.ndarray:
        """То же, что score, для поз в центрах клеток (rows, cols)

        Из центра клетки точка попадает в клетку col + floor(0.5 + x / resolution):
        сдвиг в клетках от кандидата не зависит, и вместо координат всех поз
        складываются целые индексы в сетке с полями, за картой -- промах.
        """
        pad = int(np.ceil(np.hypot(walls[:, 0], walls[:, 1]).max() / resolution)) + 2
        padded = np.zeros((height + 2 * pad, width + 2 * pad), dtype=bool)
        padded[pad:-pad, pad:-pad] = target
        stride = padded.shape[1]
        start = (rows + pad) * stride + cols + pad
        result = np.zeros((len(rows), len(yaws)))
        for j, yaw in enumerate(yaws):
            c, s = math.cos(yaw), math.sin(yaw)
            shift = np.floor(0.5 + walls @ np.array([[c, s], [-s, c]]) / resolution).astype(int)
            result[:, j] = padded.ravel()[start[:, None] + shift[:, 1] * stride + shift[:, 0]].mean(axis=1)
        return result

    # грубо: центры свободных клеток, курс через 2 градуса, стены с запасом в клетку
    rows, cols = np.nonzero(free)
    candidates = origin + (np.c_[cols, rows] + 0.5) * resolution
    yaws = np.radians(np.arange(0.0, 360.0, 2.0))
    coarse = score_cells(rows, cols, yaws, grow(occupied, 1))
    best = np.unravel_index(np.argmax(coarse), coarse.shape)
    position, yaw = candidates[best[0]], yaws[best[1]]
    second = np.sort(coarse.ravel())[-1]

    # точно: +-10 см через 1 см, +-3 градуса через 0.25
    offsets = np.arange(-0.10, 0.1001, 0.01)
    fine_candidates = position + np.array([[dx, dy] for dx in offsets for dy in offsets])
    fine_yaws = yaw + np.radians(np.arange(-3.0, 3.001, 0.25))
    fine = score(fine_candidates, fine_yaws, occupied)
    best = np.unravel_index(np.argmax(fine), fine.shape)
    position, yaw = fine_candidates[best[0]], fine_yaws[best[1]]
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

    print(f"# {len(walls)} точек стен, совпадение грубо {second:.2f}, точно {fine.max():.2f}")
    print(f"{position[0]:.3f} {position[1]:.3f} {yaw:.4f}")


if __name__ == "__main__":
    main()
