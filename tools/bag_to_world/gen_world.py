#!/usr/bin/env python3
"""Мир Gazebo по карте из build_map.py: панели как box-коллизии.

Точки стен из облака (высота WALL_LOW..WALL_HIGH) берутся только там, где
на итоговой карте занятости стена арены, и по ним последовательно ищутся
прямые (RANSAC). Прямая режется на отрезки по разрывам больше GAP; каждый
отрезок -- панель высотой HEIGHT и толщиной THICKNESS.

    gen_world.py <каталог карты> <имя карты> <имя мира> <выход .world>
"""

import math
import os
import sys

import numpy as np
import yaml
from scipy import ndimage

WALL_LOW = 0.45
WALL_HIGH = 1.0
THICKNESS = 0.02
INLIER = 0.03
MIN_POINTS = 40
MIN_LENGTH = 0.40
GAP = 0.25
# прямые почти параллельные (MERGE_ANGLE, град) и ближе MERGE_OFFSET, м, --
# одна стена: панели стоят со сдвигами, облако стены толще одной линии
MERGE_ANGLE = 5.0
MERGE_OFFSET = 0.15
# конец панели ближе этого к пересечению с другой панелью переносится в
# пересечение: углы смыкаются, концы не торчат за стену, м
JOIN = 0.2
# прямые ближе этого к осям доворачиваются точно на ось, град
SNAP = 3.0


def read_pcd(path):
    raw = open(path, "rb").read()
    start = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    return np.frombuffer(raw[start:], "<f4").reshape(-1, 3).astype(float)


def read_map(directory, name):
    meta = yaml.safe_load(open(os.path.join(directory, f"{name}.yaml")))
    raw = open(os.path.join(directory, meta["image"]), "rb").read().split(b"\n", 3)
    width, rows = map(int, raw[1].split())
    image = np.frombuffer(raw[3], np.uint8).reshape(rows, width)[::-1]
    return image, np.array(meta["origin"][:2]), meta["resolution"]


def fit_lines(xy, rng):
    """Последовательный RANSAC: (направление, точка, инлайеры) по убыванию"""
    lines = []
    left = xy
    while len(left) >= MIN_POINTS:
        best = None
        for _ in range(400):
            a, b = left[rng.choice(len(left), 2, replace=False)]
            direction = b - a
            length = np.linalg.norm(direction)
            if length < 0.1:
                continue
            direction /= length
            normal = np.array([-direction[1], direction[0]])
            inliers = np.abs((left - a) @ normal) < INLIER
            if best is None or inliers.sum() > best[2].sum():
                best = (direction, a, inliers)
        if best is None or best[2].sum() < MIN_POINTS:
            break
        # уточнение по МНК
        points = left[best[2]]
        center = points.mean(axis=0)
        _, _, vt = np.linalg.svd(points - center)
        lines.append((vt[0], center, points))
        left = left[~best[2]]
    return lines


def merge_lines(lines):
    """Слить почти совпадающие прямые и пересчитать каждую по МНК"""
    groups = []
    for direction, center, points in lines:
        for group in groups:
            gd, gc, _ = group[0]
            cos = abs(float(direction @ gd))
            normal = np.array([-gd[1], gd[0]])
            if cos > math.cos(math.radians(MERGE_ANGLE)) and abs((center - gc) @ normal) < MERGE_OFFSET:
                group.append((direction, center, points))
                break
        else:
            groups.append([(direction, center, points)])

    merged = []
    for group in groups:
        points = np.vstack([points for _, _, points in group])
        center = points.mean(axis=0)
        _, _, vt = np.linalg.svd(points - center)
        merged.append((vt[0], center, points))
    return merged


def split_segments(direction, center, points):
    """Отрезки вдоль прямой: разрыв больше GAP -- новая панель"""
    t = np.sort((points - center) @ direction)
    breaks = np.nonzero(np.diff(t) > GAP)[0]
    pieces = np.split(t, breaks + 1)
    segments = []
    for piece in pieces:
        if len(piece) < MIN_POINTS // 2:
            continue
        low, high = np.percentile(piece, [1, 99])
        if high - low >= MIN_LENGTH:
            segments.append((center + low * direction, center + high * direction))
    return segments


def join_corners(segments):
    """Сомкнуть почти сходящиеся панели в точке пересечения их прямых"""
    segments = [[np.array(a, float), np.array(b, float)] for a, b in segments]
    for i, segment in enumerate(segments):
        for end in range(2):
            point = segment[end]
            best = None
            for j, (a, b) in enumerate(segments):
                if i == j:
                    continue
                p, r = segment[0], segment[1] - segment[0]
                q, d = a, b - a
                cross = r[0] * d[1] - r[1] * d[0]
                if abs(cross) < 1e-6 * np.linalg.norm(r) * np.linalg.norm(d):
                    continue  # параллельны
                t = ((q - p)[0] * d[1] - (q - p)[1] * d[0]) / cross
                u = ((q - p)[0] * r[1] - (q - p)[1] * r[0]) / cross
                corner = p + t * r
                # пересечение должно лежать на другой панели или рядом с её концом
                length = np.linalg.norm(d)
                if not (-JOIN / length <= u <= 1 + JOIN / length):
                    continue
                distance = np.linalg.norm(corner - point)
                if distance < JOIN and (best is None or distance < best[0]):
                    best = (distance, corner)
            if best is not None:
                segment[end] = best[1]
    return [tuple(segment) for segment in segments]


def snap(direction):
    angle = math.degrees(math.atan2(direction[1], direction[0])) % 180.0
    for axis in (0.0, 90.0, 180.0):
        if abs(angle - axis) < SNAP:
            angle = axis
    return math.radians(angle)


def panel(name, start, end, height):
    middle = (start + end) / 2.0
    delta = end - start
    yaw = math.atan2(delta[1], delta[0])
    length = float(np.linalg.norm(delta)) + THICKNESS
    size = f"{length:.3f} {THICKNESS} {height:.3f}"
    return f"""      <link name="{name}">
        <pose>{middle[0]:.3f} {middle[1]:.3f} 0 0 0 {yaw:.4f}</pose>
        <collision name="{name}_Collision">
          <pose>0 0 {height / 2:.3f} 0 0 0</pose>
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
        <visual name="{name}_Visual">
          <pose>0 0 {height / 2:.3f} 0 0 0</pose>
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/White</name>
            </script>
          </material>
        </visual>
      </link>
"""


def main():
    directory, name, world, output = sys.argv[1:5]

    points = read_pcd(os.path.join(directory, f"{name}.pcd"))
    image, origin, resolution = read_map(directory, name)

    # точки стен только в клетках стен арены (с запасом в клетку)
    walls = ndimage.binary_dilation(image == 0, structure=np.ones((3, 3), bool))
    cells = np.floor((points[:, :2] - origin) / resolution).astype(int)
    inside = (
        (cells[:, 0] >= 0) & (cells[:, 0] < image.shape[1])
        & (cells[:, 1] >= 0) & (cells[:, 1] < image.shape[0])
    )
    band = inside & (points[:, 2] > WALL_LOW) & (points[:, 2] < WALL_HIGH)
    band[band] &= walls[cells[band, 1], cells[band, 0]]
    xy = points[band, :2]
    # Высота панелей -- медиана верха по клеткам стен: выше торчат только
    # стойки с поручнями, ниже -- клетки, которые лидар видел вблизи и
    # не доставал до верха
    keys = cells[band, 1] * image.shape[1] + cells[band, 0]
    order = np.argsort(keys)
    tops = np.maximum.reduceat(points[band, 2][order], np.unique(keys[order], return_index=True)[1])
    height = round(float(np.median(tops)), 2)
    # прореживание до сетки 2 см, чтобы плотные места не перевешивали
    _, keep = np.unique(np.floor(xy / 0.02).astype(int), axis=0, return_index=True)
    xy = xy[keep]

    rng = np.random.default_rng(0)
    segments = []
    for direction, center, inliers in merge_lines(fit_lines(xy, rng)):
        angle = snap(direction)
        direction = np.array([math.cos(angle), math.sin(angle)])
        # центр прямой сохраняется, направление доворачивается
        segments += split_segments(direction, center, inliers)

    segments = join_corners(segments)
    links = "".join(
        panel(f"Panel_{i}", start, end, height) for i, (start, end) in enumerate(segments)
    )
    with open(output, "w") as file:
        file.write(WORLD.format(world=world, links=links, model=f"{world}_built"))

    print(f"{len(segments)} панелей высотой {height:.2f} м -> {output}")
    for i, (start, end) in enumerate(segments):
        print(f"  Panel_{i}: ({start[0]:.2f}, {start[1]:.2f}) -> ({end[0]:.2f}, {end[1]:.2f}), "
              f"{np.linalg.norm(end - start):.2f} м")


WORLD = """<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="{world}">
    <!-- Полигон по записи с реального лидара (FAST-LIO2, gen_world.py).
         Начало координат — точка старта робота на записи, он смотрит по +x:
         спавн в (0, 0) повторяет реальный старт. Панели — box-коллизии
         модели {model}, из них же jr_map строит /map. -->

    <physics type='ode'>
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.4</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>1e-05</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic' />
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>

    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <model name='{model}'>
      <static>1</static>
{links}    </model>
  </world>
</sdf>
"""


if __name__ == "__main__":
    main()
