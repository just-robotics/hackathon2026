#!/usr/bin/env python3
"""Карта из выхода FAST-LIO2: облако .pcd и карта занятости .pgm + .yaml.

FAST-LIO отсчитывает мир от начальной позы IMU, а лидар висит вверх ногами.
Облако поворачивается так, чтобы гравитация смотрела в -z (по IMU из
исходного бэга), пол переносится на z = 0, стены ложатся вдоль осей, а робот
на старте смотрит по +x: начало координат -- точка старта.

Карта занятости (формат map_server): стены -- клетки с точками на высоте
OBSTACLE_LOW..OBSTACLE_HIGH, где облако стен тонкое (ниже -- шум пола).
Всё, куда можно доехать от траектории, не пересекая стен, -- свободно,
снаружи -- неизвестно.

    build_map.py <fastlio бэг> <исходный бэг> <каталог> <имя> [x0,y0,x1,y1 ...]

Прямоугольники в конце -- что стереть из карты и облака (координаты карты):
люди и вещи, стоявшие в арене во время записи.
"""

import math
import os
import sys

import numpy as np
from scipy import ndimage
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2

# лидар в base_footprint по URDF: перевёрнут, повёрнут на -90 град
LIDAR_IN_BASE = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
VOXEL = 0.03
RESOLUTION = 0.05
FLOOR_BAND = 0.04
OBSTACLE_LOW = 0.45
OBSTACLE_HIGH = 1.00
MIN_HITS = 3
# щели между панелями уже этого закрываются при заливке арены, клеток
GAP_CELLS = 2
# карта обрезается по траектории с таким запасом, м: дальше -- мебель комнаты
CROP = 0.6


def reader(uri, storage, topics):
    bag = rosbag2_py.SequentialReader()
    bag.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=storage),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    bag.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while bag.has_next():
        yield bag.read_next()


def xyz(message):
    offsets = {field.name: field.offset for field in message.fields}
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4"] * 3,
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": message.point_step,
        }
    )
    cloud = np.frombuffer(message.data, dtype=dtype, count=message.width * message.height)
    return np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1).astype(np.float64)


def voxel_down(points, size):
    keys = np.floor(points / size).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[index]


def rotation_to_up(vector):
    """Поворот, переводящий vector в +z (формула Родрига)"""
    a = vector / np.linalg.norm(vector)
    b = np.array([0.0, 0.0, 1.0])
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * ((1 - c) / np.dot(v, v))


def wall_angle(xy):
    """Угол поворота, после которого стены ложатся вдоль осей

    Перебор углов 0..90: у выровненных стен проекции на оси дают острые
    пики, и сумма квадратов гистограмм максимальна.
    """
    best, best_score = 0.0, -1.0
    for angle in np.radians(np.arange(0.0, 90.0, 0.25)):
        c, s = np.cos(angle), np.sin(angle)
        rotated = xy @ np.array([[c, -s], [s, c]]).T
        score = sum(
            float((np.bincount(np.floor((axis - axis.min()) / 0.02).astype(int)) ** 2).sum())
            for axis in rotated.T
        )
        if score > best_score:
            best, best_score = angle, score
    return best


def write_pcd(path, points):
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    )
    with open(path, "wb") as file:
        file.write(header.encode())
        file.write(points.astype("<f4").tobytes())


def main():
    fastlio, source, directory, name = sys.argv[1:5]
    # прямоугольники для стирания: x0,y0,x1,y1 в координатах карты
    erase = [tuple(map(float, box.split(","))) for box in sys.argv[5:]]
    os.makedirs(directory, exist_ok=True)

    # Куда смотрит «вверх» в начальной системе IMU: акселерометр в покое
    # меряет реакцию опоры, то есть вектор вверх
    accel = []
    for _, data, _ in reader(source, "mcap", ["/livox/imu"]):
        a = deserialize_message(data, Imu).linear_acceleration
        accel.append((a.x, a.y, a.z))
        if len(accel) >= 400:
            break
    up = np.mean(accel, axis=0)
    rotation = rotation_to_up(up)

    chunks = []
    for _, data, _ in reader(fastlio, "sqlite3", ["/cloud_registered"]):
        chunks.append(voxel_down(xyz(deserialize_message(data, PointCloud2)), VOXEL))
    points = voxel_down(np.vstack(chunks), VOXEL) @ rotation.T

    trajectory = []
    for _, data, _ in reader(fastlio, "sqlite3", ["/Odometry"]):
        odometry = deserialize_message(data, Odometry)
        p = odometry.pose.pose.position
        stamp = odometry.header.stamp
        trajectory.append((stamp.sec + stamp.nanosec * 1e-9, p.x, p.y, p.z))
    trajectory = np.array(trajectory)
    track = trajectory[:, 1:] @ rotation.T

    # пол -- самый населённый слой по высоте ниже лидара
    low = points[points[:, 2] < np.median(track[:, 2])]
    histogram, edges = np.histogram(low[:, 2], bins=np.arange(low[:, 2].min(), low[:, 2].max(), 0.01))
    floor = edges[np.argmax(histogram)] + 0.005
    points[:, 2] -= floor
    track[:, 2] -= floor

    # выравнивание стен по осям: угол ищется по стенам рядом с траекторией
    near = np.all(
        (points[:, :2] > track[:, :2].min(0) - CROP) & (points[:, :2] < track[:, :2].max(0) + CROP),
        axis=1,
    )
    walls = points[near & (points[:, 2] > OBSTACLE_LOW) & (points[:, 2] < OBSTACLE_HIGH)]
    yaw = wall_angle(walls[:, :2])
    # Из четырёх углов, при которых стены идут вдоль осей, берётся тот, где
    # робот на старте смотрит по +x: launch_sim спавнит его с нулевым курсом,
    # и симуляция повторяет реальный старт
    forward = rotation @ (LIDAR_IN_BASE.T @ np.array([1.0, 0.0, 0.0]))
    heading = math.atan2(forward[1], forward[0]) + yaw
    yaw -= round(heading / (math.pi / 2)) * (math.pi / 2)
    c, s = np.cos(yaw), np.sin(yaw)
    turn = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    points = points @ turn.T
    track = track @ turn.T

    # как из системы FAST-LIO попасть в систему карты: p @ rotation.T, -floor, @ turn.T
    np.savez(os.path.join(directory, f"{name}_transform.npz"), rotation=rotation, floor=floor, turn=turn)

    # карта занятости
    height = points[:, 2]
    obstacles = points[(height > OBSTACLE_LOW) & (height < OBSTACLE_HIGH)]
    ground = points[np.abs(height) < FLOOR_BAND]

    origin = track[:, :2].min(0) - CROP
    top = track[:, :2].max(0) + CROP
    inside = lambda p: p[np.all((p[:, :2] >= origin) & (p[:, :2] < top), axis=1)]
    obstacles, ground = inside(obstacles), inside(ground)
    width, rows = np.ceil((top - origin) / RESOLUTION).astype(int)

    def cells(xy):
        index = np.floor((xy - origin) / RESOLUTION).astype(int)
        return index[:, 1] * width + index[:, 0]

    hits = np.bincount(cells(obstacles[:, :2]), minlength=width * rows)
    seen = np.bincount(cells(ground[:, :2]), minlength=width * rows)
    image = np.full(width * rows, 205, dtype=np.uint8)
    image[seen > 0] = 254
    image[hits >= MIN_HITS] = 0

    # Стирание вручную: то, что стояло в арене во время записи (люди,
    # вещи), по данным от стены не отличить
    centers = np.stack(np.meshgrid(np.arange(width), np.arange(rows)), axis=-1).reshape(-1, 2)
    centers = origin + (centers + 0.5) * RESOLUTION
    keep = np.ones(len(points), dtype=bool)
    for x0, y0, x1, y1 in erase:
        box = lambda xy: (xy[:, 0] >= x0) & (xy[:, 0] <= x1) & (xy[:, 1] >= y0) & (xy[:, 1] <= y1)
        image[box(centers) & (image == 0)] = 254
        keep &= ~(box(points[:, :2]) & (points[:, 2] > FLOOR_BAND))
    write_pcd(os.path.join(directory, f"{name}.pcd"), points[keep])

    # Арена: всё, куда можно дойти от траектории, не пересекая стен. Стены
    # для заливки утолщены на GAP_CELLS, чтобы закрыть щели между панелями,
    # потом заливка расширяется обратно до самих стен. Внутри -- свободно,
    # снаружи остаются только стены арены, остальное -- неизвестно.
    grid = image.reshape(rows, width)
    # квадратное расширение: ромбом (по умолчанию) скругляются углы арены
    square = np.ones((3, 3), dtype=bool)
    # щели между панелями закрываются, контур стен становится замкнутым
    wall = ndimage.binary_closing(grid == 0, structure=square, iterations=GAP_CELLS) | (grid == 0)
    barrier = ndimage.binary_dilation(wall, structure=square, iterations=GAP_CELLS)
    regions, _ = ndimage.label(~barrier)
    path = np.floor((track[:, :2] - origin) / RESOLUTION).astype(int)
    labels = set(regions[path[:, 1], path[:, 0]].tolist()) - {0}
    arena = ndimage.binary_dilation(
        np.isin(regions, list(labels)), structure=square, iterations=GAP_CELLS
    ) & ~wall
    walls = wall & ndimage.binary_dilation(arena, structure=square, iterations=GAP_CELLS + 1)
    grid[:] = 205
    grid[arena] = 254
    grid[walls] = 0
    image = grid.reshape(-1)
    image = image.reshape(rows, width)[::-1]  # строки pgm идут сверху вниз

    with open(os.path.join(directory, f"{name}.pgm"), "wb") as file:
        file.write(f"P5\n{width} {rows}\n255\n".encode())
        file.write(image.tobytes())
    with open(os.path.join(directory, f"{name}.yaml"), "w") as file:
        file.write(
            f"image: {name}.pgm\nmode: trinary\nresolution: {RESOLUTION}\n"
            f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )
    np.savetxt(os.path.join(directory, f"{name}_trajectory.txt"), np.column_stack([trajectory[:, 0], track]), fmt="%.4f", header="t x y z (карта)")

    print(f"стены повёрнуты на {np.degrees(yaw):.2f} град, курс робота на старте "
          f"{np.degrees(heading - round(heading / (math.pi / 2)) * (math.pi / 2)):.1f} град")
    print(f"вверх по IMU {np.round(up, 3)}, пол на {floor:.3f} м от начала FAST-LIO")
    print(f"облако {len(points)} точек, карта {width}x{rows} ячеек по {RESOLUTION} м, "
          f"{(top - origin).round(2)} м; занято {int((image == 0).sum())}, свободно {int((image == 254).sum())}")
    print(f"лидар над полом: {np.median(track[:, 2]):.3f} м")
    if erase:
        print(f"стёрто прямоугольников: {len(erase)}, точек облака: {int((~keep).sum())}")


if __name__ == "__main__":
    main()
