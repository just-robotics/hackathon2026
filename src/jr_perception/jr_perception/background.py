"""Фон, записанный лидаром: занятые ячейки на плоскости и плоскость пола.

Для сцен, которых нет в .world: комната, реальный полигон. Файл пишет
record_background.py по бэгу с неподвижного лидара, читает robot_detector.py
(параметр background_file).

Ячейка считается фоном, если выше пола в ней были точки хотя бы в доле share
сканов. У Mid-360 неповторяющийся паттерн: редкий дальний предмет попадает не
в каждый скан, поэтому доля нужна небольшая. Соседние ячейки тоже
добавляются в фон -- на шум и край паттерна.
"""

import numpy as np


# ключ ячейки -- одно целое: ix * KEY_STRIDE + iy, iy по модулю меньше
# половины шага, поэтому ключи не пересекаются
KEY_STRIDE = 1_000_003


def cell_keys(xy: np.ndarray, cell: float) -> np.ndarray:
    """Ключи ячеек сетки для точек (N, 2)"""
    indices = np.floor(xy / cell).astype(np.int64)
    return indices[:, 0] * KEY_STRIDE + indices[:, 1]


def floor_heights(points: np.ndarray, plane: np.ndarray) -> np.ndarray:
    """Высоты точек (N, 3) над плоскостью пола z = a x + b y + c"""
    a, b, c = plane
    return points[:, 2] - (a * points[:, 0] + b * points[:, 1] + c)


def fit_floor(points: np.ndarray, near: float = 0.5, far: float = 4.0) -> tuple:
    """Плоскость пола по точкам (N, 3) в системе с осью z вверх

    Пол -- нижний плотный слой точек на дальности near..far от начала
    координат. Сначала он берётся слоем 5 см у 5-го перцентиля высоты, потом
    плоскость уточняется МНК с отбросом точек дальше трёх СКО.

    :return (plane, std): коэффициенты (a, b, c) и СКО точек пола, м
    """
    distance = np.hypot(points[:, 0], points[:, 1])
    ring = points[(distance > near) & (distance < far)]
    level = np.percentile(ring[:, 2], 5.0)
    floor = ring[np.abs(ring[:, 2] - level) < 0.05]

    plane = np.array([0.0, 0.0, level])
    std = 0.0
    for _ in range(3):
        design = np.c_[floor[:, 0], floor[:, 1], np.ones(len(floor))]
        plane, *_ = np.linalg.lstsq(design, floor[:, 2], rcond=None)
        residual = floor[:, 2] - design @ plane
        std = float(residual.std())
        floor = floor[np.abs(residual) < 3.0 * std + 1e-3]

    return plane, std


def occupied_cells(
    scans: list, cell: float, share: float, min_height: float, plane: np.ndarray
) -> np.ndarray:
    """Ячейки фона по набору сканов

    :scans список массивов (N, 3) в одной системе
    :cell шаг сетки, м
    :share доля сканов, в которой ячейка должна быть занята
    :min_height точки ниже этой высоты над полом -- пол
    :plane плоскость пола

    :return отсортированные ключи ячеек фона вместе с соседями
    """
    counts = {}
    for points in scans:
        above = points[floor_heights(points, plane) > min_height]
        for key in np.unique(cell_keys(above[:, :2], cell)):
            counts[key] = counts.get(key, 0) + 1

    static = np.array(
        [key for key, count in counts.items() if count >= share * len(scans)],
        dtype=np.int64,
    )
    offsets = np.array(
        [dx * KEY_STRIDE + dy for dx in (-1, 0, 1) for dy in (-1, 0, 1)],
        dtype=np.int64,
    )
    return np.unique((static[:, None] + offsets[None, :]).reshape(-1))


def save(path: str, keys: np.ndarray, cell: float, plane: np.ndarray, frame: str):
    """Сохранить фон в .npz"""
    np.savez(path, keys=keys, cell=cell, plane=plane, frame=frame)


def load(path: str) -> dict:
    """Прочитать фон из .npz

    :return {keys, cell, plane, frame}
    """
    data = np.load(path)
    return {
        "keys": data["keys"],
        "cell": float(data["cell"]),
        "plane": data["plane"],
        "frame": str(data["frame"]),
    }
