"""Фон, записанный лидаром: занятые ячейки на плоскости и плоскость пола.

Для сцен, которых нет в .world: комната, реальный полигон. Файл пишет
record_background.py по бэгу с неподвижного лидара, читает robot_detector.py
(параметр background_file). Для едущего робота тот же фон строится в
рантайме из карты занятости (from_occupancy, параметр map_topic).

Ячейка считается фоном, если выше пола в ней были точки хотя бы в доле share
сканов. У Mid-360 неповторяющийся паттерн: редкий дальний предмет попадает не
в каждый скан, поэтому доля нужна небольшая. Соседние ячейки тоже
добавляются в фон -- на шум и край паттерна.

Вторая проверка -- по времени: бэг делится на segments равных отрезков, и
ячейка должна быть занята хотя бы в min_segments из них. Мебель стоит весь
бэг, а робот, задержавшийся на несколько секунд, занимает ячейки в одном-двух
отрезках. Без этой проверки его след у лидара записывался в фон и потом
съедал подъехавшего робота.
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
    scans: list,
    cell: float,
    share: float,
    min_height: float,
    plane: np.ndarray,
    segments: int = 10,
    min_segments: int = 7,
) -> tuple:
    """Ячейки фона по набору сканов и высота верха предмета в каждой

    Верх ячейки -- медиана по отрезкам бэга от наибольшей высоты точек в
    отрезке: робот, проехавший над ячейкой, попадает в один-два отрезка и
    верх не завышает. Соседняя ячейка получает наибольший верх из
    добавивших её.

    :scans список массивов (N, 3) в одной системе, по времени
    :cell шаг сетки, м
    :share доля сканов, в которой ячейка должна быть занята
    :min_height точки ниже этой высоты над полом -- пол
    :plane плоскость пола
    :segments на сколько равных отрезков делится запись
    :min_segments в скольких отрезках ячейка должна быть занята

    :return (keys, tops): отсортированные ключи ячеек фона вместе с
    соседями и высота верха предмета над полом в каждой, м
    """
    counts = {}
    segment_tops = {}
    for index, points in enumerate(scans):
        segment = index * segments // len(scans)
        heights = floor_heights(points, plane)
        above = heights > min_height
        keys, inverse = np.unique(
            cell_keys(points[above, :2], cell), return_inverse=True
        )
        tops = np.full(len(keys), -np.inf)
        np.maximum.at(tops, inverse.reshape(-1), heights[above])
        for key, top in zip(keys.tolist(), tops.tolist()):
            counts[key] = counts.get(key, 0) + 1
            per_segment = segment_tops.setdefault(key, {})
            per_segment[segment] = max(per_segment.get(segment, -np.inf), top)

    grown = {}
    for key, count in counts.items():
        per_segment = segment_tops[key]
        if count < share * len(scans) or len(per_segment) < min_segments:
            continue
        top = float(np.median(list(per_segment.values())))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                neighbour = key + dx * KEY_STRIDE + dy
                grown[neighbour] = max(grown.get(neighbour, -np.inf), top)

    keys = np.array(sorted(grown), dtype=np.int64)
    return keys, np.array([grown[key] for key in keys.tolist()])


def covered(
    xy: np.ndarray, heights: np.ndarray, grid: dict, margin: float
) -> np.ndarray:
    """Какие точки объясняются фоном

    Точка -- фон, если её ячейка занята и сама она не выше верха предмета в
    ячейке больше чем на margin. Робот рядом с низкой ногой стойки сохраняет
    всё, что выше ноги.

    :xy (N, 2), :heights (N,) высоты над полом
    :grid фон из load
    :margin запас по высоте над верхом предмета, м

    :return булев массив (N,)
    """
    keys = cell_keys(xy, grid["cell"])
    index = np.clip(np.searchsorted(grid["keys"], keys), 0, len(grid["keys"]) - 1)
    inside = grid["keys"][index] == keys
    return inside & (heights <= grid["tops"][index] + margin)


def _flood(free: np.ndarray, row: int, col: int) -> np.ndarray:
    """Свободные клетки, связанные с (row, col) по сторонам"""
    reached = np.zeros_like(free)
    stack = [(row, col)]
    while stack:
        r, c = stack.pop()
        if 0 <= r < free.shape[0] and 0 <= c < free.shape[1] and free[r, c] and not reached[r, c]:
            reached[r, c] = True
            stack += [(r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)]
    return reached


def from_occupancy(
    values: np.ndarray,
    resolution: float,
    origin: tuple,
    seed: np.ndarray,
    dilate: int,
    pad: int,
    frame: str,
) -> tuple:
    """Фон из карты занятости: всё, кроме арены, где стоит робот

    Арена -- свободные клетки, связанные с клеткой seed. Остальное -- стены,
    неизвестные клетки, свободные пятна за стенами и pad клеток за краем
    карты -- фон на любой высоте. Фон наращивается на dilate клеток внутрь
    арены: на неточность карты и локализации.

    :values (H, W) значения nav_msgs/OccupancyGrid, строка 0 -- у origin:
    -1 неизвестно, 0..100 занятость
    :resolution шаг карты, м
    :origin (x, y) угла карты, поворот не поддерживается
    :seed (2,) точка внутри арены, обычно положение робота
    :dilate запас вокруг стен, клеток
    :pad фон за краем карты, клеток
    :frame фрейм карты

    :return (grid, found): фон в формате load и нашлась ли арена. Если seed
    не в свободной клетке, ареной считаются все свободные клетки
    """
    free = (values >= 0) & (values < 25)
    row = int(np.floor((seed[1] - origin[1]) / resolution))
    col = int(np.floor((seed[0] - origin[0]) / resolution))
    found = 0 <= row < free.shape[0] and 0 <= col < free.shape[1] and bool(free[row, col])
    arena = _flood(free, row, col) if found else free

    taken = np.pad(~arena, pad, constant_values=True)
    for _ in range(dilate):
        grown = taken.copy()
        grown[1:] |= taken[:-1]
        grown[:-1] |= taken[1:]
        grown[:, 1:] |= taken[:, :-1]
        grown[:, :-1] |= taken[:, 1:]
        taken = grown

    # центры клеток карты -- в ключи общей сетки фона с тем же шагом
    rows, cols = np.nonzero(taken)
    centers = np.c_[
        origin[0] + (cols - pad + 0.5) * resolution,
        origin[1] + (rows - pad + 0.5) * resolution,
    ]
    keys = np.unique(cell_keys(centers, resolution))
    grid = {
        "keys": keys,
        "tops": np.full(len(keys), np.inf),
        "cell": resolution,
        "plane": np.zeros(3),
        "frame": frame,
    }
    return grid, found


def save(
    path: str,
    keys: np.ndarray,
    tops: np.ndarray,
    cell: float,
    plane: np.ndarray,
    frame: str,
):
    """Сохранить фон в .npz"""
    np.savez(path, keys=keys, tops=tops, cell=cell, plane=plane, frame=frame)


def load(path: str) -> dict:
    """Прочитать фон из .npz

    :return {keys, tops, cell, plane, frame}
    """
    data = np.load(path)
    if "tops" not in data:
        raise ValueError(
            f"{path} записан старой версией record_background.py, без высоты "
            "предметов в ячейках: перезапишите фон"
        )
    return {
        "keys": data["keys"],
        "tops": data["tops"],
        "cell": float(data["cell"]),
        "plane": data["plane"],
        "frame": str(data["frame"]),
    }
