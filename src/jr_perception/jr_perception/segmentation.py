"""Поиск Kobuki в облаке лидара.

Чистые функции без ROS: numpy на входе, numpy на выходе. Точки везде в
системе мира, массив (N, 3).

Каким соседний Kobuki попадает в облако. Штатный ray-сенсор Gazebo Classic
трассирует лучи по collision-геометрии, поэтому от робота видны только
цилиндр корпуса R=0.178 (z 0.015-0.124) и три пластины толщиной 6 мм
R=0.170 (z около 0.14, 0.20 и 0.41). Стойки и сам лидар заданы только как
visual, лучи проходят сквозь них. Корпус и пластины -- тела вращения, поэтому
по одному скану курс робота не определить: здесь ищется только центр, курс
восстанавливает трекер по движению.
"""

import math
from dataclasses import dataclass

import numpy as np


def arena_bounds(boxes: list) -> tuple:
    """Габарит арены по фоновым боксам

    :boxes список (cx, cy, half_x, half_y, yaw), как у jr_map.collect_boxes

    :return (min_x, min_y, max_x, max_y)
    """
    xs, ys = [], []
    for cx, cy, hx, hy, yaw in boxes:
        cos, sin = math.cos(yaw), math.sin(yaw)
        for sx, sy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            xs.append(cx + sx * cos - sy * sin)
            ys.append(cy + sx * sin + sy * cos)

    return min(xs), min(ys), max(xs), max(ys)


def foreground_mask(
    points: np.ndarray,
    sensor: np.ndarray,
    boxes: list,
    bounds: tuple,
    wall_margin: float,
    floor_z: float,
    floor_noise: float,
    ceiling_z: float,
) -> np.ndarray:
    """Отметить точки, которые не объясняются пустым полигоном

    Фон -- пол, всё выше ceiling_z, всё за пределами арены и всё ближе
    wall_margin к фоновым боксам. Бокс проверяется в собственной системе,
    поэтому поворот учитывается точно.

    Шум дальности сдвигает точку вдоль луча, и по высоте точка пола гуляет на
    шум * sin наклона луча: у самого наблюдателя это сантиметры, вдали --
    миллиметры. Порог пола растёт вместе с этим разбросом, и вдали остаётся
    видна нижняя кромка корпуса соседа, которая начинается с z=0.015.

    :points (N, 3) точки в системе мира
    :sensor (3,) положение лидара в системе мира
    :boxes фоновые боксы (cx, cy, half_x, half_y, yaw)
    :bounds габарит арены (min_x, min_y, max_x, max_y)
    :wall_margin запас вокруг боксов, м
    :floor_z порог пола для пологих лучей, м
    :floor_noise добавка к порогу пола для отвесного луча, м
    :ceiling_z точки выше отбрасываются, м

    :return булев массив (N,), True -- точка переднего плана
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    distance = np.maximum(np.linalg.norm(points - sensor, axis=1), 1e-6)
    slope = np.clip((sensor[2] - z) / distance, 0.0, 1.0)
    keep = (z >= floor_z + floor_noise * slope) & (z <= ceiling_z)

    min_x, min_y, max_x, max_y = bounds
    keep &= (x > min_x) & (x < max_x) & (y > min_y) & (y < max_y)

    for cx, cy, hx, hy, yaw in boxes:
        cos, sin = math.cos(yaw), math.sin(yaw)
        dx = x - cx
        dy = y - cy
        local_x = dx * cos + dy * sin
        local_y = -dx * sin + dy * cos
        keep &= ~(
            (np.abs(local_x) <= hx + wall_margin)
            & (np.abs(local_y) <= hy + wall_margin)
        )

    return keep


def cluster_xy(xy: np.ndarray, cell: float) -> tuple:
    """Разбить точки на кластеры по связности ячеек сетки

    Точки раскладываются в сетку с шагом cell, занятые ячейки, касающиеся
    друг друга хотя бы углом, объединяются. Точки ближе cell всегда попадают в
    один кластер, зазор в пару ячеек кластеры разделяет.

    :xy (N, 2)
    :cell шаг сетки, м

    :return (labels, count): номер кластера каждой точки и число кластеров
    """
    if len(xy) == 0:
        return np.zeros(0, dtype=np.int64), 0

    cells = np.floor(xy / cell).astype(np.int64)
    # Ячейка кодируется одним целым, чтобы np.unique работал с одномерным
    # массивом, а соседи находились сложением. Сдвиг на единицу оставляет
    # соседей крайних ячеек внутри строки, и ключи не перескакивают на
    # соседнюю строку.
    cells -= cells.min(axis=0) - 1
    width = int(cells[:, 1].max()) + 2
    keys = cells[:, 0] * width + cells[:, 1]
    unique, inverse = np.unique(keys, return_inverse=True)

    index = {int(key): i for i, key in enumerate(unique)}
    offsets = [
        row * width + col
        for row in (-1, 0, 1)
        for col in (-1, 0, 1)
        if row or col
    ]

    cell_label = np.full(len(unique), -1, dtype=np.int64)
    count = 0
    for start in range(len(unique)):
        if cell_label[start] >= 0:
            continue

        cell_label[start] = count
        stack = [start]
        while stack:
            key = int(unique[stack.pop()])
            for offset in offsets:
                neighbour = index.get(key + offset)
                if neighbour is not None and cell_label[neighbour] < 0:
                    cell_label[neighbour] = count
                    stack.append(neighbour)

        count += 1

    return cell_label[inverse.reshape(-1)], count


def split_clusters(points: np.ndarray, labels: np.ndarray, count: int) -> list:
    """Разложить точки по кластерам

    :points (N, 3)
    :labels номер кластера каждой точки
    :count число кластеров

    :return список массивов (M, 3), по одному на кластер
    """
    if count == 0:
        return []

    order = np.argsort(labels, kind="stable")
    sizes = np.bincount(labels, minlength=count)
    return np.split(points[order], np.cumsum(sizes)[:-1])


def fit_circle(
    xy: np.ndarray, radius: float, center: np.ndarray, iterations: int = 10
) -> tuple:
    """Подогнать окружность известного радиуса

    Гаусс-Ньютон по двум параметрам, координатам центра: минимизируется
    сумма (|p - c| - radius)^2.

    :xy (N, 2) точки на окружности
    :radius радиус, м
    :center начальное приближение центра

    :return (center, residuals): центр и невязки |p - c| - radius по точкам
    """
    center = np.array(center, dtype=float)

    for _ in range(iterations):
        offset = xy - center
        distance = np.maximum(np.linalg.norm(offset, axis=1), 1e-9)
        unit = offset / distance[:, None]
        # якобиан невязок по центру равен -unit, поэтому шаг
        # -(J^T J)^-1 J^T r превращается в (U^T U)^-1 U^T r
        step = np.linalg.solve(
            unit.T @ unit + 1e-9 * np.eye(2), unit.T @ (distance - radius)
        )
        length = float(np.linalg.norm(step))
        if length > radius:
            step *= radius / length
        center += step
        if length < 1e-4:
            break

    return center, np.linalg.norm(xy - center, axis=1) - radius


def edge_center(
    xy: np.ndarray, rim: np.ndarray, observer: np.ndarray, radius: float
) -> np.ndarray:
    """Центр робота по краям облака и известному радиусу

    Считается в системе наблюдателя. Направление на центр -- середина между
    крайними по азимуту точками: это касательные к корпусу, а шаг лучей
    сдвигает обе внутрь одинаково, так что середина не смещается. Хорда по
    верхней грани или пластине тоже симметрична относительно этого
    направления.

    Дальность до центра: точка обода, отстоящая на c поперёк направления,
    лежит на a вдоль него, и центр дальше неё на sqrt(R^2 - c^2). Медиана по
    всем точкам обода вместо одной ближайшей: у ближайшей весь шум дальности
    и смещение к наблюдателю. Без обода остаётся дальность хорды, центр от
    неё может быть и ближе, и дальше на величину до радиуса.

    :xy (N, 2) все точки кластера
    :rim (M, 2) точки обода, может быть пустым
    :observer (2,) положение лидара наблюдателя
    :radius радиус корпуса, м

    :return (2,) центр
    """
    offset = xy - observer
    base = math.atan2(*offset.mean(axis=0)[::-1])
    angles = (np.arctan2(offset[:, 1], offset[:, 0]) - base + math.pi) % (
        2.0 * math.pi
    ) - math.pi
    heading = base + (angles.min() + angles.max()) / 2.0
    direction = np.array([math.cos(heading), math.sin(heading)])

    source = rim if len(rim) else xy
    offset = source - observer
    along = offset @ direction
    if len(rim):
        across = offset @ np.array([-direction[1], direction[0]])
        along = along + np.sqrt(np.clip(radius ** 2 - across ** 2, 0.0, None))

    return observer + float(np.median(along)) * direction


def principal_axes(xy: np.ndarray) -> tuple:
    """Главные оси облака точек на плоскости

    :xy (N, 2)

    :return (eigenvalues, vectors, centered): собственные числа ковариации
    по возрастанию, векторы столбцами и точки за вычетом центроида
    """
    centered = xy - xy.mean(axis=0)
    eigenvalues, vectors = np.linalg.eigh(centered.T @ centered / len(xy))
    return eigenvalues, vectors, centered


def major_extent(xy: np.ndarray) -> float:
    """Размах точек вдоль главной оси

    Берётся между 2-м и 98-м перцентилями, чтобы единичные выбросы шума не
    раздували габарит.

    :xy (N, 2)

    :return размах, м
    """
    _, vectors, centered = principal_axes(xy)
    low, high = np.percentile(centered @ vectors[:, -1], [2.0, 98.0])
    return float(high - low)


def line_rms(xy: np.ndarray) -> float:
    """СКО отклонения точек от прямой, проведённой через них по МНК

    :xy (N, 2)

    :return СКО поперёк прямой, м
    """
    eigenvalues, _, _ = principal_axes(xy)
    return math.sqrt(max(float(eigenvalues[0]), 0.0))


@dataclass
class RobotModel:
    """Как Kobuki выглядит в облаке и когда кластер считать им"""

    # радиус цилиндра корпуса, м
    radius: float = 0.178
    # обод -- точки ниже верхней грани корпуса (0.124): только они лежат на
    # окружности, попадания в грань и пластины разбросаны по кругу
    rim_max_z: float = 0.115
    # выше верхней пластины (0.41) у робота ничего нет, а коробку лидар видит
    # почти до её верха 0.5
    max_height: float = 0.43
    # размах вдоль главной оси: диаметр 0.356 плюс шум; грань коробки 0.5
    max_extent: float = 0.42
    min_points: int = 3
    # сколько точек обода нужно для фита окружности
    min_rim_points: int = 4
    # точки обода дальше от окружности -- выбросы
    outlier: float = 0.05
    # предельная невязка фита окружности, м
    fit_rms_max: float = 0.035
    # окружность должна описывать обод заметно лучше прямой: прямой кусок
    # грани коробки описывается прямой не хуже
    line_ratio: float = 0.7
    # СКО центра: по фиту окружности и по центроиду без обода, м
    measurement_std: float = 0.03
    fallback_std: float = 0.10


@dataclass
class Detection:
    """Кластер, похожий на Kobuki"""

    center: np.ndarray
    sigma: float
    # центр найден фитом окружности по ободу; только такие детекции
    # открывают новые треки
    strong: bool


def detect_robot(
    points: np.ndarray, observer: np.ndarray, model: RobotModel
) -> "Detection | None":
    """Проверить, похож ли кластер на Kobuki, и найти его центр

    :points (N, 3) точки одного кластера в системе мира
    :observer (2,) положение лидара наблюдателя на плоскости
    :model параметры робота

    :return Detection или None, если кластер на робота не похож
    """
    if len(points) < model.min_points:
        return None

    if points[:, 2].max() > model.max_height:
        return None

    xy = points[:, :2]
    if major_extent(xy) > model.max_extent:
        return None

    rim = xy[points[:, 2] <= model.rim_max_z]
    guess = edge_center(xy, rim, observer, model.radius)

    if len(rim) < model.min_rim_points:
        # Для фита обода мало: вдали кольцо проходит над кромкой корпуса и
        # ложится на верхнюю грань или пластину. Направление на центр по
        # краям остаётся точным, а дальность -- с точностью до радиуса,
        # отсюда большая СКО.
        return Detection(guess, model.fallback_std, strong=False)

    center, residual = fit_circle(rim, model.radius, guess)
    inliers = np.abs(residual) <= model.outlier
    if inliers.sum() < model.min_rim_points:
        return None

    arc = rim[inliers]
    if not inliers.all():
        center, residual = fit_circle(arc, model.radius, center)

    rms = math.sqrt(float(np.mean(residual ** 2)))
    if rms > model.fit_rms_max or rms > model.line_ratio * line_rms(arc):
        return None

    return Detection(center, model.measurement_std, strong=True)
