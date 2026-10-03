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
    :bounds габарит арены (min_x, min_y, max_x, max_y); None -- без обрезки
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

    if bounds is not None:
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
    # Верх кластера должен быть не ниже этого, м; 0 -- проверка выключена.
    # У реального Kobuki лидар видит стойки и пластины до ~0.4 м, а шум у
    # пола поднимается на пару сантиметров. В симуляции стойки прозрачны, и
    # вдали робот виден одним нижним поясом корпуса, поэтому там выключено.
    min_top: float = 0.0
    # размах вдоль главной оси: диаметр 0.356 плюс шум; грань коробки 0.5
    max_extent: float = 0.42
    # Видимая дуга корпуса не уже этого, м; 0 -- выключено. У реального
    # робота 0.28-0.34 на 0.5-3 м, а узкие высокие предметы (ножки, стойки)
    # дают 0.1-0.2.
    min_extent: float = 0.0
    # Пробел между нижними пластинами и верхней: у реального Kobuki там
    # только тонкие стойки. Доля точек кластера в этом слое не больше
    # max_gap_share; 1 -- проверка выключена.
    gap_min_z: float = 0.25
    gap_max_z: float = 0.34
    max_gap_share: float = 1.0
    # Если кластер шире робота (робот прижался к стене или предмету), в нём
    # ищется окружность радиуса radius, и дальше проверяются только точки не
    # дальше radius + contain_margin от её центра.
    contain_margin: float = 0.05
    # Грани против окружности: у коробки и стены точки лежат на вертикальных
    # гранях -- сторонах прямоугольника на плоскости, у робота -- на дуге.
    # Кластер -- предмет, если невязка граней меньше plane_ratio невязки
    # окружности корпуса (обе по всем точкам кластера). На синтетике с шумом
    # дальности 1-2 см отношение у робота не меньше 1.85, у коробок
    # 15x15x40 и 40x60x20 не больше 1.14 (95%). 0 -- проверка выключена.
    plane_ratio: float = 1.5
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


def find_circle(
    xy: np.ndarray,
    observer: np.ndarray,
    radius: float,
    tolerance: float,
    iterations: int = 200,
) -> tuple:
    """Окружность известного радиуса с наибольшим числом точек (RANSAC)

    Две точки на окружности задают два возможных центра; берётся тот, что
    дальше от наблюдателя: лидар видит ближнюю к себе половину корпуса.

    :xy (N, 2)
    :observer (2,) положение лидара наблюдателя
    :radius радиус, м
    :tolerance точка ближе этого к окружности -- на ней, м
    :iterations число пар точек

    :return (center, inliers) или (None, 0)
    """
    if len(xy) < 3:
        return None, 0

    # фиксированное зерно: одно и то же облако -- один и тот же ответ
    rng = np.random.default_rng(len(xy))
    first = xy[rng.integers(len(xy), size=iterations)]
    second = xy[rng.integers(len(xy), size=iterations)]
    chord = second - first
    half = np.linalg.norm(chord, axis=1) / 2.0
    usable = (half > 0.02) & (half < radius)
    if not usable.any():
        return None, 0

    first, chord, half = first[usable], chord[usable], half[usable]
    middle = first + chord / 2.0
    normal = np.stack([-chord[:, 1], chord[:, 0]], axis=1) / (2.0 * half[:, None])
    offset = np.sqrt(radius ** 2 - half ** 2)[:, None]
    near, far = middle - normal * offset, middle + normal * offset
    choose_far = np.linalg.norm(far - observer, axis=1) >= np.linalg.norm(
        near - observer, axis=1
    )
    centers = np.where(choose_far[:, None], far, near)

    distance = np.linalg.norm(xy[None, :, :] - centers[:, None, :], axis=2)
    counts = (np.abs(distance - radius) <= tolerance).sum(axis=1)
    best = int(np.argmax(counts))
    return centers[best], int(counts[best])


def plane_test(xy: np.ndarray, guess: np.ndarray, model: RobotModel) -> tuple:
    """Лежат ли точки на вертикальных гранях лучше, чем на корпусе робота

    :xy (N, 2) точки кластера
    :guess (2,) начальный центр окружности корпуса
    :model radius и plane_ratio

    :return (planar, faces, circle): предмет ли это, невязки граней и
    окружности радиуса radius, м
    """
    faces = rectangle_fit(xy)[3]
    _, residual = fit_circle(xy, model.radius, guess)
    circle = math.sqrt(float(np.mean(residual ** 2)))
    return faces < model.plane_ratio * circle, faces, circle


def inspect_cluster(
    points: np.ndarray, observer: np.ndarray, model: RobotModel, extract: bool = True
) -> tuple:
    """Проверить, похож ли кластер на Kobuki, и найти его центр

    :points (N, 3) точки одного кластера, z -- высота над полом
    :observer (2,) положение лидара наблюдателя на плоскости
    :model параметры робота
    :extract искать робота внутри слишком широкого кластера

    :return (Detection или None, причина отказа или "")
    """
    if len(points) < model.min_points:
        return None, f"мало точек: {len(points)}"

    xy = points[:, :2]
    extent = major_extent(xy)
    if extent > model.max_extent:
        if not extract:
            return None, f"широкий: {extent:.2f}"

        # Робот прижался к стене или предмету и слился с ним: ищем в
        # кластере окружность корпуса и проверяем только точки вокруг неё.
        rim = xy[points[:, 2] <= model.rim_max_z]
        center, count = find_circle(rim, observer, model.radius, model.outlier / 2.0)
        if center is None or count < model.min_rim_points:
            return None, f"широкий: {extent:.2f}, круга корпуса нет"
        inside = np.linalg.norm(xy - center, axis=1) <= model.radius + model.contain_margin
        detection, reason = inspect_cluster(points[inside], observer, model, extract=False)
        return detection, reason and f"широкий, внутри: {reason}"

    top = points[:, 2].max()
    if top > model.max_height:
        return None, f"высокий: {top:.2f}"
    if top < model.min_top:
        return None, f"низкий: {top:.2f}"
    if extent < model.min_extent:
        return None, f"узкий: {extent:.2f}"

    heights = points[:, 2]
    gap = float(np.mean((heights >= model.gap_min_z) & (heights < model.gap_max_z)))
    if gap > model.max_gap_share:
        return None, f"нет пробела: {100 * gap:.0f}%"

    rim = xy[heights <= model.rim_max_z]
    guess = edge_center(xy, rim, observer, model.radius)

    if model.plane_ratio > 0.0:
        planar, faces, circle = plane_test(xy, guess, model)
        if planar:
            return None, f"плоскость: {100 * faces:.1f} против окружности {100 * circle:.1f} см"

    if len(rim) < model.min_rim_points:
        # Для фита обода мало: вдали кольцо проходит над кромкой корпуса и
        # ложится на верхнюю грань или пластину. Направление на центр по
        # краям остаётся точным, а дальность -- с точностью до радиуса,
        # отсюда большая СКО.
        return Detection(guess, model.fallback_std, strong=False), ""

    center, residual = fit_circle(rim, model.radius, guess)
    inliers = np.abs(residual) <= model.outlier
    if inliers.sum() < model.min_rim_points:
        return None, f"не окружность: на ней {int(inliers.sum())} из {len(rim)}"

    arc = rim[inliers]
    if not inliers.all():
        center, residual = fit_circle(arc, model.radius, center)

    rms = math.sqrt(float(np.mean(residual ** 2)))
    if rms > model.fit_rms_max:
        return None, f"не окружность: невязка {100 * rms:.1f} см"

    straight = line_rms(arc)
    if rms > model.line_ratio * straight:
        # Короткую дугу от прямой по форме не отличить: так выглядит робот,
        # у которого часть корпуса закрыта или съедена фоном рядом с
        # предметом, а вблизи -- и сам реальный Kobuki, бок которого не
        # идеально круглый (rot_0.5m: дуга 116 градусов вместо 155). Если по
        # высоте он проходит, это слабая детекция: трек она не откроет, но
        # уже идущий продлит. Центр по краям с большой СКО: центр фита
        # короткой дуги гуляет с поворотом робота, и вблизи с ним трек
        # дрожал сильнее.
        if model.min_top > 0.0 or model.max_gap_share < 1.0:
            return Detection(guess, model.fallback_std, strong=False), ""
        return None, f"прямая: {100 * rms:.1f} против {100 * straight:.1f} см"

    return Detection(center, model.measurement_std, strong=True), ""


def detect_robot(
    points: np.ndarray, observer: np.ndarray, model: RobotModel
) -> "Detection | None":
    """То же, что inspect_cluster, без причины отказа"""
    return inspect_cluster(points, observer, model)[0]


@dataclass
class Box:
    """Предмет на сцене: повёрнутый прямоугольник вокруг его точек"""

    # центр на плоскости, м
    center: np.ndarray
    # длина вдоль yaw и ширина поперёк, м
    size: np.ndarray
    # поворот длинной стороны, рад
    yaw: float
    # высота верха над полом, м
    top: float
    points: int
    # точки лежат на вертикальных гранях лучше, чем на корпусе робота
    # (plane_test): только такие идут в трекер коробок
    planar: bool = False
    # невязка граней, м
    faces_rms: float = 0.0


def convex_hull(xy: np.ndarray) -> np.ndarray:
    """Выпуклая оболочка точек (N, 2), обход Эндрю

    :return вершины оболочки против часовой стрелки, (M, 2)
    """
    unique = np.unique(xy, axis=0)
    if len(unique) < 3:
        return unique

    def half(points):
        chain = []
        for point in points:
            while len(chain) >= 2:
                (ax, ay), (bx, by) = chain[-2], chain[-1]
                if (bx - ax) * (point[1] - ay) - (by - ay) * (point[0] - ax) > 0.0:
                    break
                chain.pop()
            chain.append(point)
        return chain[:-1]

    # np.unique уже отсортировал точки по x, потом по y
    points = unique.tolist()
    return np.array(half(points) + half(points[::-1]))


def oriented_rectangle(xy: np.ndarray) -> tuple:
    """Повёрнутый прямоугольник вокруг точек (N, 2), стороны -- по граням

    :return (center, size, yaw), см. rectangle_fit
    """
    return rectangle_fit(xy)[:3]


def rectangle_fit(xy: np.ndarray) -> tuple:
    """Повёрнутый прямоугольник вокруг точек (N, 2), стороны -- по граням

    Направления-кандидаты -- стороны выпуклой оболочки. Лидар видит у коробки
    одну-две грани, и берётся направление, при котором точки в среднем ближе
    всего к сторонам прямоугольника: грани ложатся на стороны. Наименьшая
    площадь тут не годится: оболочка двух граней -- прямоугольный
    треугольник, и рамка по гипотенузе у него той же площади, что по
    катетам.

    Невязка -- СКО расстояния точек до ближайшей стороны. У вертикальных
    граней (коробка, стена) это шум дальности: точки одной-двух граней лежат
    на сторонах. У дуги корпуса робота точки между касаниями отходят от
    сторон на сантиметры. Стороны для невязки -- по 2-му и 98-му перцентилям,
    чтобы одиночный выброс не отодвигал сторону от грани.

    :return (center, size, yaw, rms): центр (2,), длина вдоль yaw и ширина,
    yaw, невязка, м
    """
    hull = convex_hull(xy)
    if len(hull) < 3:
        # все точки на одной прямой: прямоугольник вырождается в отрезок
        if len(hull) == 1:
            return hull[0].astype(float), np.zeros(2), 0.0, 0.0
        edge = hull[-1] - hull[0]
        yaw = math.atan2(edge[1], edge[0])
        return hull.mean(axis=0), np.array([float(np.linalg.norm(edge)), 0.0]), yaw, 0.0

    edges = np.diff(np.vstack([hull, hull[:1]]), axis=0)
    # Прямоугольник с осью angle тот же, что с angle + pi/2. К сторонам
    # оболочки добавлена сетка через 1 градус: короткие рёбра на шумной грани
    # уводят направление на несколько градусов.
    angles = np.unique(
        np.concatenate(
            [
                np.mod(np.arctan2(edges[:, 1], edges[:, 0]), math.pi / 2.0),
                np.radians(np.arange(0.0, 90.0, 1.0)),
            ]
        )
    )
    along = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    across = np.stack([-np.sin(angles), np.cos(angles)], axis=1)
    # Направление выбирается по точкам, прореженным до сетки 1 см: в плотном
    # кластере их в разы меньше, а среднее расстояние до сторон то же.
    # Координаты в осях каждого кандидата, (точки, кандидаты).
    sparse = np.unique(np.round(xy / 0.01), axis=0) * 0.01
    u, v = sparse @ along.T, sparse @ across.T
    low_u, high_u, low_v, high_v = u.min(axis=0), u.max(axis=0), v.min(axis=0), v.max(axis=0)
    to_side = np.minimum.reduce([u - low_u, high_u - u, v - low_v, high_v - v])
    # при равенстве -- меньшая площадь
    score = to_side.mean(axis=0) + 1e-6 * (high_u - low_u) * (high_v - low_v)
    best = int(np.argmin(score))

    # границы -- по всем точкам
    u, v = xy @ along[best], xy @ across[best]
    center = (u.max() + u.min()) / 2.0 * along[best] + (v.max() + v.min()) / 2.0 * across[best]
    yaw = float(angles[best])
    size = np.array([u.max() - u.min(), v.max() - v.min()])

    low_u, high_u = np.percentile(u, [2.0, 98.0])
    low_v, high_v = np.percentile(v, [2.0, 98.0])
    to_side = np.minimum.reduce(
        [np.abs(u - low_u), np.abs(high_u - u), np.abs(v - low_v), np.abs(high_v - v)]
    )
    rms = math.sqrt(float(np.mean(to_side ** 2)))

    if size[1] > size[0]:
        # длинная сторона -- вдоль yaw
        size = size[::-1]
        yaw += math.pi / 2.0
    return center, size, yaw, rms


def find_boxes(
    clusters: list,
    detections: list,
    model: RobotModel,
    min_height: float,
    observer: np.ndarray = None,
) -> list:
    """Предметы на сцене: кластеры, которые не робот

    Робот, прижавшийся к предмету, сливается с ним в один кластер шире
    робота. Тогда предмет -- точки кластера вне круга корпуса.

    :clusters список массивов (M, 3), z -- высота над полом
    :detections для каждого кластера Detection или None
    :model параметры робота: min_points, radius, max_extent
    :min_height верх предмета не ниже этого: ниже -- шум у пола, м
    :observer (2,) положение лидара: для проверки граней против окружности;
    None -- проверка не делается, planar всегда False

    :return список Box
    """
    boxes = []
    for points, detection in zip(clusters, detections):
        if detection is not None:
            if len(points) < model.min_points or major_extent(points[:, :2]) <= model.max_extent:
                continue
            outside = (
                np.linalg.norm(points[:, :2] - detection.center, axis=1)
                > model.radius + model.contain_margin
            )
            points = points[outside]

        if len(points) < model.min_points or points[:, 2].max() < min_height:
            continue

        xy = points[:, :2]
        center, size, yaw, faces = rectangle_fit(xy)
        planar = False
        if observer is not None and model.plane_ratio > 0.0:
            rim = xy[points[:, 2] <= model.rim_max_z]
            planar, faces, _ = plane_test(xy, edge_center(xy, rim, observer, model.radius), model)
        boxes.append(Box(center, size, yaw, float(points[:, 2].max()), len(points), planar, faces))
    return boxes
