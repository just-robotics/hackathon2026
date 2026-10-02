"""ROS-independent SDF collision extraction and occupancy rasterization."""
import math
import xml.etree.ElementTree as ET
import numpy as np

# Значения ячеек в OccupancyGrid: 0 -- свободно, 100 -- занято.
FREE = 0
OCCUPIED = 100


def pose_of(element) -> list:
    """Прочитать <pose> элемента

    :element узел SDF

    :return [x, y, z, roll, pitch, yaw], нули если поза не задана
    """
    text = element.findtext("pose")

    if not text:
        return [0.0] * 6

    values = [float(v) for v in text.split()]
    return values + [0.0] * (6 - len(values))


def state_link_poses(world) -> dict:
    """Прочитать абсолютные позы link из секции <state>

    Миры, сохранённые из Gazebo, несут снимок симуляции в <state>, и именно
    его симулятор применяет поверх определения <model>. Позы link там заданы
    сразу в координатах мира, а не относительно модели.

    Для миров со снимком состояния это приоритетный источник: в <model> несколько
    стен стоят в одной точке (Wall_Stage_1 и Wall_0 совпадают), то есть
    определение устарело относительно снимка.

    :world узел <world>

    :return {(модель, link): поза в координатах мира}
    """
    state = world.find("state")

    if state is None:
        return {}

    poses = {}
    for model in state.findall("model"):
        for link in model.findall("link"):
            poses[(model.get("name"), link.get("name"))] = pose_of(link)

    return poses


def collect_boxes(world_path: str, z_slice: float) -> list:
    """Собрать box-коллизии мира как прямоугольники на плоскости XY

    Размеры боксов берутся из определения <model>, а позы -- из <state>, где
    они абсолютные (см. state_link_poses). Если снимка нет, поза собирается
    из цепочки модель -> link -> collision.

    Боксы, не попадающие в срез на высоте z_slice, пропускаются -- так из
    карты выпадает ground_plane и любая геометрия под или над роботом.

    :world_path путь к .world (SDF)
    :z_slice высота среза в метрах

    :return список (cx, cy, half_x, half_y, yaw) в координатах мира
    """
    world = ET.parse(world_path).getroot().find("world")
    absolute = state_link_poses(world)
    boxes = []

    for model in world.findall("model"):
        mx, my, mz, _, _, myaw = pose_of(model)

        for link in model.findall("link"):
            state_pose = absolute.get((model.get("name"), link.get("name")))
            lx, ly, lz, _, _, lyaw = (
                state_pose if state_pose else pose_of(link)
            )

            for collision in link.findall("collision"):
                box = collision.find("geometry/box")
                if box is None:
                    # плоскости, цилиндры и меши на карту не попадают
                    continue

                sx, sy, sz = [float(v) for v in box.findtext("size").split()]
                cx, cy, cz, _, _, cyaw = pose_of(collision)

                # поза коллизии задана в системе link; её поворот вокруг Z
                # складываем с поворотом link, крен и тангаж у стен нулевые
                yaw = lyaw
                rx = lx + cx * math.cos(yaw) - cy * math.sin(yaw)
                ry = ly + cx * math.sin(yaw) + cy * math.cos(yaw)

                if state_pose:
                    # поза из <state> уже в координатах мира
                    wx, wy, wz = rx, ry, lz + cz
                else:
                    wx = mx + rx * math.cos(myaw) - ry * math.sin(myaw)
                    wy = my + rx * math.sin(myaw) + ry * math.cos(myaw)
                    wz = mz + lz + cz
                    yaw += myaw

                if not (wz - sz / 2.0) <= z_slice <= (wz + sz / 2.0):
                    continue

                boxes.append((wx, wy, sx / 2.0, sy / 2.0, yaw + cyaw))

    return boxes


def rasterize(
    boxes: list,
    resolution: float,
    padding: float,
    origin: tuple = (0.0, 0.0),
) -> tuple:
    """Разложить прямоугольники в сетку занятости

    Ячейка считается занятой, если её центр лежит внутри прямоугольника.
    Проверка идёт в локальных координатах стены, поэтому поворот учитывается
    без приближения габаритным прямоугольником.

    Координаты пересчитываются относительно origin. Начало задаётся независимо от стартовых поз роботов через
    simulation.map_origin_world; по умолчанию совпадает с миром Gazebo.

    :boxes список (cx, cy, half_x, half_y, yaw) в координатах мира
    :resolution размер ячейки в метрах
    :padding свободные поля вокруг занятой области в метрах
    :origin (x, y) начала отсчёта карты в координатах мира

    :return (grid, origin_x, origin_y) -- origin в системе, заданной origin
    """
    shift_x, shift_y = origin

    corners_x, corners_y = [], []
    for cx, cy, hx, hy, yaw in boxes:
        for sx, sy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            corners_x.append(cx + sx * math.cos(yaw) - sy * math.sin(yaw))
            corners_y.append(cy + sx * math.sin(yaw) + sy * math.cos(yaw))

    # сетка должна накрыть и все стены, и саму точку отсчёта
    min_x = min(corners_x + [shift_x]) - padding
    min_y = min(corners_y + [shift_y]) - padding
    max_x = max(corners_x + [shift_x]) + padding
    max_y = max(corners_y + [shift_y]) + padding

    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))

    grid = np.full((height, width), FREE, dtype=np.int8)

    # центры ячеек
    xs = min_x + (np.arange(width) + 0.5) * resolution
    ys = min_y + (np.arange(height) + 0.5) * resolution
    mesh_x, mesh_y = np.meshgrid(xs, ys)

    # Стены тоньше ячейки иначе проскакивают между их центрами: при 0.1 м на
    # ячейку перегородки толщиной 4.5 см давали пустую карту. Поэтому
    # полутолщина поднимается до половины диагонали ячейки -- тонкая стена
    # гарантированно закрывает клетки, через которые проходит.
    minimum_half = resolution * math.sqrt(2.0) / 2.0

    for cx, cy, hx, hy, yaw in boxes:
        dx = mesh_x - cx
        dy = mesh_y - cy
        # поворот в систему стены
        local_x = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        local_y = dx * math.sin(-yaw) + dy * math.cos(-yaw)
        inside = (np.abs(local_x) <= max(hx, minimum_half)) & (
            np.abs(local_y) <= max(hy, minimum_half)
        )
        grid[inside] = OCCUPIED

    # origin отдаётся относительно выбранного начала координат.
    return grid, min_x - shift_x, min_y - shift_y

