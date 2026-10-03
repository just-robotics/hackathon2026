"""Shared static-map masking and XY clustering for obstacle geometry.

No robot detector or tracker implementation lives here.
"""
import math
import numpy as np

class StaticBackground:
    """Occupied cells only; observed dynamic cells must never be fed here."""
    def __init__(self, resolution, origin, width, height, data, margin=.08):
        if resolution <= 0 or len(data) != width * height:
            raise ValueError('invalid occupancy grid')
        self.resolution = resolution
        self.origin = np.asarray(origin)
        self.width, self.height = width, height
        self.grid = np.asarray(data).reshape(height, width)
        occupied = self.grid >= 50
        self.mask = occupied.copy()
        radius = math.ceil(margin / resolution + .71)
        # Raster approximation to the original analytic wall margin.
        padded = np.pad(occupied, radius)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if math.hypot(dx, dy) * resolution <= margin + .71 * resolution:
                    self.mask |= padded[radius+dy:radius+dy+height, radius+dx:radius+dx+width]

    def foreground(self, points):
        indices = np.floor((points[:, :2] - self.origin) / self.resolution).astype(int)
        x, y = indices.T
        inside = (x >= 0) & (y >= 0) & (x < self.width) & (y < self.height)
        keep = inside.copy()
        keep[inside] &= ~self.mask[y[inside], x[inside]]
        keep[inside] &= self.grid[y[inside], x[inside]] >= 0
        return keep

    def free_center(self, xy):
        """A fitted robot center cannot lie in a wall or unknown map cell."""
        x, y = np.floor((np.asarray(xy) - self.origin) / self.resolution).astype(int)
        return (0 <= x < self.width and 0 <= y < self.height
                and 0 <= self.grid[y, x] < 50)


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
