"""Static-map filtering and opponent position estimation from LiDAR returns."""

from collections import defaultdict, deque
from dataclasses import dataclass
from math import ceil, floor, hypot
from statistics import median


class StaticGrid:
    def __init__(self, resolution, width, height, origin_x, origin_y, data):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.data = data

    def cell(self, x, y):
        return (floor((x - self.origin_x) / self.resolution + 1e-9),
                floor((y - self.origin_y) / self.resolution + 1e-9))

    def occupied(self, x, y):
        col, row = self.cell(x, y)
        if not (0 <= col < self.width and 0 <= row < self.height):
            return True
        return self.data[row * self.width + col] != 0

    def matches_static(self, x, y, margin=0.08):
        """A measured point belongs to a wall when near an occupied map cell."""
        col, row = self.cell(x, y)
        radius = ceil(margin / self.resolution)
        for r in range(row - radius, row + radius + 1):
            for c in range(col - radius, col + radius + 1):
                if not (0 <= c < self.width and 0 <= r < self.height):
                    continue
                if self.data[r * self.width + c] < 50:
                    continue
                cx = self.origin_x + (c + 0.5) * self.resolution
                cy = self.origin_y + (r + 0.5) * self.resolution
                if hypot(x - cx, y - cy) <= margin + self.resolution * 0.71:
                    return True
        return False

    def clear_line(self, start, end, endpoint_radius=0.20):
        distance = hypot(end[0] - start[0], end[1] - start[1])
        if distance <= 2 * endpoint_radius:
            return True
        step = self.resolution * 0.5
        count = ceil((distance - 2 * endpoint_radius) / step)
        for index in range(count + 1):
            travel = endpoint_radius + index * (distance - 2 * endpoint_radius) / count
            fraction = travel / distance
            x = start[0] + fraction * (end[0] - start[0])
            y = start[1] + fraction * (end[1] - start[1])
            if self.occupied(x, y):
                return False
        return True


@dataclass(frozen=True)
class OpponentDetection:
    x: float
    y: float
    hits: int
    extent: float


def _clusters(points, tolerance):
    """Connected components of XY returns, using a small spatial hash."""
    bins = defaultdict(list)
    for index, (x, y, _) in enumerate(points):
        bins[(floor(x / tolerance), floor(y / tolerance))].append(index)
    visited = set()
    for seed in range(len(points)):
        if seed in visited:
            continue
        visited.add(seed)
        queue = deque([seed])
        cluster = []
        while queue:
            current = queue.popleft()
            cluster.append(points[current])
            x, y, _ = points[current]
            cell_x, cell_y = floor(x / tolerance), floor(y / tolerance)
            for bx in range(cell_x - 2, cell_x + 3):
                for by in range(cell_y - 2, cell_y + 3):
                    for candidate in bins.get((bx, by), ()):
                        if candidate in visited:
                            continue
                        qx, qy, _ = points[candidate]
                        if hypot(qx - x, qy - y) <= tolerance:
                            visited.add(candidate)
                            queue.append(candidate)
        yield cluster


def detect_opponent(scan_points, grid, own, previous=None, *,
                    body_radius=0.178, self_radius=0.25,
                    min_hits=3, cluster_tolerance=0.23,
                    max_cluster_extent=0.70, max_range=40.0):
    """Estimate an opponent centre from non-static LiDAR point clusters.

    The returns describe the visible surface, not the robot centre. Move each
    return outward by the known Kobuki body radius, then robustly combine the
    implied centres. Ground-truth opponent pose is not an input.
    """
    if grid is None or own is None:
        return None
    candidates = []
    for point in scan_points:
        x, y, z = point
        distance = hypot(x - own[0], y - own[1])
        if (not 0.08 <= z <= 0.60 or distance < self_radius or
                distance > max_range or grid.matches_static(x, y)):
            continue
        candidates.append((x, y, z))

    detections = []
    for cluster in _clusters(candidates, cluster_tolerance):
        if len(cluster) < min_hits:
            continue
        xs = [point[0] for point in cluster]
        ys = [point[1] for point in cluster]
        extent = hypot(max(xs) - min(xs), max(ys) - min(ys))
        if extent > max_cluster_extent:
            continue
        implied_centres = []
        for x, y, _ in cluster:
            dx, dy = x - own[0], y - own[1]
            distance = hypot(dx, dy)
            if distance > 1e-6:
                scale = body_radius / distance
                implied_centres.append((x + dx * scale, y + dy * scale))
        if not implied_centres:
            continue
        centre = (median(point[0] for point in implied_centres),
                  median(point[1] for point in implied_centres))
        if not grid.clear_line(own, centre):
            continue
        detections.append(OpponentDetection(centre[0], centre[1],
                                             len(cluster), extent))

    if not detections:
        return None
    if previous is not None:
        return min(detections, key=lambda item: hypot(item.x - previous[0],
                                                       item.y - previous[1]))
    return min(detections, key=lambda item: (-item.hits,
                                              hypot(item.x - own[0],
                                                    item.y - own[1])))
