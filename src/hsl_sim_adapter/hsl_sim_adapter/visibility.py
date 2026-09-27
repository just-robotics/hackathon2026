"""Static-map filtering and simulated LiDAR confirmation of an opponent."""

from math import ceil, floor, hypot


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


def opponent_visible(grid, own, opponent, scan_points, *, detection_radius=0.38,
                     min_hits=3, max_range=40.0):
    """Use truth only to associate a visible LiDAR cluster with the opponent."""
    if grid is None or own is None or opponent is None:
        return False
    if hypot(opponent[0] - own[0], opponent[1] - own[1]) > max_range:
        return False
    if not grid.clear_line(own, opponent):
        return False
    hits = 0
    for x, y, z in scan_points:
        if (0.08 <= z <= 0.60
                and hypot(x - opponent[0], y - opponent[1]) <= detection_radius
                and not grid.matches_static(x, y)):
            hits += 1
            if hits >= min_hits:
                return True
    return False
