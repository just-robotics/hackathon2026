"""Pure map projection, A* and sampled differential-drive local planning."""

from dataclasses import dataclass
from heapq import heappop, heappush
from math import atan2, ceil, cos, hypot, pi, sin


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float = 0.0


def angle_error(a, b):
    return (a - b + pi) % (2 * pi) - pi


def ray_cells(a, b):
    """All integer cells touched by a 2D line, including its endpoints."""
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if (x0, y0) == (x1, y1):
            return
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


class VoxelWorld:
    def __init__(self, resolution=0.15, robot_radius=0.22):
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.static_points = []
        self.scan_points = []
        self.scan_origin = None
        self.voxels = set()
        self.occupied = set()
        self.free = set()

    def cell(self, x, y):
        return round(x / self.resolution), round(y / self.resolution)

    def point(self, cell):
        return Pose2(cell[0] * self.resolution, cell[1] * self.resolution)

    def update(self, static_points, scan_points, scan_origin):
        self.static_points = list(static_points)
        self.scan_points = list(scan_points)
        self.scan_origin = scan_origin
        self.voxels.clear()
        self.occupied.clear()
        self.free.clear()
        radius_cells = ceil(self.robot_radius / self.resolution)
        for x, y, z in self.static_points + self.scan_points:
            if not 0.08 <= z <= 0.60:
                continue
            vx, vy = self.cell(x, y)
            vz = round(z / self.resolution)
            self.voxels.add((vx, vy, vz))
            for ox in range(-radius_cells, radius_cells + 1):
                for oy in range(-radius_cells, radius_cells + 1):
                    if hypot(ox * self.resolution, oy * self.resolution) <= self.robot_radius:
                        self.occupied.add((vx + ox, vy + oy))
        if scan_origin:
            origin = self.cell(scan_origin.x, scan_origin.y)
            for x, y, z in self.scan_points[::max(1, len(self.scan_points) // 1500)]:
                if z < 0.02:
                    continue
                cells = list(ray_cells(origin, self.cell(x, y)))
                self.free.update(cells[:-1])
            self.free.difference_update(self.occupied)

    def blocked(self, x, y):
        return self.cell(x, y) in self.occupied

    def clear_line_3d(self, a, b, height=0.3):
        z_cell = round(height / self.resolution)
        cells = list(ray_cells(self.cell(a.x, a.y), self.cell(b.x, b.y)))
        # The last cells contain the target robot; only the space between
        # the robots matters for a line-of-sight check.
        for x, y in cells[:-max(1, ceil(0.18 / self.resolution))]:
            if any((x, y, z) in self.voxels for z in (z_cell - 1, z_cell, z_cell + 1)):
                return False
        return True

    def frontier(self, own, destination=None):
        if not self.free:
            return None
        candidates = []
        for c in self.free:
            if c in self.occupied:
                continue
            if any((c[0] + dx, c[1] + dy) not in self.free
                   for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))):
                p = self.point(c)
                travel = hypot(p.x - own.x, p.y - own.y)
                toward = hypot(p.x - destination.x, p.y - destination.y) if destination else 0
                candidates.append((0.4 * travel + toward, p))
        return min(candidates, key=lambda pair: pair[0])[1] if candidates else None


def opponent_cost(x, y, opponent, clearance, weight):
    if opponent is None:
        return 0.0
    d = hypot(x - opponent.x, y - opponent.y)
    if d < clearance:
        return float("inf")
    return weight * max(0.0, 1.2 - d) / 1.2


def astar(world, start, goal, opponent=None, clearance=0.0, weight=0.0,
          max_cells=30000):
    source, target = world.cell(start.x, start.y), world.cell(goal.x, goal.y)
    if target in world.occupied:
        return []
    queue = [(0.0, source)]
    cost = {source: 0.0}
    parent = {}
    visited = 0
    while queue and visited < max_cells:
        _, current = heappop(queue)
        visited += 1
        if current == target:
            route = [current]
            while route[-1] != source:
                route.append(parent[route[-1]])
            route.reverse()
            return [world.point(c) for c in route]
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor in world.occupied:
                continue
            if dx and dy and ((current[0] + dx, current[1]) in world.occupied
                              or (current[0], current[1] + dy) in world.occupied):
                continue
            p = world.point(neighbor)
            threat = opponent_cost(p.x, p.y, opponent, clearance, weight)
            if threat == float("inf"):
                continue
            unknown = 1.8 if neighbor not in world.free else 1.0
            tentative = cost[current] + hypot(dx, dy) * unknown + threat
            if tentative >= cost.get(neighbor, float("inf")):
                continue
            cost[neighbor] = tentative
            parent[neighbor] = current
            heuristic = hypot(neighbor[0] - target[0], neighbor[1] - target[1])
            heappush(queue, (tentative + heuristic, neighbor))
    return []


def capture_goal(world, own, opponent):
    options = []
    for index in range(16):
        angle = 2 * pi * index / 16
        pose = Pose2(opponent.x + 0.43 * cos(angle),
                     opponent.y + 0.43 * sin(angle), angle + pi)
        rounded = world.point(world.cell(pose.x, pose.y))
        if (hypot(rounded.x - opponent.x, rounded.y - opponent.y) > 0.36
                and not world.blocked(pose.x, pose.y)
                and world.clear_line_3d(pose, opponent)):
            options.append((hypot(pose.x - own.x, pose.y - own.y), pose))
    return min(options, key=lambda pair: pair[0])[1] if options else None


def reachable_target(world, own, target, explore=False):
    """Stay inside observed free space until the intended destination is visible."""
    if target is not None and not explore and world.cell(target.x, target.y) in world.free:
        return target
    return world.frontier(own, target)


def local_rollout(world, own, global_path, opponent=None, clearance=0.0,
                  weight=0.0, max_speed=0.5, horizon=2.0, dt=0.2,
                  opponent_velocity=(0.0, 0.0)):
    if not global_path:
        return []
    lookahead = global_path[-1]
    for point in global_path:
        if hypot(point.x - own.x, point.y - own.y) >= 0.8:
            lookahead = point
            break
    initial_distance = hypot(lookahead.x - own.x, lookahead.y - own.y)
    best = None
    for velocity in (0.0, max_speed * 0.3, max_speed * 0.6, max_speed):
        for omega in (-1.2, -0.6, 0.0, 0.6, 1.2):
            x, y, yaw = own.x, own.y, own.yaw
            poses = [own]
            min_clearance = float("inf")
            valid = True
            for step in range(int(horizon / dt)):
                x += velocity * cos(yaw) * dt
                y += velocity * sin(yaw) * dt
                yaw = angle_error(yaw + omega * dt, 0)
                if world.blocked(x, y):
                    valid = False
                    break
                if opponent is not None:
                    future_x = opponent.x + opponent_velocity[0] * (step + 1) * dt
                    future_y = opponent.y + opponent_velocity[1] * (step + 1) * dt
                    distance = hypot(x - future_x, y - future_y)
                    if distance < clearance:
                        valid = False
                        break
                    min_clearance = min(min_clearance, distance)
                poses.append(Pose2(x, y, yaw))
            if not valid:
                continue
            end = poses[-1]
            remaining = hypot(end.x - lookahead.x, end.y - lookahead.y)
            heading = abs(angle_error(atan2(lookahead.y - end.y,
                                            lookahead.x - end.x), end.yaw))
            progress = initial_distance - remaining
            threat = weight * max(0, 1.2 - min_clearance) if opponent else 0.0
            score = 4 * progress - remaining - 0.35 * heading - threat
            if velocity == 0 and abs(omega) < 0.01:
                score -= 0.5
            if best is None or score > best[0]:
                best = score, poses
    return best[1] if best else []
