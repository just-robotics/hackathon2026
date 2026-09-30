"""Pure obstacle projection, global A*, and shared navigation geometry."""

from dataclasses import dataclass
from heapq import heappop, heappush
from math import atan2, ceil, cos, hypot, pi, sin

try:
    from scipy.spatial import cKDTree
except ImportError:  # Pure-Python tests and environments without SciPy.
    cKDTree = None


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float = 0.0


def angle_error(a, b):
    return (a - b + pi) % (2 * pi) - pi


def path_heading_error(own, path):
    """Return angular error to a nearby local-path lookahead, if available."""
    if len(path) < 2:
        return None
    nearest = min(range(min(len(path), 8)),
                  key=lambda index: hypot(path[index].x - own.x,
                                          path[index].y - own.y))
    target = path[min(nearest + 3, len(path) - 1)]
    if hypot(target.x - own.x, target.y - own.y) < 0.05:
        return None
    desired = atan2(target.y - own.y, target.x - own.x)
    return abs(angle_error(desired, own.yaw))


def turn_alignment_is_progress(role, behavior):
    """Only delay watchdog recovery while a guardian aligns for pursuit."""
    return role == "guardian" and behavior in (6, 7)


def cell_tie(seed, cell):
    """Stable per-run tie breaker; it cannot outweigh a better path cost."""
    value = ((seed & 0xffffffff) ^ ((cell[0] & 0xffffffff) * 0x9e3779b1)
             ^ ((cell[1] & 0xffffffff) * 0x85ebca6b)) & 0xffffffff
    value ^= value >> 16
    value = (value * 0x7feb352d) & 0xffffffff
    value ^= value >> 15
    value = (value * 0x846ca68b) & 0xffffffff
    return value ^ (value >> 16)


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
        self.obstacle_bins = {}
        self.obstacle_tree = None
        self.cell_clearance_cache = {}
        self.map_bounds = None

    def cell(self, x, y):
        return round(x / self.resolution), round(y / self.resolution)

    def point(self, cell):
        return Pose2(cell[0] * self.resolution, cell[1] * self.resolution)

    def inside_map(self, x, y, margin=0.0):
        return self.map_clearance(x, y) >= margin

    def map_clearance(self, x, y):
        if self.map_bounds is None:
            return float("inf")
        min_x, min_y, max_x, max_y = self.map_bounds
        return min(x - min_x, y - min_y, max_x - x, max_y - y)

    def update(self, static_points, scan_points, scan_origin, known_free=(),
               map_bounds=None):
        self.map_bounds = map_bounds
        self.static_points = list(static_points)
        self.scan_points = list(scan_points)
        self.scan_origin = scan_origin
        self.voxels.clear()
        self.occupied.clear()
        self.free.clear()
        self.obstacle_bins.clear()
        raw_obstacles = []
        self.cell_clearance_cache.clear()
        radius_cells = ceil(self.robot_radius / self.resolution)
        for x, y, z in self.static_points + self.scan_points:
            if not 0.08 <= z <= 0.60:
                continue
            vx, vy = self.cell(x, y)
            vz = round(z / self.resolution)
            self.voxels.add((vx, vy, vz))
            self.obstacle_bins.setdefault((vx, vy), []).append((x, y))
            raw_obstacles.append((x, y))
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
        self.free.update(known_free)
        self.free.difference_update(self.occupied)
        if map_bounds is not None:
            self.free = {cell for cell in self.free
                         if self.inside_map(self.point(cell).x,
                                            self.point(cell).y,
                                            self.robot_radius + 0.1)}
        self.obstacle_tree = cKDTree(raw_obstacles) if cKDTree and raw_obstacles else None

    def blocked(self, x, y):
        return self.cell(x, y) in self.occupied

    def obstacle_clearance(self, x, y, search_radius=0.5):
        """Distance to nearby raw obstacle points, before safety inflation."""
        if self.obstacle_tree is not None:
            return float(self.obstacle_tree.query((x, y), k=1)[0])
        cx, cy = self.cell(x, y)
        radius_cells = ceil(search_radius / self.resolution) + 1
        nearest = float("inf")
        for ox in range(-radius_cells, radius_cells + 1):
            for oy in range(-radius_cells, radius_cells + 1):
                for px, py in self.obstacle_bins.get((cx + ox, cy + oy), ()):
                    nearest = min(nearest, hypot(x - px, y - py))
        return nearest

    def hard_blocked(self, x, y):
        return self.obstacle_clearance(x, y, 0.2) < 0.19

    def cell_clearance(self, cell):
        if cell not in self.cell_clearance_cache:
            point = self.point(cell)
            self.cell_clearance_cache[cell] = self.obstacle_clearance(
                point.x, point.y, self.robot_radius + 0.15)
        return self.cell_clearance_cache[cell]

    def clear_line_3d(self, a, b, height=0.3):
        z_cell = round(height / self.resolution)
        cells = list(ray_cells(self.cell(a.x, a.y), self.cell(b.x, b.y)))
        # The last cells contain the target robot; only the space between
        # the robots matters for a line-of-sight check.
        for x, y in cells[:-max(1, ceil(0.18 / self.resolution))]:
            if any((x, y, z) in self.voxels for z in (z_cell - 1, z_cell, z_cell + 1)):
                return False
        return True

    def frontier(self, own, destination=None, tie_seed=0, min_travel=0.0,
                 avoid=None):
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
                if travel < min_travel:
                    continue
                if avoid is not None and hypot(p.x - avoid.x, p.y - avoid.y) < 0.5:
                    continue
                toward = hypot(p.x - destination.x, p.y - destination.y) if destination else 0
                candidates.append((0.4 * travel + toward, cell_tie(tie_seed, c), p))
        return min(candidates, key=lambda item: item[:2])[2] if candidates else None


def opponent_cost(x, y, opponent, clearance, weight):
    if opponent is None:
        return 0.0
    d = hypot(x - opponent.x, y - opponent.y)
    if d < clearance:
        return float("inf")
    return weight * max(0.0, 1.2 - d) / 1.2


def astar(world, start, goal, opponent=None, clearance=0.0, weight=0.0,
          max_cells=8000, tie_seed=0, avoid=None):
    source, target = world.cell(start.x, start.y), world.cell(goal.x, goal.y)
    source_blocked = source in world.occupied
    source_opponent_distance = (hypot(start.x - opponent.x, start.y - opponent.y)
                                if opponent is not None else float("inf"))
    if target in world.occupied or not world.inside_map(goal.x, goal.y,
                                                       world.robot_radius + 0.1):
        return []
    queue = [(0.0, cell_tie(tie_seed, source), source)]
    cost = {source: 0.0}
    parent = {}
    closed = set()
    visited = 0
    while queue and visited < max_cells:
        _, _, current = heappop(queue)
        if current in closed:
            continue
        closed.add(current)
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
            if neighbor in closed:
                continue
            if neighbor in world.occupied:
                if (not source_blocked or
                        max(abs(neighbor[0] - source[0]),
                            abs(neighbor[1] - source[1])) > 3 or
                        world.hard_blocked(world.point(neighbor).x,
                                           world.point(neighbor).y)):
                    continue
            if dx and dy and ((current[0] + dx, current[1]) in world.occupied
                              or (current[0], current[1] + dy) in world.occupied):
                continue
            p = world.point(neighbor)
            if not world.inside_map(p.x, p.y, world.robot_radius + 0.1):
                continue
            threat = opponent_cost(p.x, p.y, opponent, clearance, weight)
            if threat == float("inf"):
                # When the opponent enters the exclusion zone, a wall may
                # require a short sideways or even closer step to escape.
                # Keep those steps expensive, and do not re-enter once clear.
                if opponent is None or source_opponent_distance >= clearance:
                    continue
                current_distance = hypot(world.point(current).x - opponent.x,
                                         world.point(current).y - opponent.y)
                next_distance = hypot(p.x - opponent.x, p.y - opponent.y)
                if (current_distance >= clearance or
                        next_distance < min(0.45, source_opponent_distance)
                        and next_distance < current_distance):
                    continue
                threat = (weight * max(0.0, 1.2 - next_distance) / 1.2 +
                          3.0 * weight * (clearance - next_distance) / clearance)
            unknown = (3.0 if neighbor in world.occupied else
                       1.8 if neighbor not in world.free else 1.0)
            wall_margin = max(0.0, world.robot_radius + 0.12 -
                              world.cell_clearance(neighbor))
            recovery_cost = (8.0 * max(0.0, 0.5 - hypot(p.x - avoid.x,
                                                       p.y - avoid.y)) / 0.5
                             if avoid is not None else 0.0)
            tentative = (cost[current] + hypot(dx, dy) * unknown + threat
                         + 4.0 * wall_margin + recovery_cost)
            if tentative >= cost.get(neighbor, float("inf")):
                continue
            cost[neighbor] = tentative
            parent[neighbor] = current
            heuristic = hypot(neighbor[0] - target[0], neighbor[1] - target[1])
            heappush(queue, (tentative + heuristic,
                             cell_tie(tie_seed, neighbor), neighbor))
    return []


def capture_goal(world, own, opponent):
    options = []
    for index in range(16):
        angle = 2 * pi * index / 16
        pose = Pose2(opponent.x + 0.39 * cos(angle),
                     opponent.y + 0.39 * sin(angle), angle + pi)
        rounded = world.point(world.cell(pose.x, pose.y))
        if (hypot(rounded.x - opponent.x, rounded.y - opponent.y) <= 0.36
                or world.blocked(pose.x, pose.y)
                or not world.inside_map(pose.x, pose.y, world.robot_radius + 0.07)
                or world.obstacle_clearance(pose.x, pose.y) < world.robot_radius + 0.07
                or not world.clear_line_3d(pose, opponent)):
            continue
        if safe_segment(world, own, pose):
            travel = hypot(pose.x - own.x, pose.y - own.y)
        else:
            route = astar(world, own, pose, max_cells=3000)
            if not route:
                continue
            travel = sum(hypot(b.x - a.x, b.y - a.y)
                         for a, b in zip(route, route[1:]))
        options.append((travel, pose))
    return min(options, key=lambda pair: pair[0])[1] if options else None


def reachable_intercept(world, own, opponent, predicted):
    """Do not pursue an extrapolation that runs through a known wall."""
    if (predicted is not None and
            world.inside_map(predicted.x, predicted.y, world.robot_radius + 0.07) and
            not world.blocked(predicted.x, predicted.y) and
            safe_segment(world, opponent, predicted)):
        return predicted
    return capture_goal(world, own, opponent) or predicted


def smooth_intercept_target(previous, candidate, alpha=0.55,
                            reset_distance=0.8):
    """Dampen small pursuit-target jitter without delaying decisive redirects."""
    if previous is None or candidate is None:
        return candidate
    dx, dy = candidate.x - previous.x, candidate.y - previous.y
    distance = hypot(dx, dy)
    if distance >= reset_distance:
        return candidate
    return Pose2(previous.x + alpha * dx, previous.y + alpha * dy,
                 candidate.yaw)


def reachable_target(world, own, target, explore=False, tie_seed=0,
                     avoid=None):
    """Stay inside observed free space until the intended destination is visible."""
    if target is not None and not explore and world.cell(target.x, target.y) in world.free:
        return target
    return (world.frontier(own, target, tie_seed, min_travel=0.6,
                           avoid=avoid)
            or world.frontier(own, target, tie_seed, min_travel=0.6)
            or world.frontier(own, target, tie_seed))


def coverage_target(world, own, visited, rng=None, tie_seed=0):
    """Pick a reachable free cell far from already searched positions."""
    candidates = []
    for cell in world.free:
        if cell in world.occupied or cell[0] % 3 or cell[1] % 3:
            continue
        point = world.point(cell)
        distance = hypot(point.x - own.x, point.y - own.y)
        if distance < 0.8:
            continue
        novelty = min(hypot(point.x - seen.x, point.y - seen.y)
                      for seen in visited)
        candidates.append((novelty - 0.2 * distance, point))
    reachable = [(score, point) for score, point in
                 sorted(candidates, key=lambda item: (-item[0],
                        cell_tie(tie_seed, world.cell(item[1].x, item[1].y))))[:30]
                 if astar(world, own, point, max_cells=5000, tie_seed=tie_seed)]
    if reachable:
        best = reachable[0][0]
        equivalent = [point for score, point in reachable if score >= best - 0.15]
        return rng.choice(equivalent) if rng else equivalent[0]
    return world.frontier(own, tie_seed=tie_seed)


def reusable_route(world, own, route, previous_target, target,
                   opponent=None, clearance=0.0):
    """Advance a safe route without changing corridors on each map refresh."""
    if (not route or previous_target is None or target is None or
            hypot(target.x - previous_target.x,
                  target.y - previous_target.y) > 0.3):
        return []
    closest = min(range(min(len(route), 8)),
                  key=lambda index: hypot(route[index].x - own.x,
                                          route[index].y - own.y))
    if hypot(route[closest].x - own.x, route[closest].y - own.y) > 0.45:
        return []
    remaining = route[closest:]
    for point in remaining[:15]:
        distance = hypot(point.x - own.x, point.y - own.y)
        # A fresh scan can reveal an obstacle directly on the next few cells.
        # Only the robot's own raster cell may stay conservatively occupied.
        if distance > 0.2 and world.blocked(point.x, point.y):
            return []
        if (opponent is not None and clearance > 0 and distance > 0.5 and
                hypot(point.x - opponent.x, point.y - opponent.y) < clearance):
            return []
    return remaining


def safe_segment(world, start, end, opponent=None, clearance=0.0,
                 safety_margin=0.12):
    """Check the swept robot centre, not just the raster cells at endpoints."""
    distance = hypot(end.x - start.x, end.y - start.y)
    steps = max(1, ceil(distance / 0.05))
    initial_wall = world.obstacle_clearance(start.x, start.y)
    initial_map = world.map_clearance(start.x, start.y)
    initial_opponent = (hypot(start.x - opponent.x, start.y - opponent.y)
                        if opponent is not None else float("inf"))
    for index in range(1, steps + 1):
        fraction = index / steps
        x = start.x + fraction * (end.x - start.x)
        y = start.y + fraction * (end.y - start.y)
        wall = world.obstacle_clearance(x, y)
        map_margin = world.map_clearance(x, y)
        required = world.robot_radius + safety_margin
        travel = hypot(x - start.x, y - start.y)
        # A path beginning too close to a wall must actually leave it. Merely
        # holding the same small clearance allowed MPC tracking error to scrape
        # along a wall until physical contact.
        if wall < required:
            gain = min(0.04, 0.15 * travel)
            if initial_wall >= required or wall + 0.005 < initial_wall + gain:
                return False
        if map_margin < required:
            gain = min(0.04, 0.15 * travel)
            if initial_map >= required or map_margin + 0.005 < initial_map + gain:
                return False
        if (world.blocked(x, y) and
                hypot(x - start.x, y - start.y) > 0.25):
            return False
        if opponent is not None and clearance > 0:
            opponent_distance = hypot(x - opponent.x, y - opponent.y)
            if (opponent_distance < clearance and
                    (initial_opponent >= clearance or
                     opponent_distance < max(0.35, initial_opponent - 0.15))):
                return False
    return True


def smooth_control_route(world, route, opponent=None, clearance=0.0,
                         safety_margin=0.12, max_shortcut=2.0):
    """Make a long, traversable spatial path for Lat-MPC from grid A*."""
    if len(route) < 3:
        return list(route)
    anchors = [route[0]]
    index = 0
    while index < len(route) - 1:
        chosen = index + 1
        for candidate in range(index + 2, len(route)):
            if hypot(route[candidate].x - route[index].x,
                     route[candidate].y - route[index].y) > max_shortcut:
                break
            if safe_segment(world, route[index], route[candidate], opponent,
                            clearance, safety_margin):
                chosen = candidate
        anchors.append(route[chosen])
        index = chosen

    rounded = [anchors[0]]
    for index in range(1, len(anchors) - 1):
        before, corner, after = anchors[index - 1:index + 2]
        incoming = hypot(corner.x - before.x, corner.y - before.y)
        outgoing = hypot(after.x - corner.x, after.y - corner.y)
        if min(incoming, outgoing) < 0.2:
            rounded.append(corner)
            continue
        length = min(0.45, 0.38 * incoming, 0.38 * outgoing)
        entry = Pose2(corner.x - length * (corner.x - before.x) / incoming,
                      corner.y - length * (corner.y - before.y) / incoming)
        exit_point = Pose2(corner.x + length * (after.x - corner.x) / outgoing,
                           corner.y + length * (after.y - corner.y) / outgoing)
        curve = [Pose2((1 - t) ** 2 * entry.x + 2 * t * (1 - t) * corner.x +
                       t ** 2 * exit_point.x,
                       (1 - t) ** 2 * entry.y + 2 * t * (1 - t) * corner.y +
                       t ** 2 * exit_point.y)
                 for t in (0.0, 0.25, 0.5, 0.75, 1.0)]
        chain = [rounded[-1]] + curve + [after]
        if all(safe_segment(world, a, b, opponent, clearance, safety_margin)
               for a, b in zip(chain, chain[1:])):
            rounded.extend(curve)
        else:
            rounded.append(corner)
    rounded.append(anchors[-1])

    result = [rounded[0]]
    for start, end in zip(rounded, rounded[1:]):
        length = hypot(end.x - start.x, end.y - start.y)
        if length < 0.001:
            continue
        yaw = atan2(end.y - start.y, end.x - start.x)
        count = max(1, ceil(length / 0.10))
        result.extend(Pose2(start.x + (end.x - start.x) * step / count,
                            start.y + (end.y - start.y) * step / count, yaw)
                      for step in range(1, count + 1))
    return result


def local_guidance(world, own, route, opponent=None, clearance=0.0,
                   max_lookahead=1.6, min_step=0.12, safety_margin=0.12):
    """Give MPC a straight, collision-checked corridor instead of a new arc each tick."""
    if not route:
        return []
    candidates = [point for point in route
                  if hypot(point.x - own.x, point.y - own.y) >= min_step]
    visible = None
    for point in candidates:
        distance = hypot(point.x - own.x, point.y - own.y)
        if distance > max_lookahead and visible is not None:
            break
        if distance > max_lookahead + 0.3:
            break
        if safe_segment(world, own, point, opponent, clearance, safety_margin):
            visible = point
        elif visible is not None:
            break
    if visible is None:
        return []
    distance = hypot(visible.x - own.x, visible.y - own.y)
    yaw = atan2(visible.y - own.y, visible.x - own.x)
    steps = max(2, ceil(distance / 0.1))
    return [own] + [Pose2(own.x + (visible.x - own.x) * i / steps,
                          own.y + (visible.y - own.y) * i / steps, yaw)
                    for i in range(1, steps + 1)]


def recovery_step(world, own, opponent=None, clearance=0.0,
                  safety_margin=0.12):
    """Find a safe escape direction when the nominal corridor cannot be entered."""
    options = []
    for length in (0.55, 0.3, 0.2, 0.15, 0.1):
        for index in range(24):
            yaw = -pi + 2 * pi * index / 24
            endpoint = Pose2(own.x + length * cos(yaw),
                             own.y + length * sin(yaw))
            if safe_segment(world, own, endpoint, opponent, clearance,
                            safety_margin):
                wall = world.obstacle_clearance(endpoint.x, endpoint.y)
                map_margin = world.map_clearance(endpoint.x, endpoint.y)
                options.append((min(wall, 0.6) + min(map_margin, 0.6) -
                                0.05 * abs(angle_error(yaw, own.yaw)), yaw, length))
        if options:
            break
    return max(options)[1:] if options else None


def checked_recovery_target(world, own, previous=None, opponent=None,
                            clearance=0.0, safety_margin=0.12):
    """Retain only a currently reachable escape, with its travel heading."""
    if previous is not None and safe_segment(
            world, own, previous, opponent, clearance, safety_margin):
        return Pose2(previous.x, previous.y,
                     atan2(previous.y - own.y, previous.x - own.x))
    step = recovery_step(world, own, opponent, clearance, safety_margin)
    if step is None:
        return None
    heading, length = step
    return Pose2(own.x + length * cos(heading),
                 own.y + length * sin(heading), heading)
