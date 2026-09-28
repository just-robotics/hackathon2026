"""Pure map projection, A* and sampled differential-drive local planning."""

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


def safe_segment(world, start, end, opponent=None, clearance=0.0):
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
        required = world.robot_radius + 0.07
        if (wall < required and
                (initial_wall >= required or wall < initial_wall - 0.01)):
            return False
        if (map_margin < required and
                (initial_map >= required or map_margin < initial_map - 0.01)):
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


def local_guidance(world, own, route, opponent=None, clearance=0.0,
                   max_lookahead=1.6, min_step=0.12):
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
        if safe_segment(world, own, point, opponent, clearance):
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


def curved_guidance(world, own, straight, opponent=None, clearance=0.0):
    """Join the current heading to a visible corridor without stopping to turn.

    A tight bend or an obstructed swept path keeps the straight reference so
    the controller can turn in place before entering it.
    """
    if len(straight) < 3:
        return straight
    end = straight[-1]
    dx, dy = end.x - own.x, end.y - own.y
    distance = hypot(dx, dy)
    if distance < 0.65:
        return straight
    heading = atan2(dy, dx)
    error = angle_error(heading, own.yaw)
    if abs(error) < 0.18 or abs(error) > 1.15:
        return straight
    # MPC does not follow the reference exactly. Reserve tracking room around
    # every moving turn, especially beside walls and the arena boundary.
    curve_margin = world.robot_radius + 0.18
    if (world.obstacle_clearance(own.x, own.y) < curve_margin or
            world.map_clearance(own.x, own.y) < curve_margin):
        return straight
    # The first control point follows the robot heading; the last one joins
    # the original collision-checked corridor tangentially.
    handle = min(0.65, max(0.32, 0.48 * distance))
    p1 = (own.x + handle * cos(own.yaw), own.y + handle * sin(own.yaw))
    p2 = (end.x - handle * cos(heading), end.y - handle * sin(heading))
    steps = max(12, ceil(distance / 0.06))
    path = [own]
    for index in range(1, steps + 1):
        t = index / steps
        u = 1.0 - t
        x = (u ** 3 * own.x + 3 * u * u * t * p1[0] +
             3 * u * t * t * p2[0] + t ** 3 * end.x)
        y = (u ** 3 * own.y + 3 * u * u * t * p1[1] +
             3 * u * t * t * p2[1] + t ** 3 * end.y)
        point = Pose2(x, y)
        if (world.obstacle_clearance(x, y) < curve_margin or
                world.map_clearance(x, y) < curve_margin):
            return straight
        if not safe_segment(world, path[-1], point, opponent, clearance):
            return straight
        if len(path) >= 2:
            first, middle = path[-2:]
            ax, ay = middle.x - first.x, middle.y - first.y
            bx, by = point.x - middle.x, point.y - middle.y
            chord = hypot(point.x - first.x, point.y - first.y)
            curvature = (2.0 * abs(ax * by - ay * bx) /
                         max(hypot(ax, ay) * hypot(bx, by) * chord, 1e-9))
            if curvature > 2.6:
                return straight
        path.append(point)
    return path


def reusable_local_guidance(world, own, path, opponent=None, clearance=0.0):
    """Keep fixed path geometry while it remains reachable and unobstructed."""
    if len(path) < 3:
        return []
    end = path[-1]
    if hypot(end.x - own.x, end.y - own.y) < 0.35:
        return []
    closest_index = min(range(len(path)),
                        key=lambda i: hypot(path[i].x - own.x,
                                            path[i].y - own.y))
    if closest_index == len(path) - 1:
        return []
    nearest = path[closest_index]
    if hypot(nearest.x - own.x, nearest.y - own.y) > 0.12:
        return []
    if not safe_segment(world, own, nearest, opponent, clearance):
        return []
    # Fresh scans may reveal an obstacle absent when the path was created.
    for first, second in zip(path[closest_index:-1], path[closest_index + 1:]):
        if not safe_segment(world, first, second, opponent, clearance):
            return []
    return path


def recovery_step(world, own, opponent=None, clearance=0.0):
    """Find a safe escape direction when the nominal corridor cannot be entered."""
    options = []
    for length in (0.55, 0.3, 0.2, 0.15, 0.1):
        for index in range(24):
            yaw = -pi + 2 * pi * index / 24
            endpoint = Pose2(own.x + length * cos(yaw),
                             own.y + length * sin(yaw))
            if safe_segment(world, own, endpoint, opponent, clearance):
                wall = world.obstacle_clearance(endpoint.x, endpoint.y)
                map_margin = world.map_clearance(endpoint.x, endpoint.y)
                options.append((min(wall, 0.6) + min(map_margin, 0.6) -
                                0.05 * abs(angle_error(yaw, own.yaw)), yaw, length))
        if options:
            break
    return max(options)[1:] if options else None


def recovery_heading(world, own, opponent=None, clearance=0.0):
    step = recovery_step(world, own, opponent, clearance)
    return step[0] if step else None


def local_rollout(world, own, global_path, opponent=None, clearance=0.0,
                  weight=0.0, max_speed=0.5, horizon=2.0, dt=0.2,
                  opponent_velocity=(0.0, 0.0), previous_omega=None):
    if not global_path:
        return []
    lookahead = global_path[-1]
    for point in global_path:
        if hypot(point.x - own.x, point.y - own.y) >= 0.45:
            lookahead = point
            break
    # A Euclidean lookahead can jump across a U-turn or a corridor corner.
    # Follow the first reachable bend before aiming farther down the route.
    first_step = next((point for point in global_path
                       if hypot(point.x - own.x, point.y - own.y)
                       >= 0.6 * world.resolution), None)
    if first_step is not None:
        first_bearing = atan2(first_step.y - own.y, first_step.x - own.x)
        far_bearing = atan2(lookahead.y - own.y, lookahead.x - own.x)
        if abs(angle_error(first_bearing, far_bearing)) > 0.7:
            lookahead = first_step
    initial_distance = hypot(lookahead.x - own.x, lookahead.y - own.y)
    best = None
    source_cell = world.cell(own.x, own.y)
    source_blocked = source_cell in world.occupied
    initial_obstacle_clearance = world.obstacle_clearance(own.x, own.y)
    initial_map_clearance = world.map_clearance(own.x, own.y)
    initial_opponent_distance = (hypot(own.x - opponent.x, own.y - opponent.y)
                                 if opponent is not None else float("inf"))
    for velocity in (0.0, max_speed * 0.3, max_speed * 0.6, max_speed):
        for omega in (-1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2):
            x, y, yaw = own.x, own.y, own.yaw
            poses = [own]
            min_clearance = float("inf")
            min_wall_clearance = float("inf")
            for step in range(int(horizon / dt)):
                x += velocity * cos(yaw) * dt
                y += velocity * sin(yaw) * dt
                yaw = angle_error(yaw + omega * dt, 0)
                # The robot can physically stand in a cell marked occupied by
                # conservative rasterization. Permit motion inside that one
                # source cell so it can reach a genuinely free neighbor.
                wall_clearance = world.obstacle_clearance(
                    x, y, world.robot_radius + 0.2)
                map_clearance = world.map_clearance(x, y)
                safe_map_clearance = world.robot_radius + 0.1
                if (map_clearance < safe_map_clearance and
                        (initial_map_clearance >= safe_map_clearance or
                         map_clearance < initial_map_clearance - 0.01)):
                    break
                min_wall_clearance = min(min_wall_clearance, wall_clearance)
                if (wall_clearance < world.robot_radius - 0.01 and
                        (initial_obstacle_clearance >= world.robot_radius - 0.01 or
                         wall_clearance < initial_obstacle_clearance - 0.01)):
                    break
                if world.blocked(x, y):
                    if not source_blocked or hypot(x - own.x, y - own.y) > 0.5:
                        break
                    if (wall_clearance < 0.19 and
                            (initial_obstacle_clearance >= 0.19 or
                             wall_clearance < initial_obstacle_clearance - 0.01)):
                        break
                if opponent is not None:
                    future_x = opponent.x + opponent_velocity[0] * (step + 1) * dt
                    future_y = opponent.y + opponent_velocity[1] * (step + 1) * dt
                    distance = hypot(x - future_x, y - future_y)
                    if (distance < clearance and
                            (initial_opponent_distance >= clearance or
                             distance < min(0.45, initial_opponent_distance) - 0.01)):
                        break
                    min_clearance = min(min_clearance, distance)
                poses.append(Pose2(x, y, yaw))
                if hypot(x - lookahead.x, y - lookahead.y) <= 0.06:
                    break
            # A short safe prefix is useful because this planner refreshes at
            # 5 Hz. Reject only trajectories too short to control reliably.
            if len(poses) < 3:
                continue
            end = poses[-1]
            remaining = hypot(end.x - lookahead.x, end.y - lookahead.y)
            heading = abs(angle_error(atan2(lookahead.y - end.y,
                                            lookahead.x - end.x), end.yaw))
            progress = initial_distance - remaining
            threat = weight * max(0, 1.2 - min_clearance) if opponent else 0.0
            score = (4 * progress - remaining - 0.35 * heading - threat
                     - 0.2 * (horizon - (len(poses) - 1) * dt))
            safe_clearance = world.robot_radius + 0.09
            if source_blocked:
                end_clearance = world.obstacle_clearance(
                    end.x, end.y, world.robot_radius + 0.2)
                score += 6.0 * (min(safe_clearance, end_clearance) -
                                min(safe_clearance, initial_obstacle_clearance))
            else:
                score -= 5.0 * max(0.0, safe_clearance - min_wall_clearance)
            score -= 0.08 * abs(omega)
            if previous_omega is not None:
                score -= 0.2 * abs(omega - previous_omega)
            if velocity == 0:
                score -= 0.6
            if best is None or score > best[0]:
                best = score, poses, velocity
    if best is None:
        return []
    if best[2] == 0 and initial_distance > 0.05:
        # Give the MPC gate one stable absolute heading. Repeated sampled spin
        # paths used to alternate between clockwise and counterclockwise.
        bearing = atan2(lookahead.y - own.y, lookahead.x - own.x)
        if abs(angle_error(bearing, own.yaw)) < 0.15:
            return []
        return [own, Pose2(own.x, own.y, bearing)]
    return best[1]
