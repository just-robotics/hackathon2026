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


def navigation_obstacles(grid_points, map_points, scan_points, enemy=None):
    """Exclude a tracked robot from observed clouds, never from known walls."""
    def without_robot(points):
        return [point for point in points
                if enemy is None or
                hypot(point[0] - enemy.x, point[1] - enemy.y) > 0.45]

    return (list(grid_points) + without_robot(map_points),
            without_robot(scan_points))


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
    """Goal/evasion turns can be useful too; caller bounds their duration."""
    return ((role == "guardian" and behavior in (6, 7)) or
            (role == "explorer" and behavior in (2, 4)))


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

    def frontier_candidates(self, own, destination=None, tie_seed=0, min_travel=0.0,
                            avoid=None):
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
        return [item[2] for item in sorted(candidates, key=lambda item: item[:2])]

    def frontier(self, own, destination=None, tie_seed=0, min_travel=0.0,
                 avoid=None):
        candidates = self.frontier_candidates(own, destination, tie_seed, min_travel, avoid)
        return candidates[0] if candidates else None


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
    target_point = world.point(target)
    if (opponent is not None and clearance > 0 and source != target and
            source_opponent_distance >= clearance and
            hypot(target_point.x - opponent.x, target_point.y - opponent.y) < clearance):
        # A forbidden endpoint cannot become reachable by exploring more cells.
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


def reachable_frontier_route(world, own, destination, opponent=None, clearance=0.0,
                             weight=0.0, tie_seed=0, avoid=None, max_attempts=30):
    """Try ranked safe frontier endpoints before abandoning navigation for recovery."""
    attempts = 0
    for point in world.frontier_candidates(own, destination, tie_seed, min_travel=0.6,
                                           avoid=avoid):
        if not world.inside_map(point.x, point.y, world.robot_radius + 0.1):
            continue
        if (opponent is not None and clearance > 0 and
                hypot(point.x - opponent.x, point.y - opponent.y) < clearance):
            continue
        attempts += 1
        route = astar(world, own, point, opponent, clearance, weight,
                      tie_seed=tie_seed, avoid=avoid)
        if route:
            return route
        if attempts >= max_attempts:
            break
    return []


def evade_objective_route(world, own, opponent, goal, prediction=None, clearance=1.0,
                          weight=6.0, tie_seed=0, candidate=()):
    """Keep a reachable objective while departing away from the observed threat."""
    if goal is None:
        return []
    initial = hypot(own.x - opponent.x, own.y - opponent.y)
    minimum = min(initial, clearance)

    def safe_departure(route):
        if not route:
            return False
        # The predicted exclusion zone must not allow a path through the
        # currently observed robot. Check the whole route against both inputs.
        if any(hypot(point.x - opponent.x, point.y - opponent.y) < minimum - 0.02
               for point in route[1:]):
            return False
        departure = next((point for point in route
                          if hypot(point.x - own.x, point.y - own.y) > 0.2), None)
        return (departure is None or
                (departure.x - own.x) * (own.x - opponent.x) +
                (departure.y - own.y) * (own.y - opponent.y) >= 0)

    if safe_departure(candidate):
        return list(candidate)
    route = astar(world, own, goal, prediction or opponent, clearance, weight,
                  tie_seed=tie_seed)
    return route if safe_departure(route) else []


def evade_target(world, own, opponent, goal, clearance=1.0, safety_margin=0.12):
    """A reachable short departure that never initially heads into the threat."""
    away_x, away_y = own.x - opponent.x, own.y - opponent.y
    initial = hypot(away_x, away_y)
    options = []
    for length in (0.9, 0.6, 0.3):
        for index in range(32):
            yaw = 2 * pi * index / 32
            dx, dy = length * cos(yaw), length * sin(yaw)
            if dx * away_x + dy * away_y < -1e-6:
                continue
            point = Pose2(own.x + dx, own.y + dy, yaw)
            if not safe_segment(world, own, point, opponent, clearance, safety_margin):
                continue
            separation_gain = hypot(point.x - opponent.x, point.y - opponent.y) - initial
            goal_gain = (hypot(own.x - goal.x, own.y - goal.y) -
                         hypot(point.x - goal.x, point.y - goal.y)) if goal else 0.0
            options.append((2 * separation_gain + 0.4 * goal_gain, point))
        if options:
            break
    return max(options, key=lambda item: item[0])[1] if options else None


def capture_goal(world, own, opponent):
    if (0.36 < hypot(own.x - opponent.x, own.y - opponent.y) < 0.45 and
            world.clear_line_3d(own, opponent)):
        return Pose2(own.x, own.y, atan2(opponent.y - own.y, opponent.x - own.x))
    options = []
    radial = atan2(own.y - opponent.y, own.x - opponent.x)
    for angle in [radial] + [2 * pi * index / 16 for index in range(16)]:
        pose = Pose2(opponent.x + 0.39 * cos(angle),
                     opponent.y + 0.39 * sin(angle), angle + pi)
        if (world.blocked(pose.x, pose.y)
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


def moving_capture_goal(world, own, opponent, velocity, pursuer_speed=0.3):
    """Use a short reachable lead unless the prey is approaching head-on.

    Return the capture goal and the prey pose it refers to. Once already in
    capture range, face the observed prey rather than a future position.
    """
    dx, dy = opponent.x - own.x, opponent.y - own.y
    distance, speed = hypot(dx, dy), hypot(*velocity)
    if distance > 0.45 and speed > 1e-6:
        radial_cosine = (dx * velocity[0] + dy * velocity[1]) / (distance * speed)
        # Blend continuously: direct for head-on approach, full lead for
        # transverse/receding motion. Do not assume a maximum rival speed.
        fraction = min(1.0, max(0.0, 1.0 + radial_cosine))
        lead_time = min(1.0, distance / max(pursuer_speed, 1e-6)) * fraction
        predicted = Pose2(opponent.x + velocity[0] * lead_time,
                          opponent.y + velocity[1] * lead_time, opponent.yaw)
        if (lead_time > 1e-6 and
                world.inside_map(predicted.x, predicted.y, world.robot_radius + 0.07) and
                not world.blocked(predicted.x, predicted.y) and
                safe_segment(world, opponent, predicted, safety_margin=0.0)):
            target = capture_goal(world, own, predicted)
            if target is not None:
                return target, predicted
    return capture_goal(world, own, opponent), opponent


def reachable_intercept(world, own, opponent, predicted):
    """Do not pursue an extrapolation that runs through a known wall."""
    if (predicted is not None and
            world.inside_map(predicted.x, predicted.y, world.robot_radius + 0.07) and
            not world.blocked(predicted.x, predicted.y) and
            # Validate a forecast, not our own recovery manoeuvre. A prey
            # moving parallel to a wall need not increase its clearance by
            # the extra recovery margin. Keep the footprint and wall checks;
            # the guardian's route and MPPI rollout are checked separately.
            safe_segment(world, opponent, predicted, safety_margin=0.0)):
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
    if (hypot(route[-1].x - target.x, route[-1].y - target.y) > 0.3 and
            hypot(route[-1].x - own.x, route[-1].y - own.y) < 0.3):
        # A completed detour must retry the objective rather than hold its endpoint.
        return []
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
        # holding the same small clearance allowed tracking error to scrape
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


def continuous_short_goal_route(world, own, route, target, safety_margin=0.12):
    """Preserve a continuous pursuit goal when raster planning has one cell.

    MPPI needs at least two poses. Repeating the raster centre would hide a
    moved goal, so use the current pose and actual target only when their
    swept segment is safe. Longer routes keep their planned corridor.
    """
    if (len(route) == 1 and target is not None and
            safe_segment(world, own, target, safety_margin=safety_margin)):
        return [own, target]
    return route


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
