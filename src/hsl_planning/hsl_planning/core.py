"""Pure map projection, A* and sampled differential-drive local planning."""

from dataclasses import dataclass
from heapq import heappop, heappush
from math import atan2, ceil, cos, hypot, pi, sin, sqrt

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


def circle_segment_intersection(start, end, radius):
    """Return the first point where a segment exits a circle at the origin."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    a = dx * dx + dy * dy
    if a <= 1e-12:
        return None
    b = 2.0 * (start[0] * dx + start[1] * dy)
    c = start[0] * start[0] + start[1] * start[1] - radius * radius
    discriminant = b * b - 4.0 * a * c
    if discriminant < -1e-10:
        return None
    root = sqrt(max(0.0, discriminant))
    intersections = [(-b - root) / (2.0 * a),
                     (-b + root) / (2.0 * a)]
    valid = [max(0.0, min(1.0, value)) for value in intersections
             if -1e-9 <= value <= 1.0 + 1e-9]
    if not valid:
        return None
    t = min(valid)
    return start[0] + t * dx, start[1] + t * dy


def regulated_pure_pursuit_guidance(world, own, route, speed=0.0,
                                    opponent=None, clearance=0.0,
                                    lookahead_time=1.5,
                                    min_lookahead=0.5,
                                    max_lookahead=1.3,
                                    max_curvature=2.6,
                                    safety_margin=0.12,
                                    diagnostics=None):
    """Build a collision-checked RPP arc through a velocity-scaled carrot.

    Nav2's Regulated Pure Pursuit computes a lookahead point in the robot
    frame, follows the circle through that point, and regulates speed from
    curvature and obstacle clearance. This adapts that geometry to the
    existing path-following MPC: return the checked circular arc as its local
    reference instead of issuing velocity commands from the planner.
    """
    def reject(reason):
        if diagnostics is not None:
            key = "rpp_" + reason
            diagnostics[key] = diagnostics.get(key, 0) + 1
        return []

    if len(route) < 2:
        return reject("short_route")
    min_lookahead = max(0.05, float(min_lookahead))
    max_lookahead = max(min_lookahead, float(max_lookahead))
    lookahead_time = max(0.0, float(lookahead_time))
    lookahead = max(min_lookahead,
                    min(max_lookahead, abs(float(speed)) * lookahead_time))

    # Prune the path at its closest point to the robot, like Nav2's
    # transformGlobalPlan, then express the remaining plan in the robot frame.
    nearest = None
    nearest_distance = float("inf")
    for index, (first, second) in enumerate(zip(route, route[1:])):
        dx, dy = second.x - first.x, second.y - first.y
        length2 = dx * dx + dy * dy
        if length2 <= 1e-12:
            continue
        fraction = max(0.0, min(1.0,
                        ((own.x - first.x) * dx + (own.y - first.y) * dy) /
                        length2))
        px, py = first.x + fraction * dx, first.y + fraction * dy
        distance = hypot(own.x - px, own.y - py)
        if distance < nearest_distance:
            nearest_distance = distance
            nearest = (index, px, py)
    if nearest is None or nearest_distance > 0.45:
        return reject("route_miss")

    index, px, py = nearest
    world_plan = [(px, py)]
    world_plan.extend((point.x, point.y) for point in route[index + 1:])
    c, s = cos(own.yaw), sin(own.yaw)
    local_plan = [(c * (x - own.x) + s * (y - own.y),
                   -s * (x - own.x) + c * (y - own.y))
                  for x, y in world_plan]

    # If the checked arc would hit an obstacle, progressively shorten the
    # carrot before falling back to the planner's tight-space manoeuvres.
    lookaheads = []
    for scale in (1.0, 0.8, 0.6):
        candidate = max(min_lookahead, lookahead * scale)
        if not lookaheads or candidate < lookaheads[-1] - 1e-6:
            lookaheads.append(candidate)
    last_reason = "no_carrot"
    for distance in lookaheads:
        carrot = None
        for point_index, point in enumerate(local_plan):
            if hypot(point[0], point[1]) + 1e-9 < distance:
                continue
            if point_index == 0:
                carrot = point
            else:
                carrot = circle_segment_intersection(
                    local_plan[point_index - 1], point, distance) or point
            break
        if carrot is None:
            carrot = local_plan[-1]
        carrot_distance = hypot(carrot[0], carrot[1])
        if carrot_distance < 0.18:
            last_reason = "short_carrot"
            continue
        if carrot[0] <= 0.02:
            last_reason = "carrot_behind"
            continue

        curvature = 2.0 * carrot[1] / max(carrot_distance * carrot_distance,
                                         1e-9)
        if abs(curvature) > max_curvature:
            last_reason = "curvature"
            continue
        if abs(curvature) < 1e-4:
            arc_length = carrot_distance
        else:
            angle = 2.0 * atan2(carrot[1], carrot[0])
            arc_length = angle / curvature
        if arc_length <= 0.0 or arc_length > 2.0:
            last_reason = "arc_length"
            continue

        steps = max(2, ceil(arc_length / 0.05))
        path = [own]
        for step in range(1, steps + 1):
            along = arc_length * step / steps
            if abs(curvature) < 1e-4:
                x = own.x + along * c
                y = own.y + along * s
            else:
                heading = own.yaw + curvature * along
                x = own.x + (sin(heading) - sin(own.yaw)) / curvature
                y = own.y - (cos(heading) - cos(own.yaw)) / curvature
            point = Pose2(x, y, own.yaw + curvature * along)
            if not safe_segment(world, path[-1], point, opponent, clearance,
                                safety_margin):
                last_reason = "collision"
                break
            path.append(point)
        if len(path) == steps + 1:
            if diagnostics is not None:
                diagnostics["rpp_accepted"] = diagnostics.get("rpp_accepted", 0) + 1
                diagnostics["rpp_lookahead_m"] = distance
                diagnostics["rpp_curvature_1pm"] = curvature
            return path
    return reject(last_reason)


def max_curve_heading_error(world, own):
    """Scale the admissible curve entry angle with measured free space."""
    available_room = (min(world.obstacle_clearance(own.x, own.y),
                          world.map_clearance(own.x, own.y)) -
                      world.robot_radius)
    return 1.45 + min(0.65, max(0.0, available_room - 0.18) * 1.5)


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


def checked_cubic(world, own, end, end_heading, opponent=None, clearance=0.0,
                  handle=None):
    """Return a forward cubic only when its swept path has tracking room."""
    dx, dy = end.x - own.x, end.y - own.y
    distance = hypot(dx, dy)
    if distance < 0.45:
        return [], "short_distance"
    heading = atan2(dy, dx)
    error = angle_error(heading, own.yaw)
    initial_wall = world.obstacle_clearance(own.x, own.y)
    initial_map = world.map_clearance(own.x, own.y)
    available_room = min(initial_wall, initial_map) - world.robot_radius
    # Expand the admissible turn angle only as measured wall/map clearance
    # increases; the complete swept path is still checked below.
    max_heading_error = max_curve_heading_error(world, own)
    if abs(error) < 0.18 or abs(error) > max_heading_error:
        return [], "heading_small" if abs(error) < 0.18 else "heading_large"
    # MPC does not follow the reference exactly. Reserve tracking room around
    # every moving turn, especially beside walls and the arena boundary. If a
    # robot starts below that reserve, only accept a curve that measurably
    # increases both wall and arena clearance as it leaves the tight spot.
    curve_margin = world.robot_radius + 0.18
    # The first control point follows the robot heading; the last one joins
    # the original collision-checked corridor tangentially.
    max_handle = 2.0 if available_room > 0.35 else 1.2
    handle = (min(max_handle, max(0.32, 0.40 * distance))
              if handle is None else min(max_handle, max(0.18, handle)))
    p1 = (own.x + handle * cos(own.yaw), own.y + handle * sin(own.yaw))
    p2 = (end.x - handle * cos(end_heading),
          end.y - handle * sin(end_heading))
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
        travel = hypot(x - own.x, y - own.y)
        wall = world.obstacle_clearance(x, y)
        map_margin = world.map_clearance(x, y)
        gain = min(0.04, 0.15 * travel)
        if (wall < curve_margin and
                (initial_wall >= curve_margin or
                 wall + 0.005 < initial_wall + gain)):
            return [], "curve_clearance"
        if (map_margin < curve_margin and
                (initial_map >= curve_margin or
                 map_margin + 0.005 < initial_map + gain)):
            return [], "curve_clearance"
        if not safe_segment(world, path[-1], point, opponent, clearance):
            return [], "curve_segment"
        if len(path) >= 2:
            first, middle = path[-2:]
            ax, ay = middle.x - first.x, middle.y - first.y
            bx, by = point.x - middle.x, point.y - middle.y
            chord = hypot(point.x - first.x, point.y - first.y)
            curvature = (2.0 * abs(ax * by - ay * bx) /
                         max(hypot(ax, ay) * hypot(bx, by) * chord, 1e-9))
            if curvature > 2.6:
                return [], "curvature"
        path.append(point)
    return path, "accepted"


def curved_guidance(world, own, straight, opponent=None, clearance=0.0,
                    diagnostics=None):
    """Join the current heading to a visible corridor without stopping to turn."""
    if len(straight) < 3:
        reason, path = "short_path", []
    else:
        end = straight[-1]
        heading = atan2(end.y - own.y, end.x - own.x)
        path, reason = checked_cubic(world, own, end, heading,
                                     opponent, clearance)
    if diagnostics is not None:
        diagnostics[reason] = diagnostics.get(reason, 0) + 1
    return path or straight


def route_curve_guidance(world, own, route, straight, opponent=None,
                         clearance=0.0, diagnostics=None, max_distance=1.8):
    """Try a checked curve through the next global bend when sight ends early."""
    if not route or len(straight) < 2:
        return []
    visible = straight[-1]
    visible_distance = hypot(visible.x - own.x, visible.y - own.y)
    visible_heading = atan2(visible.y - own.y, visible.x - own.x)
    heading_error = abs(angle_error(visible_heading, own.yaw))
    available_room = (min(world.obstacle_clearance(own.x, own.y),
                          world.map_clearance(own.x, own.y)) -
                      world.robot_radius)
    handle_limit = 2.0 if available_room > 0.35 else 1.2
    if heading_error > 1.45 and available_room > 0.35:
        # Near-reversing starts need a longer checked turn that first moves
        # along the robot's current heading before joining the route.
        max_distance = max(max_distance, 4.0)
    # A long collision-free chord can still force the controller to rotate
    # in place. Try the next route bend first when its chord is poorly aligned.
    if visible_distance >= 0.75 and heading_error < 0.45:
        return []
    near = min(range(len(route)), key=lambda i: hypot(route[i].x - visible.x,
                                                       route[i].y - visible.y))
    candidates = []
    for index in range(near + 1, len(route)):
        point = route[index]
        distance = hypot(point.x - own.x, point.y - own.y)
        if distance > max_distance:
            break
        if distance < max(0.45, visible_distance + 0.15):
            continue
        if candidates and hypot(point.x - route[candidates[-1]].x,
                                point.y - route[candidates[-1]].y) < 0.2:
            continue
        candidates.append(index)
    for index in reversed(candidates[-6:]):
        before = route[max(0, index - 1)]
        after = route[min(len(route) - 1, index + 1)]
        end_heading = atan2(after.y - before.y, after.x - before.x)
        path, reason = checked_cubic(world, own, route[index], end_heading,
                                     opponent, clearance)
        if diagnostics is not None:
            key = "route_" + reason
            diagnostics[key] = diagnostics.get(key, 0) + 1
        if path:
            return path
        if reason in ("curve_clearance", "curvature"):
            distance = hypot(route[index].x - own.x, route[index].y - own.y)
            # The usual middle handles can both fail in a narrow but open
            # corridor. Try the two extremes as well; checked_cubic still
            # enforces curvature and swept-clearance limits on every sample.
            handles = (max(0.18, 0.30 * distance),
                       min(handle_limit, 0.45 * distance),
                       min(handle_limit, 0.55 * distance),
                       min(handle_limit, 0.66 * distance),
                       max(0.18, 0.18 * distance),
                       min(handle_limit, 0.82 * distance))
            for handle in dict.fromkeys(handles):
                path, reason = checked_cubic(
                    world, own, route[index], end_heading, opponent,
                    clearance, handle=handle)
                if diagnostics is not None:
                    key = "route_handle_" + reason
                    diagnostics[key] = diagnostics.get(key, 0) + 1
                if path:
                    return path
    return []


def reusable_local_guidance(world, own, path, opponent=None, clearance=0.0,
                            safety_margin=0.12):
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
    if not safe_segment(world, own, nearest, opponent, clearance,
                        safety_margin):
        return []
    # Fresh scans may reveal an obstacle absent when the path was created.
    for first, second in zip(path[closest_index:-1], path[closest_index + 1:]):
        if not safe_segment(world, first, second, opponent, clearance,
                            safety_margin):
            return []
    return path


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


def recovery_heading(world, own, opponent=None, clearance=0.0):
    step = recovery_step(world, own, opponent, clearance)
    return step[0] if step else None


def local_rollout(world, own, global_path, opponent=None, clearance=0.0,
                  weight=0.0, max_speed=0.5, horizon=2.0, dt=0.2,
                  opponent_velocity=(0.0, 0.0), previous_omega=None,
                  safety_margin=0.18):
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
    required_clearance = world.robot_radius + max(0.0, safety_margin)
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
                # Permit a route to leave a locally tight start, but only if it
                # measurably increases wall/map clearance on its way out.
                wall_clearance = world.obstacle_clearance(
                    x, y, world.robot_radius + 0.2)
                map_clearance = world.map_clearance(x, y)
                travel = hypot(x - own.x, y - own.y)
                gain = min(0.04, 0.15 * travel)
                if (map_clearance < required_clearance and
                        (initial_map_clearance >= required_clearance or
                         map_clearance + 0.005 < initial_map_clearance + gain)):
                    break
                min_wall_clearance = min(min_wall_clearance, wall_clearance)
                if (wall_clearance < required_clearance and
                        (initial_obstacle_clearance >= required_clearance or
                         wall_clearance + 0.005 <
                         initial_obstacle_clearance + gain)):
                    break
                if world.blocked(x, y):
                    if not source_blocked or hypot(x - own.x, y - own.y) > 0.5:
                        break
                    if (wall_clearance < required_clearance and
                            (initial_obstacle_clearance >= required_clearance or
                             wall_clearance + 0.005 <
                             initial_obstacle_clearance + gain)):
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
            safe_clearance = required_clearance
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
