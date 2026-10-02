"""Pure behavior logic; ROS transport lives in node.py."""

from dataclasses import dataclass
from math import atan2, hypot, isfinite, sqrt


WAIT, STOP, GOAL, EXPLORE, EVADE, SEARCH, PURSUE, CAPTURE = range(8)


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Observation:
    now: float
    own: Pose2 | None
    own_stamp: float
    opponent: Pose2 | None
    opponent_stamp: float
    scan_stamp: float
    map_stamp: float
    allowed: bool
    opponent_velocity: tuple[float, float] = (0.0, 0.0)


@dataclass(frozen=True)
class Decision:
    behavior: int
    target: Pose2 | None
    tolerance: float
    max_speed: float
    opponent_clearance: float
    opponent_cost_weight: float
    reason: str


def polygon_center(flat_vertices):
    if len(flat_vertices) < 6 or len(flat_vertices) % 2:
        raise ValueError("a start area needs at least three x,y vertices")
    count = len(flat_vertices) // 2
    return Pose2(sum(flat_vertices[::2]) / count, sum(flat_vertices[1::2]) / count)


GOAL_CENTER_TOLERANCE = 0.08  # Matches the native MPPI XY goal tolerance.


def start_center_reached(point, flat_vertices):
    center = polygon_center(flat_vertices)
    return hypot(point.x - center.x, point.y - center.y) <= GOAL_CENTER_TOLERANCE


def distance_to_polygon(point, flat_vertices):
    """Zero inside a polygon; shortest distance to its boundary outside."""
    vertices = list(zip(flat_vertices[::2], flat_vertices[1::2]))
    inside = False
    best = float("inf")
    for index, (ax, ay) in enumerate(vertices):
        bx, by = vertices[(index + 1) % len(vertices)]
        if (ay > point.y) != (by > point.y):
            crossing = ax + (point.y - ay) * (bx - ax) / (by - ay)
            if point.x < crossing:
                inside = not inside
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((point.x - ax) * dx + (point.y - ay) * dy)
                              / max(dx * dx + dy * dy, 1e-12)))
        best = min(best, hypot(point.x - ax - t * dx, point.y - ay - t * dy))
    return 0.0 if inside else best


def intercept_point(guardian, explorer, velocity, pursuer_speed=0.3,
                    horizon=2.0):
    """Bounded constant-velocity interception estimate in the map frame."""
    rx, ry = explorer.x - guardian.x, explorer.y - guardian.y
    vx, vy = velocity
    a = vx * vx + vy * vy - pursuer_speed * pursuer_speed
    b = 2.0 * (rx * vx + ry * vy)
    c = rx * rx + ry * ry
    roots = []
    if abs(a) < 1e-9:
        if abs(b) > 1e-9:
            roots.append(-c / b)
    else:
        discriminant = b * b - 4.0 * a * c
        if discriminant >= 0:
            roots.extend(((-b - sqrt(discriminant)) / (2.0 * a),
                          (-b + sqrt(discriminant)) / (2.0 * a)))
    positive = [value for value in roots if value > 0]
    estimate = min(positive) if positive else hypot(rx, ry) / pursuer_speed
    lead_time = min(horizon, max(0.0, estimate))
    return Pose2(explorer.x + vx * lead_time,
                 explorer.y + vy * lead_time, explorer.yaw)


class DecisionPolicy:
    def __init__(self, role, opponent_start, *, own_start=None, pose_timeout=0.5,
                 scan_timeout=1.0, opponent_timeout=1.0, switch_margin=0.15,
                 min_dwell=0.5, evade_distance=1.8, capture_distance=0.8,
                 danger_weight=2.0, goal_weight=1.0, own_max_speed=0.5):
        if role not in ("explorer", "guardian"):
            raise ValueError("role must be explorer or guardian")
        if not isfinite(own_max_speed) or own_max_speed <= 0:
            raise ValueError("own_max_speed must be finite and positive")
        self.own_max_speed = own_max_speed
        self.role = role
        self.goal = polygon_center(opponent_start)
        self.goal_polygon = opponent_start
        if own_start is not None:
            polygon_center(own_start)
        self.pose_timeout = pose_timeout
        self.scan_timeout = scan_timeout
        self.opponent_timeout = opponent_timeout
        self.switch_margin = switch_margin
        self.min_dwell = min_dwell
        self.evade_distance = evade_distance
        self.capture_distance = capture_distance
        self.danger_weight = danger_weight
        self.goal_weight = goal_weight
        self.previous = WAIT
        self.last_switch = float("-inf")
        self.search_anchor = None
        self.search_exploring = False

    def _select(self, scores, now):
        best = max(scores, key=scores.get)
        if (self.previous in scores and best != self.previous
                and now - self.last_switch < self.min_dwell
                and scores[best] < scores[self.previous] + self.switch_margin):
            return self.previous
        if best != self.previous:
            self.last_switch = now
        return best

    def step(self, obs):
        if not obs.allowed:
            self.previous = WAIT
            self.search_anchor = None
            self.search_exploring = False
            return Decision(WAIT, None, 0, 0, 0, 0, "waiting for start permission")
        if (obs.own is None or obs.now - obs.own_stamp > self.pose_timeout
                or obs.now - obs.scan_stamp > self.scan_timeout):
            self.previous = STOP
            return Decision(STOP, None, 0, 0, 0, 0, "own pose or scan is stale")

        opponent_fresh = (obs.opponent is not None
                          and obs.now - obs.opponent_stamp <= self.opponent_timeout)
        distance = hypot(obs.opponent.x - obs.own.x,
                         obs.opponent.y - obs.own.y) if opponent_fresh else float("inf")

        if self.role == "explorer":
            if start_center_reached(obs.own, self.goal_polygon):
                self.previous = STOP
                return Decision(STOP, None, 0, 0, 0, 0,
                                "guardian start center reached")
            threat_age = obs.now - obs.opponent_stamp
            threat_pose = obs.opponent if obs.opponent is not None and threat_age <= 2.0 else None
            threat_distance = (hypot(threat_pose.x - obs.own.x,
                                     threat_pose.y - obs.own.y)
                               if threat_pose is not None else float("inf"))
            danger = max(0.0, 1.0 - threat_distance / self.evade_distance)
            scores = {GOAL: self.goal_weight,
                      EVADE: self.danger_weight * danger + 0.35 +
                      (0.3 if self.previous == EVADE and threat_pose else 0.0)}
            if obs.map_stamp <= 0:
                scores[EXPLORE] = self.goal_weight + 0.05
            chosen = self._select(scores, obs.now)
            if chosen == EVADE:
                # Keep the real objective as the target. The planner applies
                # higher opponent clearance/cost and lets A* first escape a
                # violated safety radius before continuing toward the goal.
                result = Decision(EVADE, self.goal, 0.35, 1.0, 1.0, 8.0,
                                  "moving to goal along a route clear of the guardian")
            elif chosen == EXPLORE:
                result = Decision(EXPLORE, self.goal, 0.35, 1.0, 0.65, 3.0,
                                  "map not available")
            else:
                result = Decision(GOAL, self.goal, 0.35, 1.0, 0.85, 6.0,
                                  "moving toward guardian start around danger")
        elif not opponent_fresh:
            anchor = obs.opponent if obs.opponent else self.goal
            if obs.opponent is not None:
                # When LiDAR loses line of sight, search slightly ahead of the
                # last measured point instead of repeatedly driving to where a
                # moving opponent was. Bound time, not opponent speed: velocity
                # is the last observed estimate and is never clamped to a
                # guessed rival speed.
                age = max(0.0, obs.now - obs.opponent_stamp)
                lead = min(max(0.0, age - self.opponent_timeout), 1.0)
                anchor = Pose2(obs.opponent.x + obs.opponent_velocity[0] * lead,
                               obs.opponent.y + obs.opponent_velocity[1] * lead,
                               obs.opponent.yaw)
            if self.search_anchor != anchor:
                self.search_anchor = anchor
                self.search_exploring = False
            if hypot(anchor.x - obs.own.x, anchor.y - obs.own.y) <= 0.35:
                self.search_exploring = True
            result = Decision(SEARCH, None if self.search_exploring else anchor,
                              0.3, 1.0, 0.0, 0.0,
                              "sweeping known free space" if self.search_exploring
                              else "searching last seen position" if obs.opponent
                              else "searching opponent start area")
        else:
            self.search_anchor = None
            self.search_exploring = False
            # Only the referee can declare a capture: distance and heading
            # alone cannot rule out a wall between the robots.
            scores = {PURSUE: 1.0, CAPTURE: 1.0 + max(0.0, 1.0 - distance / self.capture_distance)}
            chosen = self._select(scores, obs.now)
            predicted = (intercept_point(obs.own, obs.opponent,
                                         obs.opponent_velocity,
                                         pursuer_speed=self.own_max_speed)
                         if chosen == PURSUE else obs.opponent)
            dx = obs.own.x - predicted.x
            dy = obs.own.y - predicted.y
            norm = max(hypot(dx, dy), 0.01)
            target = Pose2(predicted.x + 0.42 * dx / norm,
                           predicted.y + 0.42 * dy / norm,
                           atan2(-dy, -dx))
            result = Decision(chosen, target, 0.015 if chosen == CAPTURE else 0.25,
                              1.0, 0.0, 0.0,
                              "orient for capture" if chosen == CAPTURE else
                              "intercepting moving opponent")
        self.previous = result.behavior
        return result
