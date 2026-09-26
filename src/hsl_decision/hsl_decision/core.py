"""Pure behavior logic; ROS transport lives in node.py."""

from dataclasses import dataclass
from math import atan2, hypot


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


class DecisionPolicy:
    def __init__(self, role, opponent_start, *, own_start=None, pose_timeout=0.5,
                 scan_timeout=1.0, opponent_timeout=1.0, switch_margin=0.15,
                 min_dwell=0.5, evade_distance=1.5, capture_distance=0.8,
                 danger_weight=2.0, goal_weight=1.0):
        if role not in ("explorer", "guardian"):
            raise ValueError("role must be explorer or guardian")
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
            if distance_to_polygon(obs.own, self.goal_polygon) <= 0.178:
                self.previous = STOP
                return Decision(STOP, None, 0, 0, 0, 0,
                                "guardian start area reached")
            danger = max(0.0, 1.0 - distance / self.evade_distance)
            scores = {GOAL: self.goal_weight, EVADE: self.danger_weight * danger + 0.35}
            if obs.map_stamp <= 0:
                scores[EXPLORE] = self.goal_weight + 0.05
            chosen = self._select(scores, obs.now)
            if chosen == EVADE:
                dx = obs.own.x - obs.opponent.x
                dy = obs.own.y - obs.opponent.y
                norm = max(hypot(dx, dy), 0.01)
                target = Pose2(obs.own.x + 1.2 * dx / norm,
                               obs.own.y + 1.2 * dy / norm,
                               atan2(dy, dx))
                result = Decision(EVADE, target, 0.25, 0.50, 0.85, 6.0,
                                  "opponent within evasion range")
            elif chosen == EXPLORE:
                result = Decision(EXPLORE, self.goal, 0.35, 0.35, 0.65, 3.0,
                                  "map not available")
            else:
                result = Decision(GOAL, self.goal, 0.35, 0.50, 0.65, 4.0,
                                  "moving toward guardian start")
        elif not opponent_fresh:
            chosen = SEARCH
            result = Decision(SEARCH, None, 0.3, 0.35, 0.36, 1.0,
                              "opponent track missing")
        else:
            scores = {PURSUE: 1.0, CAPTURE: 1.0 + max(0.0, 1.0 - distance / self.capture_distance)}
            chosen = self._select(scores, obs.now)
            dx = obs.own.x - obs.opponent.x
            dy = obs.own.y - obs.opponent.y
            norm = max(hypot(dx, dy), 0.01)
            target = Pose2(obs.opponent.x + 0.41 * dx / norm,
                           obs.opponent.y + 0.41 * dy / norm,
                           atan2(-dy, -dx))
            result = Decision(chosen, target, 0.08 if chosen == CAPTURE else 0.25,
                              0.25 if chosen == CAPTURE else 0.55, 0.36, 1.5,
                              "orient for capture" if chosen == CAPTURE else "pursuing opponent")
        self.previous = result.behavior
        return result
