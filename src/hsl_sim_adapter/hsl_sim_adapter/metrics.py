"""Pure, run-scoped measurements from Gazebo truth and contact events."""

from math import atan2, hypot, isfinite, pi, sqrt

from hsl_decision.core import Pose2, distance_to_polygon


def timing_summary(values, budget_ms=None):
    if not values:
        return {"count": 0, "p95": None, "max": None,
                "over_budget": 0, "over_2s": 0}
    ordered = sorted(values)
    index = max(0, (95 * len(ordered) + 99) // 100 - 1)
    return {
        "count": len(values),
        "p95": round(ordered[index], 2),
        "max": round(ordered[-1], 2),
        "over_budget": sum(value > budget_ms for value in values)
        if budget_ms is not None else 0,
        "over_2s": sum(value >= 2000 for value in values),
    }


def capture_possible(guardian, explorer, grid):
    """Regulation geometry: 0.45 m, heading within 45 degrees, clear line."""
    if grid is None or guardian is None or explorer is None:
        return False
    dx, dy = explorer[0] - guardian[0], explorer[1] - guardian[1]
    if hypot(dx, dy) >= 0.45:
        return False
    error = (atan2(dy, dx) - guardian[2] + pi) % (2 * pi) - pi
    return abs(error) <= pi / 4 and grid.clear_line(
        guardian[:2], explorer[:2], endpoint_radius=0.1)


def duel_outcome(first_role, first_pose, second_pose, first_start, second_start, grid):
    """Return the first geometric result; the caller handles elapsed time."""
    if first_role == "explorer":
        explorer, guardian, goal_area = first_pose, second_pose, second_start
    elif first_role == "guardian":
        explorer, guardian, goal_area = second_pose, first_pose, first_start
    else:
        raise ValueError("first_role must be explorer or guardian")
    if capture_possible(guardian, explorer, grid):
        return "guardian_capture"
    if explorer is not None and distance_to_polygon(
            Pose2(explorer[0], explorer[1]), goal_area) <= 0.178:
        return "explorer_goal"
    return None


class RunMetrics:
    def __init__(self, max_speed=0.7):
        self.max_speed = max_speed
        self.active = False
        self.reset()

    def reset(self):
        self.started_at = None
        self.ended_at = None
        self.last_sample = None
        self.distance_m = 0.0
        self.pose_jumps = 0
        self.moving_s = 0.0
        self.turning_s = 0.0
        self.behavior_s = {}
        self.turning_by_behavior_s = {}
        self.idle_by_behavior_s = {}
        self.commanded_motion_s = 0.0
        self.commanded_while_still_s = 0.0
        self.planner_status_s = {}
        self.planner_failure_by_behavior_s = {}
        self.visible_s = 0.0
        self.planner_ok_s = 0.0
        self.accel_squared_s = 0.0
        self.angular_accel_squared_s = 0.0
        self.accel_time_s = 0.0
        self.starts = 0
        self.moving = False
        self.collisions = 0
        self.wall_collisions = 0
        self.robot_collisions = 0
        self.collision_points = []
        self.contact_active = False
        self.last_contact_at = float("-inf")
        self.capture_at = None
        self.goal_at = None
        self.wall_started_at = None
        self.wall_ended_at = None
        self.timings = {}
        self.last_stream_at = {}
        self.last_scan_at = None
        self.samples = []
        self.contacts = []
        self.timing_events = []

    def start(self, now, wall_now=None):
        self.reset()
        self.active = True
        self.started_at = now
        self.wall_started_at = wall_now

    def stop(self, now, wall_now=None):
        if self.active:
            self.ended_at = now
            self.wall_ended_at = wall_now
            self.active = False
        self.contact_active = False

    def record_timing(self, name, duration_ms, sim_now=None):
        if self.active and isfinite(duration_ms) and duration_ms >= 0:
            self.timings.setdefault(name, []).append(duration_ms)
            self.timing_events.append((sim_now, name, duration_ms))

    def observe_stream(self, name, wall_now, sim_now=None):
        if not self.active:
            return
        last = self.last_stream_at.get(name)
        if last is not None and wall_now > last:
            self.record_timing(name + "_period_wall", (wall_now - last) * 1000, sim_now)
        self.last_stream_at[name] = wall_now
        if name == "scan":
            self.last_scan_at = wall_now
        elif name == "command" and self.last_scan_at is not None:
            self.record_timing("scan_to_command_wall", (wall_now - self.last_scan_at) * 1000,
                               sim_now)

    def sample(self, now, x, y, speed, angular_speed, visible=False, planner_ok=False,
               behavior=None, command=None, planner_status=None):
        if not self.active:
            return
        self.samples.append((now, x, y, speed, angular_speed, visible, planner_ok,
                             behavior, command, planner_status))
        if self.last_sample is not None:
            last_t, last_x, last_y, last_speed, last_angular = self.last_sample
            dt = now - last_t
            if 0 < dt <= 1.0:
                step = hypot(x - last_x, y - last_y)
                # Gazebo's `gz model` teleport should not count as driven distance.
                if step / dt > max(1.5, 2 * self.max_speed):
                    self.pose_jumps += 1
                else:
                    self.distance_m += step
                self.moving_s += dt if speed >= 0.05 else 0.0
                motion_plan = planner_ok or planner_status == "RECOVERY_MPPI"
                turning = speed < 0.05 and abs(angular_speed) >= 0.15 and motion_plan
                self.turning_s += dt if turning else 0.0
                still = speed < 0.05 and abs(angular_speed) < 0.15
                if behavior is not None:
                    self.behavior_s[behavior] = self.behavior_s.get(behavior, 0.0) + dt
                    if turning:
                        self.turning_by_behavior_s[behavior] = (
                            self.turning_by_behavior_s.get(behavior, 0.0) + dt)
                    if still and planner_ok:
                        self.idle_by_behavior_s[behavior] = (
                            self.idle_by_behavior_s.get(behavior, 0.0) + dt)
                if command is not None:
                    commanded = abs(command[0]) >= 0.05 or abs(command[1]) >= 0.15
                    if commanded:
                        self.commanded_motion_s += dt
                        if still:
                            self.commanded_while_still_s += dt
                self.visible_s += dt if visible else 0.0
                self.planner_ok_s += dt if planner_ok else 0.0
                if planner_status is not None:
                    self.planner_status_s[planner_status] = (
                        self.planner_status_s.get(planner_status, 0.0) + dt)
                    if not planner_ok and behavior is not None:
                        key = f"{behavior}:{planner_status}"
                        self.planner_failure_by_behavior_s[key] = (
                            self.planner_failure_by_behavior_s.get(key, 0.0) + dt)
                acceleration = (speed - last_speed) / dt
                angular_acceleration = (angular_speed - last_angular) / dt
                self.accel_squared_s += acceleration ** 2 * dt
                self.angular_accel_squared_s += angular_acceleration ** 2 * dt
                self.accel_time_s += dt
            elif dt <= 0:
                self.last_sample = None
                return
        if not self.moving and speed >= 0.08:
            self.moving = True
            self.starts += 1
        elif self.moving and speed <= 0.04:
            self.moving = False
        self.last_sample = (now, x, y, speed, angular_speed)

    def contact(self, now, kind=None, position=None):
        """Count a new body impact after separation, with a debounce window."""
        if not self.active:
            return
        self.contacts.append((now, kind, position))
        if kind is None:
            self.contact_active = False
            return
        if not self.contact_active and now - self.last_contact_at >= 0.5:
            self.collisions += 1
            if kind == "robot":
                self.robot_collisions += 1
            else:
                self.wall_collisions += 1
            if position is not None and len(self.collision_points) < 50:
                self.collision_points.append({
                    "at_s": round(now - self.started_at, 2),
                    "x": round(position[0], 3),
                    "y": round(position[1], 3),
                    "kind": kind,
                })
            self.last_contact_at = now
        self.contact_active = True

    def finish_window(self, start, end, wall_duration=None):
        """Replay observations inside the referee interval, clipping boundary steps."""
        if end < start:
            raise ValueError("match end precedes start")
        records = [row for row in self.samples if start <= row[0] <= end]
        for boundary in (start, end):
            if any(row[0] == boundary for row in records):
                continue
            for left, right in zip(self.samples, self.samples[1:]):
                if left[0] < boundary < right[0] and right[0] - left[0] <= 1.0:
                    fraction = (boundary - left[0]) / (right[0] - left[0])
                    values = tuple(left[k] + fraction * (right[k] - left[k])
                                   for k in range(1, 5))
                    records.append((boundary, *values, *left[5:]))
                    break
        rebuilt = RunMetrics(self.max_speed)
        rebuilt.start(start, 0.0 if wall_duration is not None else None)
        for row in sorted(records, key=lambda row: row[0]):
            rebuilt.sample(*row)
        for row in self.contacts:
            if start <= row[0] <= end:
                rebuilt.contact(*row)
        for stamp, name, duration in self.timing_events:
            if stamp is not None and start <= stamp <= end:
                rebuilt.record_timing(name, duration, stamp)
        rebuilt.capture_at = self.capture_at if (
            self.capture_at is not None and start <= self.capture_at <= end) else None
        rebuilt.goal_at = self.goal_at if (
            self.goal_at is not None and start <= self.goal_at <= end) else None
        rebuilt.stop(end, wall_duration)
        self.__dict__.update(rebuilt.__dict__)

    def snapshot(self, now, wall_now=None):
        elapsed = max(0.0, (self.ended_at if self.ended_at is not None else now)
                      - (self.started_at if self.started_at is not None else now))
        wall_end = self.wall_ended_at if self.wall_ended_at is not None else wall_now
        wall_elapsed = (max(0.0, wall_end - self.wall_started_at)
                        if self.wall_started_at is not None and wall_end is not None
                        else None)
        mean_speed = self.distance_m / elapsed if elapsed else 0.0
        return {
            "active": self.active,
            "duration_s": round(elapsed, 2),
            "window_start_sim_s": self.started_at,
            "window_end_sim_s": self.ended_at,
            "sample_coverage_fraction": round(
                self.accel_time_s / elapsed, 4) if elapsed else 0.0,
            "wall_duration_s": round(wall_elapsed, 2) if wall_elapsed is not None else None,
            "real_time_factor": round(elapsed / wall_elapsed, 3) if wall_elapsed else None,
            "timing_ms": {name: timing_summary(values, {
                "decision_compute": 200, "planner_compute": 200, "control_compute": 50,
            }.get(name)) for name, values in sorted(self.timings.items())},
            "distance_m": round(self.distance_m, 3),
            "pose_jumps_ignored": self.pose_jumps,
            "mean_speed_mps": round(mean_speed, 3),
            "moving_speed_mps": round(self.distance_m / self.moving_s, 3)
            if self.moving_s else 0.0,
            "speed_utilization": round(mean_speed / self.max_speed, 3),
            "max_reference_speed_mps": self.max_speed,
            "moving_fraction": round(self.moving_s / elapsed, 3) if elapsed else 0.0,
            "turning_fraction": round(self.turning_s / elapsed, 3) if elapsed else 0.0,
            "active_motion_fraction": round((self.moving_s + self.turning_s) / elapsed, 3)
            if elapsed else 0.0,
            "behavior_fraction": {str(key): round(value / elapsed, 3)
                                  for key, value in sorted(self.behavior_s.items())}
            if elapsed else {},
            "turning_by_behavior_fraction": {
                str(key): round(value / elapsed, 3)
                for key, value in sorted(self.turning_by_behavior_s.items())}
            if elapsed else {},
            "idle_by_behavior_fraction": {
                str(key): round(value / elapsed, 3)
                for key, value in sorted(self.idle_by_behavior_s.items())}
            if elapsed else {},
            "commanded_motion_fraction": round(self.commanded_motion_s / elapsed, 3)
            if elapsed else 0.0,
            "commanded_while_still_fraction": round(
                self.commanded_while_still_s / elapsed, 3) if elapsed else 0.0,
            "planner_status_fraction": {
                key: round(value / elapsed, 3)
                for key, value in sorted(self.planner_status_s.items())}
            if elapsed else {},
            "planner_failure_by_behavior_fraction": {
                key: round(value / elapsed, 3)
                for key, value in sorted(self.planner_failure_by_behavior_s.items())}
            if elapsed else {},
            "linear_accel_rms_mps2": round(sqrt(self.accel_squared_s / self.accel_time_s), 3)
            if self.accel_time_s else 0.0,
            "angular_accel_rms_radps2": round(sqrt(self.angular_accel_squared_s / self.accel_time_s), 3)
            if self.accel_time_s else 0.0,
            "stop_go_events": max(0, self.starts - 1),
            "collisions": self.collisions,
            "wall_collisions": self.wall_collisions,
            "robot_collisions": self.robot_collisions,
            "collision_points": self.collision_points,
            "opponent_visible_fraction": round(self.visible_s / elapsed, 3) if elapsed else 0.0,
            "planner_ok_fraction": round(self.planner_ok_s / elapsed, 3) if elapsed else 0.0,
            "capture_at_s": round(self.capture_at - self.started_at, 2)
            if self.capture_at is not None else None,
            "goal_at_s": round(self.goal_at - self.started_at, 2)
            if self.goal_at is not None else None,
        }
