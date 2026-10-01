"""Differential-drive local path optimisation adapted from Nav2 MPPI.

Nav2's MPPI controller samples noisy velocity sequences, forward-simulates
their trajectories, scores them with path and obstacle critics, and updates
the nominal sequence with a temperature-weighted average. This module keeps
that local-planning core but returns the selected trajectory as a ``Path``
visualization of the velocity rollout; direct commands drive the motion gate.

Algorithm reference (ROS 2 Humble):
https://github.com/ros-navigation/navigation2/tree/humble/nav2_mppi_controller
"""

from math import hypot, inf, pi, sqrt

import numpy as np

from .core import Pose2, safe_segment


def _pruned_route(own, route):
    """Project onto the closest route segment and keep only its forward tail."""
    if len(route) < 2:
        return None
    best = None
    distance_from_start = 0.0
    for index, (first, second) in enumerate(zip(route, route[1:])):
        # The route is already anchored near the robot by A* or
        # reusable_route(). A later loop can pass closer to the robot while
        # pointing back the other way; projecting onto it would skip the
        # current corridor and make the local path jump backwards.
        if index and distance_from_start >= 1.0:
            break
        dx, dy = second.x - first.x, second.y - first.y
        length2 = dx * dx + dy * dy
        if length2 <= 1e-12:
            continue
        distance_from_start += sqrt(length2)
        fraction = max(0.0, min(1.0,
            ((own.x - first.x) * dx + (own.y - first.y) * dy) / length2))
        x, y = first.x + fraction * dx, first.y + fraction * dy
        distance = hypot(own.x - x, own.y - y)
        if best is None or distance < best[0]:
            best = distance, index, x, y
    if best is None or best[0] > 0.65:
        return None
    _, index, x, y = best
    points = [(x, y)]
    points.extend((point.x, point.y) for point in route[index + 1:])
    # Remove zero-length pieces so projections remain well-conditioned.
    compact = [points[0]]
    for point in points[1:]:
        if hypot(point[0] - compact[-1][0], point[1] - compact[-1][1]) > 1e-5:
            compact.append(point)
    if len(compact) < 2:
        return None
    points = np.asarray(compact, dtype=np.float64)
    delta = points[1:] - points[:-1]
    lengths = np.hypot(delta[:, 0], delta[:, 1])
    starts = np.concatenate(([0.0], np.cumsum(lengths[:-1])))
    return points[:-1, 0], points[:-1, 1], delta[:, 0], delta[:, 1], \
        lengths, starts, float(np.sum(lengths))


def _project_batch(x, y, route, max_progress=None):
    """Project onto reachable route segments, avoiding nearby later loops."""
    ax, ay, dx, dy, lengths, starts, _ = route
    px = x[:, None] - ax[None, :]
    py = y[:, None] - ay[None, :]
    fractions = np.clip((px * dx[None, :] + py * dy[None, :]) /
                        (lengths[None, :] ** 2), 0.0, 1.0)
    if max_progress is not None:
        cap = np.broadcast_to(np.asarray(max_progress, dtype=np.float64), x.shape)
        fractions = np.minimum(
            fractions, np.clip((cap[:, None] - starts[None, :]) /
                               lengths[None, :], 0.0, 1.0))
    nearest_x = ax[None, :] + fractions * dx[None, :]
    nearest_y = ay[None, :] + fractions * dy[None, :]
    squared = (x[:, None] - nearest_x) ** 2 + (y[:, None] - nearest_y) ** 2
    if max_progress is not None:
        squared = np.where(starts[None, :] <= cap[:, None], squared, inf)
    indices = np.argmin(squared, axis=1)
    rows = np.arange(x.shape[0])
    lateral = np.sqrt(squared[rows, indices])
    progress = starts[indices] + fractions[rows, indices] * lengths[indices]
    tangent = np.arctan2(dy[indices], dx[indices])
    return lateral, progress, tangent


def _point_at_progress(route, progress):
    ax, ay, dx, dy, lengths, starts, total = route
    progress = np.clip(progress, 0.0, total)
    indices = np.searchsorted(starts + lengths, progress, side="left")
    indices = np.clip(indices, 0, len(lengths) - 1)
    fraction = np.clip((progress - starts[indices]) / lengths[indices], 0.0, 1.0)
    return (ax[indices] + fraction * dx[indices],
            ay[indices] + fraction * dy[indices])


def _initial_path_angle_errors(own, xs, ys, lookahead_steps=3):
    """Estimate the published Path's initial tangent error relative to the robot heading."""
    lookahead = min(max(1, int(lookahead_steps)), xs.shape[1] - 1)
    dx = xs[:, lookahead] - own.x
    dy = ys[:, lookahead] - own.y
    short = np.hypot(dx, dy) < 0.02
    dx = np.where(short, xs[:, -1] - own.x, dx)
    dy = np.where(short, ys[:, -1] - own.y, dy)
    tangent = np.arctan2(dy, dx)
    return np.abs((tangent - own.yaw + pi) % (2 * pi) - pi)


def _clearance_batch(world, x, y):
    """Conservative obstacle clearance for sampled centres."""
    if world.obstacle_tree is not None:
        distances = world.obstacle_tree.query(
            np.column_stack((x, y)), k=1)[0]
        return np.asarray(distances, dtype=np.float64)
    return np.fromiter((world.obstacle_clearance(float(px), float(py))
                        for px, py in zip(x, y)),
                       dtype=np.float64, count=x.shape[0])


def _correlated_noise(rng, batch, steps, std, correlation=0.72):
    """Use temporally coherent controls so samples include smooth turns."""
    white = rng.normal(0.0, std, (batch, steps))
    noise = np.empty_like(white)
    noise[:, 0] = white[:, 0]
    innovation = sqrt(1.0 - correlation * correlation)
    for step in range(1, steps):
        noise[:, step] = (correlation * noise[:, step - 1] +
                          innovation * white[:, step])
    return noise


def _constrain_control_rates(velocities, omegas, speed, omega, dt,
                             max_speed, max_angular, linear_accel,
                             angular_accel):
    """Bound a command sequence to the measured actuator state and rates."""
    velocities = np.clip(velocities, 0.0, max_speed)
    omegas = np.clip(omegas, -max_angular, max_angular)
    speed_before = np.full(velocities.shape[:-1], speed, dtype=np.float64)
    omega_before = np.full(omegas.shape[:-1], omega, dtype=np.float64)
    for step in range(velocities.shape[-1]):
        velocities[..., step] = np.clip(
            velocities[..., step], speed_before - linear_accel * dt,
            speed_before + linear_accel * dt)
        omegas[..., step] = np.clip(
            omegas[..., step], omega_before - angular_accel * dt,
            omega_before + angular_accel * dt)
        speed_before = velocities[..., step]
        omega_before = omegas[..., step]
    return velocities, omegas


def _simulate(own, initial_speed, initial_omega, velocities, omegas, dt):
    """Forward-simulate a batch using the differential-drive model."""
    batch, steps = velocities.shape
    xs = np.empty((batch, steps + 1), dtype=np.float64)
    ys = np.empty((batch, steps + 1), dtype=np.float64)
    yaws = np.empty((batch, steps + 1), dtype=np.float64)
    xs[:, 0], ys[:, 0], yaws[:, 0] = own.x, own.y, own.yaw
    x = np.full(batch, own.x, dtype=np.float64)
    y = np.full(batch, own.y, dtype=np.float64)
    yaw = np.full(batch, own.yaw, dtype=np.float64)
    for step in range(steps):
        v = (np.full(batch, initial_speed, dtype=np.float64)
             if step == 0 else velocities[:, step - 1])
        w = (np.full(batch, initial_omega, dtype=np.float64)
             if step == 0 else omegas[:, step - 1])
        next_yaw = yaw + w * dt
        moving_turn = np.abs(w) > 1e-6
        dx = v * np.cos(yaw) * dt
        dy = v * np.sin(yaw) * dt
        dx[moving_turn] = (v[moving_turn] / w[moving_turn] *
                           (np.sin(next_yaw[moving_turn]) -
                            np.sin(yaw[moving_turn])))
        dy[moving_turn] = (-v[moving_turn] / w[moving_turn] *
                           (np.cos(next_yaw[moving_turn]) -
                            np.cos(yaw[moving_turn])))
        x += dx
        y += dy
        yaw = next_yaw
        xs[:, step + 1], ys[:, step + 1], yaws[:, step + 1] = x, y, yaw
    return xs, ys, yaws


def _evaluate(world, own, route, velocities, omegas, dt, safety_margin,
              opponent, opponent_velocity, opponent_clearance,
              measured_speed, measured_omega, path_alignment_enabled,
              nominal_v, nominal_w, rng_std_v, rng_std_w, gamma,
              follow_target_progress=None):
    batch, steps = velocities.shape
    xs, ys, yaws = _simulate(own, measured_speed, measured_omega,
                             velocities, omegas, dt)
    valid = np.ones(batch, dtype=bool)
    obstacle_cost = np.zeros(batch, dtype=np.float64)
    unknown_cost = np.zeros(batch, dtype=np.float64)
    path_deviation = np.zeros(batch, dtype=np.float64)
    furthest_progress = np.zeros(batch, dtype=np.float64)
    travelled = np.zeros(batch, dtype=np.float64)
    previous_x = np.full(batch, own.x, dtype=np.float64)
    previous_y = np.full(batch, own.y, dtype=np.float64)
    max_route_progress = route[-1]
    required = world.robot_radius + max(0.0, safety_margin)
    initial_wall = world.obstacle_clearance(own.x, own.y)
    initial_map = world.map_clearance(own.x, own.y)
    initial_opponent = (hypot(own.x - opponent.x, own.y - opponent.y)
                        if opponent is not None else inf)
    source_blocked = world.cell(own.x, own.y) in world.occupied
    time = 0.0

    for step in range(1, steps + 1):
        x, y = xs[:, step], ys[:, step]
        travelled += np.hypot(x - previous_x, y - previous_y)
        previous_x, previous_y = x, y
        time += dt
        wall_clearance = _clearance_batch(world, x, y)
        map_clearance = np.full(batch, inf, dtype=np.float64)
        if world.map_bounds is not None:
            min_x, min_y, max_x, max_y = world.map_bounds
            map_clearance = np.minimum.reduce((x - min_x, y - min_y,
                                               max_x - x, max_y - y))
        travel = np.hypot(x - own.x, y - own.y)
        required_gain = np.minimum(0.04, 0.15 * travel)

        wall_bad = wall_clearance < required
        if initial_wall < required:
            wall_bad &= wall_clearance + 0.005 < initial_wall + required_gain
        map_bad = map_clearance < required
        if initial_map < required:
            map_bad &= map_clearance + 0.005 < initial_map + required_gain

        cells = [world.cell(float(px), float(py)) for px, py in zip(x, y)]
        occupied = np.fromiter((cell in world.occupied for cell in cells),
                               dtype=bool, count=batch)
        if source_blocked:
            occupied &= travel > 0.25
            occupied &= wall_clearance + 0.005 < initial_wall + required_gain
        valid &= ~(wall_bad | map_bad | occupied)

        # Nav2's obstacle critic separates hard collision rejection from a
        # softer preference for extra clearance around the footprint.
        obstacle_cost += np.maximum(0.0, required + 0.35 - wall_clearance)
        if opponent is not None and opponent_clearance > 0:
            ox = opponent.x + opponent_velocity[0] * time
            oy = opponent.y + opponent_velocity[1] * time
            separation = np.hypot(x - ox, y - oy)
            too_close = separation < opponent_clearance
            if initial_opponent < opponent_clearance:
                too_close &= separation < min(0.45, initial_opponent) - 0.01
            valid &= ~too_close
            obstacle_cost += np.maximum(0.0, opponent_clearance + 0.25 - separation)

        # Unknown cells are discouraged while the global route remains the
        # preferred corridor. A newly observed obstacle can still be skirted.
        unknown = np.fromiter((cell not in world.free and cell not in world.occupied
                               for cell in cells), dtype=np.float64, count=batch)
        unknown_cost += unknown

        lateral, progress, _ = _project_batch(
            x, y, route, max_progress=travelled + 0.35)
        path_deviation += lateral
        furthest_progress = np.maximum(furthest_progress, progress)

    path_deviation /= max(1, steps)
    obstacle_cost /= max(1, steps)
    unknown_cost /= max(1, steps)
    batch_furthest = (float(np.max(furthest_progress[valid]))
                      if np.any(valid) else 0.0)
    if follow_target_progress is not None:
        batch_furthest = float(follow_target_progress) - 0.45
    target_x, target_y = _point_at_progress(
        route, min(batch_furthest + 0.45, max_route_progress))
    follow_distance = np.hypot(xs[:, -1] - target_x, ys[:, -1] - target_y)
    _, _, route_yaw = _project_batch(
        xs[:, -1], ys[:, -1], route, max_progress=travelled + 0.35)
    yaw_error = np.abs((yaws[:, -1] - route_yaw + pi) % (2 * pi) - pi)
    initial_path_angle_error = _initial_path_angle_errors(own, xs, ys)
    initial_path_angle_cost = np.square(
        np.maximum(0.0, initial_path_angle_error - 0.65))

    # Match Nav2's PathAlign/PathFollow/PathAngle critics and MPPI's control
    # perturbation term. Add a small sequence smoothness cost for this planner's
    # Path output, which visualizes the same rollout as the direct velocity command.
    align_weight = 6.0 if path_alignment_enabled else 1.5
    follow_weight = 6.0 if path_alignment_enabled else 2.0
    progress_weight = 4.0 if path_alignment_enabled else 8.0
    stall_cost = (5.0 * np.maximum(0.0, 0.25 - furthest_progress)
                  if not path_alignment_enabled else 0.0)
    costs = (align_weight * path_deviation + follow_weight * follow_distance +
             -progress_weight * furthest_progress +
             1.2 * yaw_error + 8.0 * initial_path_angle_cost +
             4.0 * obstacle_cost + 0.5 * unknown_cost +
             stall_cost)
    dv = np.diff(velocities, axis=1)
    dw = np.diff(omegas, axis=1)
    costs += 1.4 * np.sum(dv * dv, axis=1) + 0.18 * np.sum(dw * dw, axis=1)
    costs += gamma / max(rng_std_v * rng_std_v, 1e-6) * np.sum(
        nominal_v[None, :] * (velocities - nominal_v[None, :]), axis=1)
    costs += gamma / max(rng_std_w * rng_std_w, 1e-6) * np.sum(
        nominal_w[None, :] * (omegas - nominal_w[None, :]), axis=1)
    costs[~valid] = 1e6
    details = {
        "valid_samples": int(np.count_nonzero(valid)),
        "batch_size": int(batch),
        "path_deviation": float(np.min(path_deviation[valid])) if np.any(valid) else None,
        "obstacle_cost": float(np.min(obstacle_cost[valid])) if np.any(valid) else None,
        "furthest_progress": float(np.max(furthest_progress[valid])) if np.any(valid) else 0.0,
        "initial_path_angle_error": float(
            np.median(initial_path_angle_error[valid])) if np.any(valid) else None,
        "path_align_enabled": bool(path_alignment_enabled),
    }
    return costs, valid, xs, ys, yaws, details


def _safe_path(world, path, opponent, opponent_velocity, clearance, safety_margin,
               dt):
    if len(path) < 3:
        return False
    initial_opponent = (hypot(path[0].x - opponent.x, path[0].y - opponent.y)
                        if opponent is not None else inf)
    for index, (first, second) in enumerate(zip(path, path[1:])):
        if not safe_segment(world, first, second, opponent, clearance,
                            safety_margin):
            return False
        if opponent is not None and clearance > 0:
            t = (index + 1) * dt
            future_x = opponent.x + opponent_velocity[0] * t
            future_y = opponent.y + opponent_velocity[1] * t
            distance = hypot(second.x - future_x, second.y - future_y)
            if (distance < clearance and
                    (initial_opponent >= clearance or
                     distance < min(0.45, initial_opponent) - 0.01)):
                return False
    return True


def _reference_prefix(path, max_length=1.2):
    """Publish only the nearby MPPI arc while retaining its full warm start."""
    if len(path) < 3:
        return path
    origin = path[0]
    prefix = [origin]
    travelled = 0.0
    for point in path[1:]:
        step = hypot(point.x - prefix[-1].x, point.y - prefix[-1].y)
        if travelled + step > max_length and len(prefix) >= 2:
            break
        prefix.append(point)
        travelled += step
    return prefix


def mppi_local_guidance(world, own, global_path, *, max_speed=1.0,
                        max_angular=1.5, horizon=3.0, dt=0.15,
                        linear_accel=None, angular_accel=None,
                        batch_size=192, iterations=2, temperature=0.3,
                        velocity_std=0.22, angular_std=0.55, gamma=0.015,
                        measured_speed=0.0, measured_omega=0.0,
                        opponent=None, opponent_velocity=(0.0, 0.0),
                        opponent_clearance=0.0, safety_margin=0.18,
                        previous_controls=None, rng=None,
                        _allow_short_retry=True):
    """Optimise a safe local differential-drive trajectory along a global path.

    Returns ``(path, controls, diagnostics)``. ``path`` is the smoothed,
    collision-checked trajectory published for visualization. Empty paths indicate
    that every sampled sequence failed hard safety checks; callers should then
    enter their bounded recovery behavior.
    """
    route = _pruned_route(own, global_path)
    if route is None:
        return [], None, {"result": "route_miss", "valid_samples": 0}
    if max_speed <= 0 or max_angular <= 0 or horizon <= 0 or dt <= 0:
        return [], None, {"result": "invalid_configuration", "valid_samples": 0}
    if ((linear_accel is None) != (angular_accel is None) or
            (linear_accel is not None and
             (linear_accel <= 0 or angular_accel <= 0))):
        return [], None, {"result": "invalid_configuration", "valid_samples": 0}

    steps = max(4, int(round(horizon / dt)))
    dt = horizon / steps
    batch_size = max(32, int(batch_size))
    iterations = max(1, int(iterations))
    rng = rng or np.random.default_rng()
    max_speed = max(0.0, float(max_speed))
    max_angular = abs(float(max_angular))
    measured_speed = float(np.clip(measured_speed, 0.0, max_speed))
    measured_omega = float(np.clip(measured_omega, -max_angular, max_angular))

    if previous_controls is not None and len(previous_controls) >= steps:
        previous = np.asarray(previous_controls, dtype=np.float64)
        shift = max(1, int(round(0.2 / dt)))
        velocities = np.concatenate((previous[shift:, 0],
                                     np.repeat(previous[-1, 0], shift)))[:steps]
        omegas = np.concatenate((previous[shift:, 1],
                                 np.repeat(previous[-1, 1], shift)))[:steps]
        velocities[0] = measured_speed
        omegas[0] = measured_omega
    else:
        # Seed the first batch around the motion model's nominal
        # speed. This is a warm-start only; samples remain bounded by the
        # behavior request and the configured motion model owns speed limits.
        velocities = np.full(steps, min(0.3, max_speed), dtype=np.float64)
        omegas = np.zeros(steps, dtype=np.float64)

    # The Nav2 alignment critic is disabled when most of the local path is
    # blocked, allowing obstacle/path-follow critics to route around new data.
    route_length = route[-1]
    route_samples = max(2, int(min(route_length, horizon * max_speed) / 0.15))
    sample_progress = np.linspace(0.0, min(route_length, horizon * max_speed),
                                   route_samples)
    path_x, path_y = _point_at_progress(route, sample_progress)
    blocked_ratio = (sum(world.blocked(float(x), float(y))
                         for x, y in zip(path_x, path_y)) / route_samples)
    alignment_enabled = blocked_ratio <= 0.07

    best_mean = None
    best_sample = None
    best_details = {}
    for _ in range(iterations):
        nominal_v = velocities.copy()
        nominal_w = omegas.copy()
        sampled_v = np.clip(
            velocities[None, :] + _correlated_noise(
                rng, batch_size, steps, velocity_std),
            0.0, max_speed)
        sampled_w = np.clip(
            omegas[None, :] + _correlated_noise(
                rng, batch_size, steps, angular_std),
            -max_angular, max_angular)
        # Small deterministic seeds make broad left/right arcs available even
        # in modest batches. They are scored by the same Nav2-style critics as
        # all stochastic samples and never bypass collision validation.
        turn_templates = (-1.0, -0.65, -0.35, 0.35, 0.65, 1.0)
        for index, fraction in enumerate(turn_templates):
            if index < batch_size:
                sampled_w[index, :] = np.clip(
                    omegas + fraction * max_angular,
                    -max_angular, max_angular)
                sampled_v[index, :] = np.maximum(
                    sampled_v[index, :], min(0.3, max_speed))
        template_index = len(turn_templates)
        for turn_fraction in (0.25, 0.38, 0.50, 0.62):
            turn_steps = max(2, int(round(steps * turn_fraction)))
            for sign in (-1.0, 1.0):
                if template_index >= batch_size:
                    break
                sampled_v[template_index, :] = min(0.3, max_speed)
                sampled_w[template_index, :] = 0.0
                sampled_w[template_index, :turn_steps] = sign * max_angular
                template_index += 1
        # A tight corner sometimes requires turning before moving. These
        # candidates are still scored against route progress, so they only
        # win when a moving arc has no safe swept footprint.
        for turn_fraction in (0.25, 0.40):
            turn_steps = max(2, int(round(steps * turn_fraction)))
            for sign in (-1.0, 1.0):
                if template_index >= batch_size:
                    break
                sampled_v[template_index, :] = min(0.3, max_speed)
                sampled_v[template_index, :turn_steps] = 0.0
                sampled_w[template_index, :] = 0.0
                sampled_w[template_index, :turn_steps] = sign * max_angular
                template_index += 1
        # Keep a full-acceleration choice in the batch. Around a slow warm
        # start, noise alone seldom reaches the actuator's upper range; these
        # sequences still pass the same rate and swept-collision checks.
        for steering in (omegas, np.zeros(steps, dtype=np.float64)):
            if template_index >= batch_size:
                break
            sampled_v[template_index, :] = max_speed
            sampled_w[template_index, :] = steering
            template_index += 1
        if linear_accel is not None:
            sampled_v, sampled_w = _constrain_control_rates(
                sampled_v, sampled_w, measured_speed, measured_omega, dt,
                max_speed, max_angular, linear_accel, angular_accel)
        costs, valid, xs, ys, yaws, details = _evaluate(
            world, own, route, sampled_v, sampled_w, dt, safety_margin,
            opponent, opponent_velocity, opponent_clearance,
            measured_speed, measured_omega, alignment_enabled,
            nominal_v, nominal_w, velocity_std, angular_std, gamma)
        if not np.any(valid):
            best_details = details
            continue

        valid_costs = costs[valid]
        minimum = float(np.min(valid_costs))
        weights = np.zeros(batch_size, dtype=np.float64)
        weights[valid] = np.exp(-np.minimum((valid_costs - minimum) /
                                             max(temperature, 1e-4), 700.0))
        weights /= max(float(np.sum(weights)), 1e-12)
        velocities = np.sum(sampled_v * weights[:, None], axis=0)
        omegas = np.sum(sampled_w * weights[:, None], axis=0)

        # Keep the optimizer's control history smooth before constructing its
        # path, then recheck the resulting swept footprint below.
        if steps >= 3:
            velocities[1:-1] = (0.25 * velocities[:-2] +
                                0.50 * velocities[1:-1] +
                                0.25 * velocities[2:])
            omegas[1:-1] = (0.25 * omegas[:-2] +
                            0.50 * omegas[1:-1] +
                            0.25 * omegas[2:])
        velocities = np.clip(velocities, 0.0, max_speed)
        omegas = np.clip(omegas, -max_angular, max_angular)
        if linear_accel is not None:
            velocities, omegas = _constrain_control_rates(
                velocities, omegas, measured_speed, measured_omega, dt,
                max_speed, max_angular, linear_accel, angular_accel)

        # The weighted mean is the next optimizer seed. For the published Path,
        # compare it with the best sampled controls: averaging equally good
        # left/right obstacle-avoidance modes can cancel into a stationary path.
        best_details = details
        mean_costs, mean_valid, _, _, _, mean_details = _evaluate(
            world, own, route, velocities[None, :], omegas[None, :], dt,
            safety_margin, opponent, opponent_velocity, opponent_clearance,
            measured_speed, measured_omega, alignment_enabled,
            nominal_v, nominal_w, velocity_std, angular_std, gamma,
            follow_target_progress=min(
                details["furthest_progress"] + 0.45, route_length))
        if mean_valid[0]:
            tx, ty, tyaw = _simulate(
                own, measured_speed, measured_omega,
                velocities[None, :], omegas[None, :], dt)
            mean_path = [own] + [Pose2(float(tx[0, i]), float(ty[0, i]),
                                       float(tyaw[0, i]))
                                 for i in range(1, steps + 1)]
            if _safe_path(world, mean_path, opponent, opponent_velocity,
                          opponent_clearance, safety_margin, dt):
                mean_distance = sum(hypot(b.x - a.x, b.y - a.y)
                                    for a, b in zip(mean_path, mean_path[1:]))
                _, mean_progress, _ = _project_batch(
                    np.array([mean_path[-1].x]),
                    np.array([mean_path[-1].y]), route,
                    max_progress=mean_distance + 0.35)
                candidate = (float(mean_costs[0]), float(mean_progress[0]),
                             mean_path, np.column_stack((velocities, omegas)).tolist())
                if best_mean is None or candidate[0] < best_mean[0]:
                    best_mean = candidate
        order = np.argsort(costs)
        for i in order[:min(32, batch_size)]:
            if not valid[i]:
                continue
            candidate_v, candidate_w = sampled_v[i], sampled_w[i]
            one_v = np.asarray(candidate_v, dtype=np.float64)[None, :]
            one_w = np.asarray(candidate_w, dtype=np.float64)[None, :]
            tx, ty, tyaw = _simulate(own, measured_speed, measured_omega,
                                     one_v, one_w, dt)
            path = [own] + [Pose2(float(tx[0, i]), float(ty[0, i]),
                                  float(tyaw[0, i]))
                            for i in range(1, steps + 1)]
            if _safe_path(world, path, opponent, opponent_velocity,
                          opponent_clearance, safety_margin, dt):
                distance = sum(hypot(b.x - a.x, b.y - a.y)
                               for a, b in zip(path, path[1:]))
                _, progress, _ = _project_batch(
                    np.array([path[-1].x]), np.array([path[-1].y]), route,
                    max_progress=distance + 0.35)
                candidate = (float(costs[i]), float(progress[0]), path,
                             np.column_stack((candidate_v, candidate_w)).tolist())
                if best_sample is None or candidate[0] < best_sample[0]:
                    best_sample = candidate
                break

    # Nav2 publishes the weighted MPPI control sequence. Here its simulated
    # trajectory becomes the reference Path. Fall back to one sampled mode
    # when averaging left/right obstacle avoidance would erase progress.
    selected = best_mean
    selected_type = "weighted_mean"
    if best_sample is not None and (selected is None or
            selected[0] > best_sample[0] + 0.75 or
            selected[1] < best_sample[1] - 0.15):
        selected = best_sample
        selected_type = "safe_sample"
    if selected is None:
        if _allow_short_retry and horizon > 2.0:
            short_horizon = min(1.65, 0.55 * horizon)
            short_path, short_controls, short_details = mppi_local_guidance(
                world, own, global_path, max_speed=max_speed,
                max_angular=max_angular, horizon=short_horizon, dt=dt,
                linear_accel=linear_accel, angular_accel=angular_accel,
                batch_size=batch_size, iterations=iterations,
                temperature=temperature, velocity_std=velocity_std,
                angular_std=angular_std, gamma=gamma,
                measured_speed=measured_speed, measured_omega=measured_omega,
                opponent=opponent, opponent_velocity=opponent_velocity,
                opponent_clearance=opponent_clearance,
                safety_margin=safety_margin, previous_controls=None, rng=rng,
                _allow_short_retry=False)
            if short_path:
                short_details.update({"result": "ok_short_horizon",
                                      "original_horizon": float(horizon),
                                      "short_horizon": float(short_horizon)})
                return short_path, short_controls, short_details
            best_details["short_retry_result"] = short_details.get("result")
        best_details.update({"result": "no_safe_sample", "valid_samples": 0})
        return [], None, best_details
    selected_cost, selected_progress, best_trajectory, best_controls = selected
    optimized_points = len(best_trajectory)
    best_trajectory = _reference_prefix(best_trajectory)
    best_details.update({"result": "ok", "selected_cost": selected_cost,
                         "selected_progress": selected_progress,
                         "selected_type": selected_type,
                         "path_points": len(best_trajectory),
                         "optimized_path_points": optimized_points,
                         "path_alignment_blocked_ratio": float(blocked_ratio)})
    return best_trajectory, best_controls, best_details
