import sys
import unittest
from math import atan2, cos, hypot, pi, sin
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1] / "src"
for package in ("hsl_decision", "hsl_planning", "hsl_debug_control", "hsl_sim_adapter"):
    sys.path.insert(0, str(ROOT / package))

from hsl_debug_control.core import (follow, path_turning_decision, safe_follow,
                                    safe_mpc_command, select_control_command)
from hsl_decision.core import (CAPTURE, EVADE, GOAL, PURSUE, SEARCH, STOP,
                               WAIT, DecisionPolicy, Observation, Pose2 as DecisionPose,
                               distance_to_polygon, intercept_point)
from hsl_planning.core import (Pose2, VoxelWorld, astar, capture_goal, coverage_target,
                               local_guidance, reachable_target,
                               reachable_intercept, path_heading_error,
                               recovery_step, turn_alignment_is_progress,
                               reusable_route, safe_segment, smooth_control_route,
                               smooth_intercept_target)
from hsl_planning.mppi import (_initial_path_angle_errors, _project_batch,
                               _pruned_route,
                               _reference_prefix, _simulate,
                               mppi_local_guidance)
from hsl_sim_adapter.cloud import transform
from hsl_sim_adapter.patrol import patrol_command
from hsl_sim_adapter.visibility import StaticGrid, detect_opponent
from hsl_sim_adapter.metrics import RunMetrics, capture_possible, duel_outcome, timing_summary


class DecisionTests(unittest.TestCase):
    def setUp(self):
        self.area = [3.5, -2.5, 4.5, -2.5, 4.5, -1.5, 3.5, -1.5]

    def observation(self, *, now=10, allowed=True, opponent=None, scan_stamp=None,
                    own_stamp=None, map_stamp=None):
        return Observation(now, DecisionPose(0, 0), now if own_stamp is None else own_stamp,
                           opponent, now, now if scan_stamp is None else scan_stamp,
                           now if map_stamp is None else map_stamp, allowed)

    def test_freeze_and_stale_input_override_behavior(self):
        policy = DecisionPolicy("explorer", self.area)
        self.assertEqual(policy.step(self.observation(allowed=False)).behavior, WAIT)
        self.assertEqual(policy.step(self.observation(scan_stamp=7)).behavior, STOP)
        self.assertEqual(policy.step(self.observation(own_stamp=7)).behavior, STOP)
        self.assertEqual(policy.step(self.observation()).behavior, GOAL)

    def test_explorer_evades_close_opponent_and_goes_to_goal_otherwise(self):
        policy = DecisionPolicy("explorer", self.area)
        close = self.observation(opponent=DecisionPose(0.3, 0))
        result = policy.step(close)
        self.assertEqual(result.behavior, EVADE)
        self.assertEqual(result.target, policy.goal)
        self.assertEqual(result.opponent_clearance, 1.0)
        self.assertEqual(result.opponent_cost_weight, 8.0)
        self.assertEqual(result.max_speed, 1.0)
        far = self.observation(now=12, opponent=DecisionPose(3, 0))
        goal = policy.step(far)
        self.assertEqual(goal.behavior, GOAL)
        self.assertEqual(goal.max_speed, 1.0)

    def test_explorer_keeps_escaping_until_clear_then_avoids_danger_on_goal_route(self):
        policy = DecisionPolicy("explorer", self.area)
        first = policy.step(self.observation(opponent=DecisionPose(0.8, 0)))
        self.assertEqual(first.behavior, EVADE)
        self.assertEqual(first.target, policy.goal)
        self.assertEqual(policy.step(
            self.observation(now=12, opponent=DecisionPose(1.35, 0))).behavior,
            EVADE)
        safe = policy.step(self.observation(now=14, opponent=DecisionPose(1.8, 0)))
        self.assertEqual(safe.behavior, GOAL)
        self.assertGreaterEqual(safe.opponent_clearance, 0.8)

    def test_guardian_leads_a_moving_target_and_updates_when_velocity_changes(self):
        own, opponent = DecisionPose(0, 0), DecisionPose(2, 0)
        ahead = intercept_point(own, opponent, (0.3, 0))
        behind = intercept_point(own, opponent, (-0.3, 0))
        self.assertGreater(ahead.x, opponent.x)
        self.assertLess(behind.x, opponent.x)
        policy = DecisionPolicy("guardian", self.area)
        obs = self.observation(opponent=opponent)
        moving = Observation(obs.now, obs.own, obs.own_stamp, obs.opponent,
                             obs.opponent_stamp, obs.scan_stamp, obs.map_stamp,
                             obs.allowed, (0.3, 0))
        target = policy.step(moving)
        self.assertEqual(target.behavior, PURSUE)
        self.assertGreater(target.target.x, opponent.x)

    def test_guardian_caps_intercept_lead_time(self):
        target = intercept_point(DecisionPose(0, 0), DecisionPose(2, 0),
                                 (0.35, 0))
        self.assertAlmostEqual(target.x, 2.7)

    def test_guardian_searches_then_pursues_and_captures(self):
        policy = DecisionPolicy("guardian", self.area)
        initial = policy.step(self.observation())
        self.assertEqual(initial.behavior, SEARCH)
        self.assertEqual((initial.target.x, initial.target.y), (4.0, -2.0))
        self.assertEqual(initial.max_speed, 1.0)
        self.assertEqual(policy.step(self.observation(now=12, opponent=DecisionPose(2, 0))).behavior,
                         PURSUE)
        capture = policy.step(self.observation(now=14, opponent=DecisionPose(0.5, 0)))
        self.assertEqual(capture.behavior, CAPTURE)
        self.assertLess(capture.tolerance, 0.03)
        self.assertGreater(abs(capture.target.x - 0.5), 0.4)
        self.assertLess(abs(capture.target.x - 0.5), 0.45)
        self.assertEqual(capture.max_speed, 1.0)

    def test_decision_policy_leaves_motion_speed_to_controller(self):
        for distance in (1.5, 0.8, 0.6, 0.5, 0.42):
            policy = DecisionPolicy("guardian", self.area)
            decision = policy.step(self.observation(
                opponent=DecisionPose(distance, 0)))
            self.assertEqual(decision.max_speed, 1.0)

    def test_guardian_searches_last_seen_position_after_track_expires(self):
        policy = DecisionPolicy("guardian", self.area)
        lost = self.observation(now=12, opponent=DecisionPose(2, 1))
        lost = Observation(lost.now, lost.own, lost.own_stamp, lost.opponent, 10,
                           lost.scan_stamp, lost.map_stamp, lost.allowed)
        result = policy.step(lost)
        self.assertEqual(result.behavior, SEARCH)
        self.assertEqual((result.target.x, result.target.y), (2, 1))

    def test_guardian_search_projects_recently_lost_track_using_measured_velocity(self):
        policy = DecisionPolicy("guardian", self.area)
        lost = Observation(11.6, DecisionPose(0, 0), 11.6,
                           DecisionPose(2, 1), 10.0, 11.6, 11.6, True,
                           (0.5, -0.25))
        result = policy.step(lost)
        self.assertEqual(result.behavior, SEARCH)
        self.assertAlmostEqual(result.target.x, 2.3)
        self.assertAlmostEqual(result.target.y, 0.85)

    def test_guardian_sweeps_and_does_not_assume_capture_through_wall(self):
        policy = DecisionPolicy("guardian", self.area)
        policy.step(self.observation(now=10, opponent=DecisionPose(2, 0)))
        lost = Observation(12, DecisionPose(2, 0), 12,
                           DecisionPose(2, 0), 10, 12, 12, True)
        result = policy.step(lost)
        self.assertEqual(result.behavior, SEARCH)
        self.assertIsNone(result.target)
        moved = Observation(13, DecisionPose(1, 1), 13,
                            DecisionPose(2, 0), 10, 13, 13, True)
        self.assertIsNone(policy.step(moved).target)
        captured = Observation(14, DecisionPose(0, 0, 0), 14,
                               DecisionPose(0.4, 0), 14, 14, 14, True)
        self.assertEqual(policy.step(captured).behavior, CAPTURE)

    def test_hysteresis_prevents_immediate_switch_back(self):
        policy = DecisionPolicy("explorer", self.area, min_dwell=1.0,
                                switch_margin=0.5)
        policy.step(self.observation(opponent=DecisionPose(0.3, 0)))
        result = policy.step(self.observation(now=10.1, opponent=DecisionPose(1.0, 0)))
        self.assertEqual(result.behavior, EVADE)

    def test_reaching_guardian_area_stops_explorer(self):
        policy = DecisionPolicy("explorer", self.area)
        reached = Observation(10, DecisionPose(3.4, -2), 10, None, 0, 10, 10, True)
        self.assertEqual(policy.step(reached).behavior, STOP)
        self.assertAlmostEqual(distance_to_polygon(DecisionPose(4, -2), self.area), 0)


class PlanningTests(unittest.TestCase):
    def test_smoothed_long_mpc_route_checks_every_segment(self):
        world = VoxelWorld(0.1, 0.22)
        world.update([(1.0, 0.0, 0.2)], [], None,
                     map_bounds=(-1.0, -1.0, 3.0, 2.0))
        route = [Pose2(0.0, 0.0), Pose2(0.0, 0.6),
                 Pose2(0.5, 0.6), Pose2(1.0, 0.6),
                 Pose2(1.5, 0.6), Pose2(2.0, 0.6), Pose2(2.0, 0.0)]
        result = smooth_control_route(world, route, safety_margin=0.12)
        self.assertEqual((result[0].x, result[0].y), (0.0, 0.0))
        self.assertEqual((result[-1].x, result[-1].y), (2.0, 0.0))
        self.assertGreater(len(result), len(route))
        self.assertTrue(all(safe_segment(world, a, b, safety_margin=0.12)
                            for a, b in zip(result, result[1:])))

    def test_direct_mppi_commands_respect_stock_mpc_actuator_rates(self):
        world = VoxelWorld(0.1, 0.22)
        world.update([], [], None, map_bounds=(-2, -2, 5, 2))
        route = [Pose2(index * 0.15, 0) for index in range(21)]
        path, controls, diagnostics = mppi_local_guidance(
            world, Pose2(0, 0), route, max_speed=0.5,
            linear_accel=0.5, angular_accel=2.0,
            measured_speed=0.0, measured_omega=0.0,
            rng=np.random.default_rng(12))
        self.assertEqual(diagnostics["result"], "ok")
        self.assertGreater(len(path), 3)
        self.assertIsNotNone(controls)
        previous_v, previous_w = 0.0, 0.0
        for v, w in controls:
            self.assertLessEqual(abs(v - previous_v), 0.5 * 0.15 + 1e-9)
            self.assertLessEqual(abs(w - previous_w), 2.0 * 0.15 + 1e-9)
            self.assertLessEqual(v, 0.5 + 1e-9)
            previous_v, previous_w = v, w

    def test_nav2_mppi_local_path_tracks_straight_route_smoothly(self):
        world = VoxelWorld(0.1, 0.22)
        world.update([], [], None, map_bounds=(-2, -2, 5, 2))
        route = [Pose2(index * 0.15, 0.0) for index in range(21)]

        path, controls, diagnostics = mppi_local_guidance(
            world, Pose2(0.0, 0.0), route,
            rng=np.random.default_rng(12), batch_size=256)

        self.assertEqual(diagnostics["result"], "ok")
        self.assertGreater(len(path), 3)
        self.assertGreater(diagnostics["optimized_path_points"], len(path))
        self.assertIsNotNone(controls)
        self.assertGreater(path[-1].x, 0.8)
        self.assertLessEqual(sum(hypot(b.x - a.x, b.y - a.y)
                                 for a, b in zip(path, path[1:])), 1.21)
        self.assertLess(max(abs(point.y) for point in path), 0.2)
        self.assertLess(path_heading_error(Pose2(0, 0), path), 0.25)
        self.assertTrue(all(safe_segment(world, first, second, safety_margin=0.18)
                            for first, second in zip(path, path[1:])))
        alternate, _, _ = mppi_local_guidance(
            world, Pose2(0, 0), route,
            rng=np.random.default_rng(2), batch_size=256,
            safety_margin=0.18)
        self.assertNotEqual(path, alternate)

    def test_mppi_prunes_route_near_its_start_instead_of_later_loop(self):
        route = [Pose2(0, 0), Pose2(1, 0), Pose2(1, 1),
                 Pose2(0, 1), Pose2(0, 0.1), Pose2(-1, 0.1)]
        pruned = _pruned_route(Pose2(0, 0.08), route)
        self.assertIsNotNone(pruned)
        self.assertGreater(pruned[2][0], 0.0)

    def test_mppi_progress_cannot_jump_to_nearby_later_loop(self):
        route = [Pose2(0, 0), Pose2(1, 0), Pose2(1, 1),
                 Pose2(0, 1), Pose2(0, 0.1), Pose2(-1, 0.1)]
        pruned = _pruned_route(Pose2(0, 0.08), route)
        self.assertIsNotNone(pruned)
        _, unconstrained, _ = _project_batch(
            np.array([0.02]), np.array([0.08]), pruned)
        _, constrained, _ = _project_batch(
            np.array([0.02]), np.array([0.08]), pruned,
            max_progress=np.array([0.35]))
        self.assertGreater(unconstrained[0], 3.0)
        self.assertLessEqual(constrained[0], 0.35)

    def test_mppi_rollout_uses_first_control_after_measured_initial_step(self):
        own = Pose2(0.0, 0.0, 0.0)
        velocities = np.array([[0.4, 0.1, 0.1]])
        omegas = np.array([[0.0, 0.0, 0.0]])
        xs, _, _ = _simulate(own, 0.2, 0.0, velocities, omegas, 1.0)
        self.assertTrue(np.allclose(xs[0], [0.0, 0.2, 0.6, 0.7]))

    def test_mppi_prefix_keeps_forward_driven_u_turn(self):
        points = [Pose2(0.0, 0.0, 0.0), Pose2(0.2, 0.0, 0.4),
                  Pose2(0.3, 0.2, 1.7), Pose2(0.2, 0.4, 2.5),
                  Pose2(0.0, 0.5, 3.0), Pose2(-0.15, 0.5, 3.1)]
        prefix = _reference_prefix(points)
        self.assertEqual(prefix, points)
        self.assertLess(prefix[-1].x, points[0].x)

    def test_mppi_initial_path_angle_critic_prefers_gentle_forward_arc(self):
        own = Pose2(0.0, 0.0, 0.0)
        xs = np.array([[0.0, 0.1, 0.2, 0.3],
                       [0.0, 0.05, 0.08, 0.1],
                       [0.0, 0.0, 0.0, 0.0]])
        ys = np.array([[0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.04, 0.09, 0.15],
                       [0.0, 0.0, 0.0, 0.0]])

        errors = _initial_path_angle_errors(own, xs, ys)

        self.assertAlmostEqual(errors[0], 0.0)
        self.assertGreater(errors[1], 0.9)
        self.assertAlmostEqual(errors[2], 0.0)

    def test_nav2_mppi_local_path_follows_astar_detour_around_new_obstacle(self):
        world = VoxelWorld(0.1, 0.22)
        free = {world.cell(x * 0.1, y * 0.1)
                for x in range(-20, 41) for y in range(-20, 21)}
        world.update([(0.65, 0.0, 0.3)], [], Pose2(0, 0), free,
                     map_bounds=(-2, -2, 4, 2))
        route = astar(world, Pose2(0, 0), Pose2(2, 0))
        self.assertTrue(route)
        self.assertGreater(max(abs(point.y) for point in route), 0.2)

        path, controls, diagnostics = mppi_local_guidance(
            world, Pose2(0, 0), route,
            rng=np.random.default_rng(1), batch_size=256,
            safety_margin=0.18)

        self.assertEqual(diagnostics["result"], "ok")
        self.assertIsNotNone(controls)
        self.assertGreater(path[-1].x, 0.2)
        self.assertGreater(path[-1].y, 0.1)
        self.assertTrue(all(safe_segment(world, first, second, safety_margin=0.18)
                            for first, second in zip(path, path[1:])))

    def test_path_heading_error_reports_alignment_progress(self):
        path = [Pose2(0, 0), Pose2(0.1, 0), Pose2(0.2, 0), Pose2(0.5, 0)]
        self.assertAlmostEqual(path_heading_error(Pose2(0, 0, 1.0), path), 1.0)
        self.assertAlmostEqual(path_heading_error(Pose2(0, 0, 0.7), path), 0.7)
        self.assertIsNone(path_heading_error(Pose2(0, 0), [Pose2(0, 0)]))

    def test_only_guardian_chase_turns_count_as_watchdog_progress(self):
        self.assertTrue(turn_alignment_is_progress("guardian", 6))
        self.assertTrue(turn_alignment_is_progress("guardian", 7))
        self.assertFalse(turn_alignment_is_progress("guardian", 5))
        self.assertFalse(turn_alignment_is_progress("explorer", 6))

    def test_local_guidance_is_straight_on_clear_corridor(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([], [], None, map_bounds=(-1, -1, 4, 1))
        own = Pose2(0, 0, 0)
        route = [Pose2(i * 0.1, 0) for i in range(21)]
        local = local_guidance(world, own, route)
        self.assertGreater(len(local), 10)
        self.assertTrue(all(abs(point.y) < 1e-9 for point in local))
        self.assertTrue(all(later.x > earlier.x for earlier, later in
                            zip(local, local[1:])))

    def test_guardian_does_not_chase_prediction_through_wall(self):
        world = VoxelWorld(0.1, 0.2)
        wall = [(1.0, y * 0.1, 0.3) for y in range(-10, 11)]
        world.update(wall, [], None, map_bounds=(-1, -2, 2, 2))
        own, opponent = Pose2(0, 0), Pose2(0.55, 0)
        blocked_prediction = Pose2(1.4, 0)
        revised = reachable_intercept(world, own, opponent, blocked_prediction)
        self.assertNotEqual(revised, blocked_prediction)
        self.assertLess(revised.x, 1.0)
        self.assertTrue(world.inside_map(revised.x, revised.y, 0.3))
        visible_prediction = Pose2(0.65, 0.1)
        self.assertEqual(reachable_intercept(world, own, opponent,
                                             visible_prediction), visible_prediction)

    def test_small_intercept_updates_are_smoothed_but_large_redirects_are_immediate(self):
        previous = Pose2(1, 1, 0)
        updated = smooth_intercept_target(previous, Pose2(1.4, 1, 0.5))
        self.assertAlmostEqual(updated.x, 1.22)
        self.assertAlmostEqual(updated.y, 1.0)
        self.assertAlmostEqual(updated.yaw, 0.5)
        redirected = Pose2(1, 2, -0.5)
        self.assertEqual(smooth_intercept_target(previous, redirected), redirected)

    def test_local_path_must_increase_clearance_if_already_near_wall(self):
        world = VoxelWorld(0.1, 0.2)
        wall = [(x * 0.1, 0.27, 0.3) for x in range(16)]
        world.update(wall, [], None, map_bounds=(-1, -1, 3, 2))
        own = Pose2(0, 0)
        self.assertFalse(safe_segment(world, own, Pose2(1, 0)))
        self.assertTrue(safe_segment(world, own, Pose2(0.3, -0.3)))

    def test_local_segment_keeps_extra_margin_for_tracking_error(self):
        world = VoxelWorld(0.1, 0.2)
        wall = [(x * 0.05, 0.33, 0.3) for x in range(-10, 31)]
        world.update(wall, [], None, map_bounds=(-1, -1, 2, 1))
        self.assertTrue(safe_segment(world, Pose2(0, 0), Pose2(0.5, 0)))
        self.assertFalse(safe_segment(world, Pose2(0, 0), Pose2(0.5, 0),
                                      safety_margin=0.14))

    def test_local_guidance_does_not_cut_wall_and_recovery_turns_inward(self):
        world = VoxelWorld(0.1, 0.2)
        wall = [(0.6, y * 0.1, 0.3) for y in range(-5, 6)]
        world.update(wall, [], None, map_bounds=(-1, -1, 2, 1))
        self.assertFalse(local_guidance(world, Pose2(0, 0),
                                        [Pose2(1, 0)]))
        near_boundary = Pose2(1.72, 0, 0)
        step = recovery_step(world, near_boundary)
        self.assertIsNotNone(step)
        self.assertLess(cos(step[0]), 0)

    def test_recovery_can_choose_a_short_step_in_a_tight_free_pocket(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([], [], None, map_bounds=(-0.45, -0.45, 0.45, 0.45))
        self.assertIsNotNone(recovery_step(world, Pose2(0, 0)))
        self.assertTrue(local_guidance(world, Pose2(0, 0),
                                       [Pose2(0.09, 0)], min_step=0.04))

    def test_recovery_uses_the_length_it_actually_checked(self):
        world = VoxelWorld(0.1, 0.2)
        walls = [(x, y, 0.3) for x, y in
                 ((0.28, 0), (-0.43, 0), (0, 0.28), (0, -0.28))]
        world.update(walls, [], None, map_bounds=(-1, -1, 1, 1))
        own = Pose2(0, 0, pi)
        step = recovery_step(world, own, safety_margin=0.14)
        self.assertIsNotNone(step)
        heading, length = step
        self.assertLessEqual(length, 0.15)
        target = Pose2(length * cos(heading), length * sin(heading))
        self.assertTrue(local_guidance(world, own, [target], min_step=0.04,
                                       safety_margin=0.14))

    def test_3d_projection_ignores_ground_but_blocks_robot_height(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([(1, 0, 0.0), (2, 0, 0.2)], [], None)
        self.assertFalse(world.blocked(1, 0))
        self.assertTrue(world.blocked(2, 0))

    def test_astar_routes_around_static_wall_and_opponent(self):
        world = VoxelWorld(0.1, 0.18)
        wall = [(1.0, y / 10, 0.3) for y in range(-5, 6)]
        world.update(wall, [], None)
        route = astar(world, Pose2(0, 0), Pose2(2, 0),
                      Pose2(1.6, -0.6), 0.35, 2.0)
        self.assertTrue(route)
        self.assertTrue(all(not world.blocked(p.x, p.y) for p in route))
        self.assertTrue(all((p.x - 1.6) ** 2 + (p.y + 0.6) ** 2 >= 0.35 ** 2
                            for p in route))

    def test_astar_keeps_extra_clearance_beside_a_wall(self):
        world = VoxelWorld(0.1, 0.18)
        world.update([(x * 0.1, 0.25, 0.3) for x in range(1, 20)], [], None)
        route = astar(world, Pose2(0, 0), Pose2(2, 0))
        self.assertTrue(route)
        self.assertTrue(all(point.y <= -0.1 for point in route[2:-2]))

    def test_astar_escapes_opponent_clearance_when_start_is_inside(self):
        world = VoxelWorld(0.1, 0.18)
        world.update([], [], None)
        route = astar(world, Pose2(0, 0), Pose2(-1, 0),
                      Pose2(0.4, 0), clearance=0.65, weight=4.0)
        self.assertTrue(route)
        distances = [hypot(point.x - 0.4, point.y) for point in route]
        self.assertTrue(all(later > earlier for earlier, later in
                            zip(distances, distances[1:]) if earlier < 0.65))

    def test_astar_escapes_threat_then_continues_to_the_goal(self):
        world = VoxelWorld(0.1, 0.2)
        known = {(x, y) for x in range(-20, 31) for y in range(-20, 21)}
        world.update([], [], None, known_free=known,
                     map_bounds=(-2, -2, 3, 2))
        own, opponent, goal = Pose2(0, 0), Pose2(0.5, 0), Pose2(2, 0)
        route = astar(world, own, goal, opponent, clearance=1.0, weight=8.0)
        self.assertTrue(route)
        self.assertAlmostEqual(route[-1].x, goal.x)
        self.assertAlmostEqual(route[-1].y, goal.y)
        distances = [hypot(point.x - opponent.x, point.y - opponent.y)
                     for point in route]
        self.assertGreater(distances[1], distances[0])
        cleared = next(i for i, distance in enumerate(distances)
                       if distance >= 1.0)
        self.assertTrue(all(distance >= 1.0 for distance in distances[cleared:]))

    def test_escape_can_detour_inside_opponent_zone_when_wall_blocks_direct_exit(self):
        world = VoxelWorld(0.1, 0.18)
        world.occupied.update({world.cell(x, y) for x, y in (
            (-0.1, 0), (0, 0.1), (0, -0.1), (-0.1, 0.1),
            (-0.1, -0.1), (0.1, 0.1), (0.1, -0.1))})
        own, opponent = Pose2(0, 0), Pose2(0.6, 0.6)
        route = astar(world, own, Pose2(0.2, -0.6), opponent,
                      clearance=0.85, weight=6.0)
        self.assertTrue(route)
        self.assertGreater(route[1].x, own.x)
        self.assertLess(hypot(route[1].x - opponent.x,
                              route[1].y - opponent.y),
                        hypot(own.x - opponent.x, own.y - opponent.y))

    def test_equal_cost_astar_routes_vary_reproducibly_by_seed(self):
        world = VoxelWorld(0.1, 0.18)
        world.update([(1.0, y * 0.1, 0.3) for y in range(-4, 5)], [], None)
        start, goal = Pose2(0, 0), Pose2(2, 0)
        routes = [astar(world, start, goal, tie_seed=seed) for seed in range(12)]
        self.assertEqual(routes[2], astar(world, start, goal, tie_seed=2))
        self.assertEqual({len(route) for route in routes}, {21})
        self.assertEqual({route[len(route) // 2].y > 0 for route in routes},
                         {True, False})

    def test_reachable_target_avoids_a_frontier_at_own_position(self):
        world = VoxelWorld(0.1, 0.18)
        world.free.update({world.cell(0, 0), world.cell(1, 0)})
        target = reachable_target(world, Pose2(0, 0), Pose2(0.2, 0.2))
        self.assertEqual(target, Pose2(1, 0))

    def test_safe_route_is_advanced_instead_of_replanned(self):
        world = VoxelWorld(0.15, 0.23)
        route = [Pose2(index * 0.15, 0) for index in range(12)]
        target = Pose2(1.65, 0)
        kept = reusable_route(world, Pose2(0.32, 0), route, target, target)
        self.assertEqual(kept[0], route[2])
        self.assertEqual(kept[-1], target)
        self.assertFalse(reusable_route(world, Pose2(0.32, 0), route,
                                        target, Pose2(1.1, 0)))
        world.occupied.add(world.cell(1.2, 0))
        self.assertFalse(reusable_route(world, Pose2(0.32, 0), route,
                                        target, target))

    def test_new_obstacle_on_next_route_cell_forces_replanning(self):
        world = VoxelWorld(0.15, 0.23)
        route = [Pose2(index * 0.15, 0) for index in range(8)]
        target = route[-1]
        world.occupied.add(world.cell(0.3, 0))
        self.assertFalse(reusable_route(world, Pose2(0, 0), route,
                                        target, target))

    def test_frontier_and_capture_line_of_sight(self):
        world = VoxelWorld(0.1, 0.18)
        scan = [(1.0, 0.0, 0.3), (1.0, 1.0, 0.3)]
        world.update([], scan, Pose2(0, 0))
        self.assertIsNotNone(world.frontier(Pose2(0, 0), Pose2(2, 0)))
        capture = capture_goal(world, Pose2(0, 0), Pose2(1.5, 0))
        self.assertIsNotNone(capture)
        self.assertTrue(world.clear_line_3d(capture, Pose2(1.5, 0)))

    def test_unobserved_goal_uses_frontier(self):
        world = VoxelWorld(0.1, 0.18)
        world.update([], [(1.0, 0, 0.3), (1.0, 1.0, 0.3)], Pose2(0, 0))
        target = reachable_target(world, Pose2(0, 0), Pose2(2, 0))
        self.assertIsNotNone(target)
        self.assertIn(world.cell(target.x, target.y), world.free)
        self.assertLess(target.x, 2.0)

    def test_lidar_free_space_cannot_extend_navigation_outside_known_map(self):
        world = VoxelWorld(0.1, 0.2)
        bounds = (-0.5, -0.5, 1.0, 0.5)
        world.update([], [(2.0, 0.0, 0.3)], Pose2(0, 0),
                     map_bounds=bounds)
        self.assertTrue(world.free)
        self.assertTrue(all(world.inside_map(world.point(cell).x,
                                             world.point(cell).y,
                                             world.robot_radius + 0.1)
                            for cell in world.free))
        frontier = world.frontier(Pose2(0, 0), Pose2(2, 0))
        self.assertIsNotNone(frontier)
        self.assertLessEqual(frontier.x, 0.7)
        self.assertFalse(astar(world, Pose2(0, 0), Pose2(2, 0)))

    def test_guardian_coverage_target_is_free_and_reachable(self):
        world = VoxelWorld(0.15, 0.2)
        world.update([], [], Pose2(0, 0),
                     {world.cell(x * 0.15, y * 0.15)
                      for x in range(25) for y in range(15)})
        target = coverage_target(world, Pose2(0, 0), [Pose2(0, 0)])
        self.assertIsNotNone(target)
        self.assertIn(world.cell(target.x, target.y), world.free)
        self.assertTrue(astar(world, Pose2(0, 0), target))

    def test_known_grid_free_cells_do_not_override_inflated_walls(self):
        world = VoxelWorld(0.1, 0.2)
        known = {world.cell(x / 10, 0) for x in range(20)}
        world.update([(1.0, 0, 0.3)], [], None, known)
        self.assertNotIn(world.cell(1.0, 0), world.free)
        self.assertIn(world.cell(0.0, 0), world.free)

    def test_debug_follower_stops_without_path(self):
        self.assertEqual(follow((0, 0, 0), [], 0.5), (0, 0))

    def test_debug_follower_stops_for_frozen_or_stale_data(self):
        own = ((0, 0, 0), 10.0)
        path = ([(0, 0, 0), (1, 0, 0)], 10.0)
        self.assertEqual(safe_follow(10.0, own, path, (WAIT, 0.5, 10.0)), (0, 0))
        self.assertEqual(safe_follow(11.0, own, path, (GOAL, 0.5, 11.0)), (0, 0))
        self.assertGreater(safe_follow(10.0, own, path, (GOAL, 0.5, 10.0))[0], 0)
        self.assertGreater(safe_follow(10.7, own, path, (GOAL, 0.5, 10.0),
                                       pose_timeout=1.2, path_timeout=1.0,
                                       intent_timeout=1.0)[0], 0)

    def test_straight_path_moves_while_aligning_at_moderate_heading_error(self):
        straight = [(0.0, 0.0), (0.1, 0.0), (0.2, 0.0), (0.3, 0.0)]
        rotation, turning, is_curve = path_turning_decision(
            straight, (0.0, 0.0), 0.8, False)
        self.assertIsNone(rotation)
        self.assertFalse(turning)
        self.assertFalse(is_curve)
        rotation, turning, _ = path_turning_decision(
            straight, (0.0, 0.0), 0.8, True)
        self.assertAlmostEqual(rotation, -0.8)
        self.assertTrue(turning)

    def test_straight_path_still_turns_in_place_for_a_large_heading_error(self):
        straight = [(0.0, 0.0), (0.1, 0.0), (0.2, 0.0), (0.3, 0.0)]
        rotation, turning, is_curve = path_turning_decision(
            straight, (0.0, 0.0), 1.2, False)
        self.assertAlmostEqual(rotation, -1.2)
        self.assertTrue(turning)
        self.assertFalse(is_curve)

    def test_short_local_arc_uses_initial_tangent_instead_of_chord(self):
        arc = [(0.0, 0.0), (0.02, 0.0), (0.04, 0.005),
               (0.05, 0.03), (0.05, 0.10)]
        rotation, turning, is_curve = path_turning_decision(
            arc, (0.0, 0.0), 0.0, False)
        self.assertIsNone(rotation)
        self.assertFalse(turning)
        self.assertFalse(is_curve)

    def test_mpc_gate_preserves_controller_angular_range(self):
        for angular in (-1.5, 1.5):
            self.assertEqual(safe_mpc_command(
                10.0, 10.0, 10.0, ([object()], 10.0),
                (GOAL, 1.0, 10.0), (0.5, angular, 10.0)),
                (0.5, angular))

    def test_direct_mppi_command_is_selected_only_for_checked_path(self):
        mpc = (0.2, 0.1, 10.0)
        mppi = (0.3, -0.2, 10.0)
        self.assertEqual(select_control_command("mppi", "OK", mpc, mppi), mppi)
        self.assertEqual(select_control_command("mppi", "RECOVERY_MPPI", mpc, mppi),
                         mppi)
        self.assertEqual(select_control_command("mppi", "RECOVERY_ESCAPE", mpc, mppi), mpc)
        self.assertEqual(select_control_command("mppi", "RECOVERY_FALLBACK", mpc, mppi), mpc)
        self.assertIsNone(select_control_command("mppi", "OK", mpc, None))
        self.assertEqual(select_control_command("mpc", "OK", mpc, mppi), mpc)

    def test_mpc_gate_stops_for_wait_empty_path_or_stale_scan(self):
        path = ([object()], 10.0)
        command = (0.4, 0.2, 10.0)
        self.assertEqual(safe_mpc_command(10.0, 10.0, 10.0, path,
                                          (WAIT, 0.3, 10.0), command), (0, 0))
        self.assertEqual(safe_mpc_command(10.0, 10.0, 10.0, ([], 10.0),
                                          (GOAL, 0.3, 10.0), command), (0, 0))
        self.assertEqual(safe_mpc_command(12.0, 12.0, 10.0, path,
                                          (GOAL, 0.3, 12.0), (0.4, 0.2, 12.0)), (0, 0))
        self.assertEqual(safe_mpc_command(10.0, 10.0, 10.0, path,
                                          (GOAL, 0.3, 10.0), command), (0.3, 0.2))
        self.assertEqual(safe_mpc_command(10.0, 10.0, 10.0, path,
                                          (CAPTURE, 0.3, 10.0), None,
                                          rotation_error=0.5), (0.0, 1.0))

    def test_quaternion_transform(self):
        self.assertEqual(transform((1, 2, 3), (4, 5, 6), (0, 0, 0, 1)),
                         (5, 7, 9))

    def test_opponent_position_is_estimated_from_nonstatic_lidar_cluster(self):
        data = [0] * (40 * 30)
        grid = StaticGrid(0.1, 40, 30, 0, 0, data)
        own = (0.5, 1.0)
        hits = [(1.322, 0.95, 0.3), (1.32, 1.0, 0.3),
                (1.322, 1.05, 0.3)]
        self.assertIsNone(detect_opponent([], grid, own))
        detection = detect_opponent(hits, grid, own)
        self.assertIsNotNone(detection)
        self.assertEqual(detection.hits, 3)
        self.assertAlmostEqual(detection.x, 1.5, delta=0.015)
        self.assertAlmostEqual(detection.y, 1.0, delta=0.02)

    def test_lidar_opponent_cluster_behind_static_wall_is_rejected(self):
        data = [0] * (40 * 30)
        for row in range(30):
            data[row * 40 + 10] = 100
        grid = StaticGrid(0.1, 40, 30, 0, 0, data)
        hits = [(1.322, 0.95, 0.3), (1.32, 1.0, 0.3),
                (1.322, 1.05, 0.3)]
        self.assertIsNone(detect_opponent(hits, grid, (0.5, 1.0)))

    def test_lidar_static_wall_returns_are_not_opponent_detections(self):
        data = [0] * (40 * 30)
        data[10 * 40 + 10] = 100
        grid = StaticGrid(0.1, 40, 30, 0, 0, data)
        wall_hits = [(1.04, 1.04, 0.3)] * 3
        self.assertIsNone(detect_opponent(wall_hits, grid, (0.5, 1.0)))

    def test_scripted_opponent_patrol_turns_then_drives(self):
        linear, angular, reached = patrol_command(2.5, 2.5, 0, 1.0, 2.5)
        self.assertEqual(linear, 0)
        self.assertNotEqual(angular, 0)
        linear, angular, reached = patrol_command(2.5, 2.5, 3.14159, 1.0, 2.5)
        self.assertGreater(linear, 0)
        self.assertFalse(reached)
        self.assertTrue(patrol_command(1.05, 2.5, 3.14159, 1.0, 2.5)[2])

    def test_run_metrics_from_truth_and_separated_contacts(self):
        run = RunMetrics(0.7)
        run.start(10)
        run.sample(10, 0, 0, 0.35, 0)
        run.sample(11, 0.35, 0, 0.35, 0.2, visible=True, planner_ok=True)
        run.contact(11, "wall", (0.35, 0))
        run.contact(11.1, "wall")
        run.contact(11.2, None)
        run.contact(11.7, "robot")
        run.stop(12)
        report = run.snapshot(12)
        self.assertAlmostEqual(report["mean_speed_mps"], 0.175)
        self.assertEqual(report["collisions"], 2)
        self.assertEqual(report["wall_collisions"], 1)
        self.assertEqual(report["robot_collisions"], 1)
        self.assertEqual(report["collision_points"][0]["x"], 0.35)
        self.assertEqual(report["opponent_visible_fraction"], 0.5)
        self.assertGreater(report["angular_accel_rms_radps2"], 0)

    def test_metrics_count_planned_rotation_separately_from_linear_speed(self):
        run = RunMetrics(0.7)
        run.start(10)
        run.sample(10, 0, 0, 0, 0, planner_ok=True)
        run.sample(11, 0, 0, 0, 0.5, planner_ok=True, behavior=2)
        run.sample(12, 0, 0, 0, 0.5, planner_ok=True, behavior=4)
        run.stop(12)
        report = run.snapshot(12)
        self.assertEqual(report["mean_speed_mps"], 0.0)
        self.assertEqual(report["moving_fraction"], 0.0)
        self.assertEqual(report["turning_fraction"], 1.0)
        self.assertEqual(report["active_motion_fraction"], 1.0)
        self.assertEqual(report["turning_by_behavior_fraction"],
                         {"2": 0.5, "4": 0.5})

    def test_metrics_clip_gate_start_and_stop_delay_to_referee_window(self):
        run = RunMetrics(0.7)
        run.start(0, 100)
        for t in range(6):
            run.sample(t, 0.3 * t, 0, 0.3, 0,
                       planner_ok=True, planner_status="OK")
        run.contact(0.5, "wall")
        run.contact(1, None)
        run.contact(2, "robot")
        run.contact(3, None)
        run.contact(4, "wall")
        run.record_timing("planner_compute", 10, 0.5)
        run.record_timing("planner_compute", 20, 2)
        run.record_timing("planner_compute", 30, 4)
        run.finish_window(1.5, 3.5, 4)
        report = run.snapshot(9, 110)
        self.assertEqual(report["duration_s"], 2)
        self.assertEqual(report["distance_m"], 0.6)
        self.assertEqual(report["mean_speed_mps"], 0.3)
        self.assertEqual(report["sample_coverage_fraction"], 1)
        self.assertEqual(report["planner_ok_fraction"], 1)
        self.assertEqual(report["collisions"], 1)
        self.assertEqual(report["robot_collisions"], 1)
        self.assertEqual(report["timing_ms"]["planner_compute"]["count"], 1)
        self.assertEqual(report["real_time_factor"], 0.5)

    def test_metrics_coverage_excludes_gaps_and_recovery_turn_is_active(self):
        run = RunMetrics()
        run.start(0)
        run.sample(0, 0, 0, 0, 0.5, planner_status="RECOVERY_MPPI")
        run.sample(1, 0, 0, 0, 0.5, planner_status="RECOVERY_MPPI")
        run.sample(3, 0, 0, 0, 0.5, planner_status="RECOVERY_MPPI")
        run.stop(3)
        report = run.snapshot(3)
        self.assertEqual(report["sample_coverage_fraction"], 0.3333)
        self.assertEqual(report["active_motion_fraction"], 0.333)
        self.assertEqual(report["mean_speed_mps"], 0)
        self.assertEqual(report["planner_ok_fraction"], 0)

    def test_metrics_distinguish_commanded_motion_from_physical_stall(self):
        run = RunMetrics(0.7)
        run.start(10)
        run.sample(10, 0, 0, 0, 0, planner_ok=True)
        run.sample(11, 0, 0, 0, 0, planner_ok=True,
                   behavior=5, command=(0.3, 0.0))
        run.stop(11)
        report = run.snapshot(11)
        self.assertEqual(report["commanded_motion_fraction"], 1.0)
        self.assertEqual(report["commanded_while_still_fraction"], 1.0)
        self.assertEqual(report["idle_by_behavior_fraction"], {"5": 1.0})

    def test_metrics_attribute_planner_failure_to_behavior(self):
        run = RunMetrics(0.7)
        run.start(10)
        run.sample(10, 0, 0, 0, 0)
        run.sample(11, 0, 0, 0, 0, behavior=4,
                   planner_status="NO_GLOBAL_PATH")
        run.stop(11)
        report = run.snapshot(11)
        self.assertEqual(report["planner_status_fraction"],
                         {"NO_GLOBAL_PATH": 1.0})
        self.assertEqual(report["planner_failure_by_behavior_fraction"],
                         {"4:NO_GLOBAL_PATH": 1.0})

    def test_capture_requires_distance_heading_and_clear_line(self):
        data = [0] * (20 * 20)
        grid = StaticGrid(0.1, 20, 20, 0, 0, data)
        self.assertTrue(capture_possible((0.5, 1.0, 0), (0.9, 1.0, 0), grid))
        self.assertFalse(capture_possible((0.5, 1.0, 3.14), (0.9, 1.0, 0), grid))
        self.assertFalse(capture_possible((0.5, 1.0, 0), (1.0, 1.0, 0), grid))
        data[10 * 20 + 7] = 100
        self.assertFalse(capture_possible((0.5, 1.0, 0), (0.9, 1.0, 0), grid))

    def test_duel_outcome_uses_roles_and_goal_geometry(self):
        grid = StaticGrid(0.1, 60, 30, 0, 0, [0] * (60 * 30))
        first_start = [0.2, 0.7, 0.8, 0.7, 0.8, 1.3, 0.2, 1.3]
        second_start = [3.2, 0.7, 3.8, 0.7, 3.8, 1.3, 3.2, 1.3]
        self.assertEqual(duel_outcome("explorer", (3.05, 1.0, 0),
                                      (4.5, 1.0, 0), first_start,
                                      second_start, grid), "explorer_goal")
        self.assertEqual(duel_outcome("guardian", (4.0, 1.0, 3.14),
                                      (3.6, 1.0, 0), first_start,
                                      second_start, grid), "guardian_capture")
        self.assertIsNone(duel_outcome("explorer", (1.0, 1.0, 0),
                                       (4.5, 1.0, 0), first_start,
                                       second_start, grid))

    def test_teleport_is_not_counted_as_driven_distance(self):
        run = RunMetrics()
        run.start(0)
        run.sample(0, 0, 0, 0, 0)
        run.sample(0.1, 3, 0, 0, 0)
        run.sample(0.2, 3.04, 0, 0.4, 0)
        report = run.snapshot(0.2)
        self.assertEqual(report["pose_jumps_ignored"], 1)
        self.assertAlmostEqual(report["distance_m"], 0.04)

    def test_performance_metrics_catch_long_planner_cycle_and_stalled_stream(self):
        run = RunMetrics()
        run.start(10, wall_now=100)
        for duration in (10, 12, 15, 2000):
            run.record_timing("planner_compute", duration)
        run.observe_stream("scan", 100.0)
        run.observe_stream("command", 100.1)
        run.observe_stream("command", 102.1)
        run.stop(12, wall_now=104)
        report = run.snapshot(12, wall_now=104)
        self.assertEqual(report["real_time_factor"], 0.5)
        self.assertEqual(report["timing_ms"]["planner_compute"]["max"], 2000)
        self.assertEqual(report["timing_ms"]["planner_compute"]["over_budget"], 1)
        self.assertEqual(report["timing_ms"]["planner_compute"]["over_2s"], 1)
        self.assertEqual(report["timing_ms"]["command_period_wall"]["max"], 2000)
        self.assertEqual(report["timing_ms"]["command_period_wall"]["over_2s"], 1)
        self.assertEqual(report["timing_ms"]["scan_to_command_wall"]["max"], 2100)
        self.assertEqual(timing_summary([])["p95"], None)

    def test_performance_metrics_keep_long_stream_outage(self):
        run = RunMetrics()
        run.start(0, wall_now=100)
        run.observe_stream("command", 100)
        run.observe_stream("command", 112)
        report = run.snapshot(1, wall_now=112)
        self.assertEqual(report["timing_ms"]["command_period_wall"]["max"], 12000)
        self.assertEqual(report["timing_ms"]["command_period_wall"]["over_2s"], 1)


if __name__ == "__main__":
    unittest.main()
