import sys
import unittest
from math import atan2, cos, hypot, pi, sin
from pathlib import Path



ROOT = Path(__file__).resolve().parents[1] / "src"
for package in ("hsl_decision", "hsl_planning", "hsl_debug_control", "hsl_sim_adapter"):
    sys.path.insert(0, str(ROOT / package))

from hsl_debug_control.core import (
                                    safe_motion_command, select_control_command, match_is_active)
from hsl_decision.core import (CAPTURE, EVADE, GOAL, PURSUE, SEARCH, STOP,
                               WAIT, DecisionPolicy, Observation, Pose2 as DecisionPose,
                               distance_to_polygon, intercept_point)
from hsl_planning.core import (Pose2, VoxelWorld, astar, capture_goal, moving_capture_goal, coverage_target, reachable_frontier_route,
                               reachable_target, navigation_obstacles, evade_target, evade_objective_route,
                               reachable_intercept, path_heading_error,
                               recovery_step, checked_recovery_target, turn_alignment_is_progress,
                               reusable_route, safe_segment, continuous_short_goal_route,
                               smooth_intercept_target)
from hsl_sim_adapter.cloud import transform
from hsl_sim_adapter.visibility import StaticGrid
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

    def test_guardian_interception_uses_configured_own_speed(self):
        obs = self.observation(opponent=DecisionPose(1, 0))
        obs = Observation(obs.now, obs.own, obs.own_stamp, obs.opponent,
                          obs.opponent_stamp, obs.scan_stamp, obs.map_stamp,
                          obs.allowed, (-0.25, 0))
        slow = DecisionPolicy("guardian", self.area, own_max_speed=0.3).step(obs)
        fast = DecisionPolicy("guardian", self.area, own_max_speed=0.5).step(obs)
        self.assertEqual(slow.behavior, PURSUE)
        self.assertEqual(fast.behavior, PURSUE)
        # Analytic head-on meeting: t = distance / (own speed + approach speed).
        self.assertAlmostEqual(slow.target.x, 1 - 0.25 / 0.55 - 0.42)
        self.assertAlmostEqual(fast.target.x, 1 - 0.25 / 0.75 - 0.42)
        for invalid in (0, -0.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                DecisionPolicy("guardian", self.area, own_max_speed=invalid)

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








    def test_path_heading_error_reports_alignment_progress(self):
        path = [Pose2(0, 0), Pose2(0.1, 0), Pose2(0.2, 0), Pose2(0.5, 0)]
        self.assertAlmostEqual(path_heading_error(Pose2(0, 0, 1.0), path), 1.0)
        self.assertAlmostEqual(path_heading_error(Pose2(0, 0, 0.7), path), 0.7)
        self.assertIsNone(path_heading_error(Pose2(0, 0), [Pose2(0, 0)]))

    def test_navigation_and_evasion_turns_can_be_bounded_watchdog_progress(self):
        self.assertTrue(turn_alignment_is_progress("guardian", 6))
        self.assertTrue(turn_alignment_is_progress("guardian", 7))
        self.assertFalse(turn_alignment_is_progress("guardian", 5))
        self.assertFalse(turn_alignment_is_progress("explorer", 6))
        self.assertTrue(turn_alignment_is_progress("explorer", 2))
        self.assertTrue(turn_alignment_is_progress("explorer", 4))


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

    def test_interception_forecast_does_not_require_prey_to_leave_wall(self):
        world = VoxelWorld(0.1, 0.23)
        wall = [(index * 0.1, 0, 0.3) for index in range(-10, 21)]
        world.update(wall, [], None, map_bounds=(-2, -1, 3, 2))
        own, opponent, predicted = Pose2(-0.5, 1), Pose2(0, 0.3), Pose2(1, 0.3)
        # The unchanged recovery policy demands departure from this wall,
        # but it is a valid constant-velocity forecast along the corridor.
        self.assertFalse(safe_segment(world, opponent, predicted))
        self.assertEqual(reachable_intercept(world, own, opponent, predicted), predicted)

    def test_close_capture_leads_receding_and_crossing_prey_but_directly_approaches_incoming_prey(self):
        world = VoxelWorld(0.1, 0.23)
        world.update([], [], None, map_bounds=(-2, -2, 3, 2))
        own, enemy = Pose2(0, 0), Pose2(0.6, 0)
        direct = capture_goal(world, own, enemy)
        target, predicted = moving_capture_goal(world, own, enemy, (0.5, 0), 0.5)
        self.assertGreater(predicted.x, enemy.x)
        self.assertGreater(target.x, direct.x)
        _, crossing = moving_capture_goal(world, own, enemy, (0, 0.5), 0.5)
        self.assertGreater(crossing.y, enemy.y)
        for velocity in ((0, 0), (-0.5, 0)):
            target, predicted = moving_capture_goal(world, own, enemy, velocity, 0.5)
            self.assertEqual(predicted, enemy)
            self.assertEqual(target, direct)
        _, fast = moving_capture_goal(world, own, enemy, (1.0, 0), 0.5)
        self.assertAlmostEqual(fast.x - enemy.x, 1.0)

    def test_close_capture_forecast_respects_wall_and_existing_capture_orientation(self):
        world = VoxelWorld(0.1, 0.23)
        wall = [(1.0, index * 0.1, 0.3) for index in range(-10, 11)]
        world.update(wall, [], None, map_bounds=(-2, -2, 3, 2))
        own, enemy = Pose2(0, 0), Pose2(0.6, 0)
        target, predicted = moving_capture_goal(world, own, enemy, (0.8, 0), 0.5)
        self.assertEqual(predicted, enemy)
        self.assertEqual(target, capture_goal(world, own, enemy))
        own = Pose2(0.2, 0, pi)
        target, predicted = moving_capture_goal(world, own, enemy, (0.3, 0), 0.5)
        self.assertEqual(predicted, enemy)
        self.assertEqual((target.x, target.y), (own.x, own.y))
        self.assertAlmostEqual(target.yaw, 0.0)

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


    def test_native_escape_rechecks_new_obstacle_and_faces_travel(self):
        world = VoxelWorld(0.1, 0.2)
        own = Pose2(0, 0)
        world.update([], [], None, map_bounds=(-2, -2, 2, 2))
        previous = Pose2(0, 0.55)
        retained = checked_recovery_target(world, own, previous)
        self.assertAlmostEqual(retained.yaw, pi / 2)
        world.update([(0, 0.55, 0.3)], [], None, map_bounds=(-2, -2, 2, 2))
        alternate = checked_recovery_target(world, own, previous)
        self.assertIsNotNone(alternate)
        self.assertGreater(hypot(alternate.x - previous.x, alternate.y - previous.y), 0.2)
        self.assertTrue(safe_segment(world, own, alternate))

    def test_native_escape_rechecks_observed_opponent(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([], [], None, map_bounds=(-2, -2, 2, 2))
        own, enemy = Pose2(0, 0), Pose2(0.6, 0)
        alternate = checked_recovery_target(world, own, Pose2(0.55, 0), enemy, 0.6)
        self.assertIsNotNone(alternate)
        self.assertTrue(safe_segment(world, own, alternate, enemy, 0.6))
        self.assertLess(alternate.x, 0.2)

    def test_recovery_can_choose_a_short_step_in_a_tight_free_pocket(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([], [], None, map_bounds=(-0.45, -0.45, 0.45, 0.45))
        self.assertIsNotNone(recovery_step(world, Pose2(0, 0)))
        self.assertTrue(safe_segment(world, Pose2(0, 0), Pose2(0.09, 0)))

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
        self.assertTrue(safe_segment(world, own, target, safety_margin=0.14))

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

    def test_blocked_objective_selects_a_frontier_outside_the_threat(self):
        world = VoxelWorld(0.1, 0.18)
        world.update([], [], Pose2(0, 0),
                     {(x, y) for x in range(-5, 25) for y in range(-5, 6)})
        own, objective, enemy = Pose2(0, 0), Pose2(2, 0), Pose2(2, 0)
        self.assertFalse(astar(world, own, objective, enemy, 0.7, 6))
        route = reachable_frontier_route(world, own, objective, enemy, 0.7, 6)
        self.assertTrue(route)
        self.assertGreaterEqual(hypot(route[-1].x - enemy.x, route[-1].y - enemy.y), 0.7)
        self.assertGreaterEqual(hypot(route[-1].x - own.x, route[-1].y - own.y), 0.6)
        self.assertTrue(all(hypot(p.x - enemy.x, p.y - enemy.y) >= 0.7 for p in route))

    def test_frontier_search_tries_another_component_when_first_is_unreachable(self):
        world = VoxelWorld(0.1, 0.18)
        world.map_bounds = (-1.0, -1.0, 1.8, 1.0)
        world.free.update({(0, 0), (-5, 0), (13, 0)})
        world.occupied.update({(6, y) for y in range(-11, 12)})
        own, goal = Pose2(0, 0), Pose2(1.3, 0)
        self.assertEqual(world.frontier(own, goal, min_travel=0.6), goal)
        # Use a valid candidate farther than the minimum travel threshold.
        world.free.add((-6, 0))
        route = reachable_frontier_route(world, own, goal)
        self.assertTrue(route)
        self.assertLess(route[-1].x, 0.6)
        self.assertTrue(all(p.x < 0.6 for p in route))

    def test_reachable_target_avoids_a_frontier_at_own_position(self):
        world = VoxelWorld(0.1, 0.18)
        world.free.update({world.cell(0, 0), world.cell(1, 0)})
        target = reachable_target(world, Pose2(0, 0), Pose2(0.2, 0.2))
        self.assertEqual(target, Pose2(1, 0))

    def test_completed_partial_route_retries_the_real_objective(self):
        world = VoxelWorld(0.1, 0.18)
        route = [Pose2(0, 0), Pose2(0.5, 0), Pose2(1, 0)]
        goal = Pose2(2, 0)
        self.assertTrue(reusable_route(world, Pose2(0.1, 0), route, goal, goal))
        self.assertFalse(reusable_route(world, Pose2(0.9, 0), route, goal, goal))

    def test_single_cell_pursuit_keeps_continuous_target_and_heading(self):
        world = VoxelWorld(0.15, 0.23)
        own, target = Pose2(0.01, 0.01), Pose2(0.04, 0.03, 1.2)
        route = [Pose2(0, 0)]
        self.assertEqual(continuous_short_goal_route(world, own, route, target),
                         [own, target])

    def test_reused_single_cell_tracks_moved_goal(self):
        world = VoxelWorld(0.15, 0.23)
        own, previous = Pose2(0.02, 0), Pose2(0.04, 0)
        target, route = Pose2(0.24, 0.02), [Pose2(0, 0)]
        reused = reusable_route(world, own, route, previous, target)
        self.assertEqual(reused, route)
        self.assertEqual(continuous_short_goal_route(world, own, reused, target),
                         [own, target])

    def test_single_cell_goal_does_not_bypass_obstacles_or_map_boundary(self):
        for points, bounds, target in (
                ([(0.3, 0, 0.3)], (-2, -2, 2, 2), Pose2(0.6, 0)),
                ([], (-1, -1, 0.4, 1), Pose2(0.5, 0))):
            for use_scan in (False, True):
                world = VoxelWorld(0.1, 0.18)
                world.update([] if use_scan else points,
                             points if use_scan else [], Pose2(0, 0),
                             map_bounds=bounds)
                route = [Pose2(0, 0)]
                self.assertEqual(continuous_short_goal_route(
                    world, Pose2(0, 0), route, target), route)

    def test_continuous_goal_leaves_long_and_empty_routes_unchanged(self):
        world = VoxelWorld(0.15, 0.23)
        for route in ([], [Pose2(0, 0), Pose2(0, 1), Pose2(1, 1)]):
            self.assertEqual(continuous_short_goal_route(
                world, Pose2(0, 0), route, Pose2(1, 0)), route)

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



    def test_common_match_permission_is_required_and_expires(self):
        self.assertFalse(match_is_active(10.0, None))
        self.assertFalse(match_is_active(10.0, (False, 10.0)))
        self.assertTrue(match_is_active(10.2, (True, 10.0)))
        self.assertFalse(match_is_active(10.6, (True, 10.0)))
        self.assertFalse(match_is_active(9.0, (True, 10.0)))
        self.assertFalse(match_is_active(10.2, (False, 10.1)))

    def test_motion_gate_preserves_controller_angular_range(self):
        for angular in (-1.5, 1.5):
            self.assertEqual(safe_motion_command(
                10.0, 10.0, 10.0, ([object()], 10.0),
                (GOAL, 1.0, 10.0), (0.5, angular, 10.0)),
                (0.5, angular))

    def test_direct_mppi_command_is_selected_only_for_checked_path(self):
        mppi = (0.3, -0.2, 10.0)
        self.assertEqual(select_control_command("OK", mppi), mppi)
        self.assertEqual(select_control_command("RECOVERY_MPPI", mppi), mppi)
        for status in ("RECOVERY_ROUTE", "UNKNOWN_STATUS", "NO_LOCAL_PATH"):
            self.assertIsNone(select_control_command(status, mppi))
        self.assertIsNone(select_control_command("OK", None))

    def test_motion_gate_stops_for_wait_empty_path_or_stale_scan(self):
        path = ([object()], 10.0)
        command = (0.4, 0.2, 10.0)
        self.assertEqual(safe_motion_command(10.0, 10.0, 10.0, path,
                                          (WAIT, 0.3, 10.0), command), (0, 0))
        self.assertEqual(safe_motion_command(10.0, 10.0, 10.0, ([], 10.0),
                                          (GOAL, 0.3, 10.0), command), (0, 0))
        self.assertEqual(safe_motion_command(12.0, 12.0, 10.0, path,
                                          (GOAL, 0.3, 12.0), (0.4, 0.2, 12.0)), (0, 0))
        self.assertEqual(safe_motion_command(10.0, 10.0, 10.0, path,
                                          (GOAL, 0.3, 10.0), command), (0.3, 0.2))
        self.assertEqual(safe_motion_command(10.0, 10.0, 10.0, path,
                                          (CAPTURE, 0.3, 10.0), None), (0.0, 0.0))

    def test_quaternion_transform(self):
        self.assertEqual(transform((1, 2, 3), (4, 5, 6), (0, 0, 0, 1)),
                         (5, 7, 9))

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


class KnownWallPreservationTests(unittest.TestCase):
    def test_wall_near_tracked_robot_remains_blocked_for_interception(self):
        wall = (1.0, 0.0, 0.3)
        robot_return = (0.8, 0.0, 0.3)
        far_obstacle = (2.0, 0.0, 0.3)
        static, scan = navigation_obstacles(
            [wall], [robot_return, far_obstacle],
            [robot_return, far_obstacle], Pose2(0.8, 0))
        self.assertEqual(static, [wall, far_obstacle])
        self.assertEqual(scan, [far_obstacle])
        world = VoxelWorld(0.1, 0.23)
        world.update(static, scan, Pose2(0, 0))
        self.assertTrue(world.blocked(1.0, 0))
        self.assertFalse(safe_segment(world, Pose2(0.5, 0), Pose2(1.5, 0)))

    def test_lost_track_keeps_all_observed_obstacles(self):
        wall, obstacle = (1.0, 0.0, 0.3), (0.8, 0.0, 0.3)
        self.assertEqual(navigation_obstacles([wall], [obstacle], [obstacle]),
                         ([wall, obstacle], [obstacle]))





class EncounterRegressionTests(unittest.TestCase):
    def test_evasion_in_open_space_departs_away_from_guardian(self):
        world = VoxelWorld(0.1, 0.23)
        world.update([], [], None, map_bounds=(-3, -3, 3, 3))
        target = evade_target(world, Pose2(0, 0), Pose2(0.7, 0), Pose2(2, 0))
        self.assertIsNotNone(target)
        self.assertLess(target.x, 0)
        self.assertGreater(hypot(target.x - 0.7, target.y), 0.7)
        self.assertTrue(safe_segment(world, Pose2(0, 0), target))

    def test_evasion_with_wall_chooses_safe_side_departure(self):
        world = VoxelWorld(0.1, 0.23)
        world.update([(-0.45, y / 10, 0.3) for y in range(-20, 21)], [], None,
                     map_bounds=(-3, -3, 3, 3))
        target = evade_target(world, Pose2(0, 0), Pose2(0.7, 0), Pose2(2, 0), safety_margin=0.12)
        self.assertIsNotNone(target)
        self.assertLessEqual(target.x, 1e-6)
        self.assertTrue(safe_segment(world, Pose2(0, 0), target))

    def test_evasion_keeps_objective_when_guardian_is_behind(self):
        world = VoxelWorld(0.1, 0.23)
        world.update([], [], None, map_bounds=(-3, -3, 3, 3))
        route = evade_objective_route(world, Pose2(0, 0), Pose2(-0.7, 0),
                                     Pose2(2, 0), Pose2(-0.4, 0))
        self.assertTrue(route)
        self.assertAlmostEqual(route[-1].x, 2)
        self.assertGreater(route[1].x, 0)

    def test_evasion_does_not_route_through_observed_threat_when_prediction_moves_away(self):
        world = VoxelWorld(0.1, 0.23)
        world.update([], [], None, map_bounds=(-3, -3, 3, 3))
        route = evade_objective_route(world, Pose2(0, 0), Pose2(0.7, 0),
                                     Pose2(2, 0), Pose2(0.7, 2))
        self.assertEqual(route, [])

    def test_evasion_rejects_objective_inside_guardian_zone(self):
        world = VoxelWorld(0.1, 0.23)
        world.update([], [], None, map_bounds=(-3, -3, 3, 3))
        self.assertEqual(evade_objective_route(
            world, Pose2(0, 0), Pose2(1.2, 0), Pose2(1.2, 0)), [])

    def test_guardian_in_capture_range_faces_prey_without_orbiting(self):
        world = VoxelWorld(0.15, 0.23)
        world.update([], [], None, map_bounds=(-3, -3, 3, 3))
        own, enemy = Pose2(0.41, 0, pi), Pose2(0, 0)
        target = capture_goal(world, own, enemy)
        self.assertAlmostEqual(target.x, own.x)
        self.assertAlmostEqual(target.y, own.y)
        self.assertAlmostEqual(abs(target.yaw), pi)

    def test_guardian_radial_approach_retains_continuous_capture_distance(self):
        world = VoxelWorld(0.15, 0.23)
        world.update([], [], None, map_bounds=(-3, -3, 3, 3))
        target = capture_goal(world, Pose2(1, 0), Pose2(0, 0))
        self.assertAlmostEqual(target.x, 0.39)
        self.assertAlmostEqual(target.y, 0)


if __name__ == "__main__":
    unittest.main()
