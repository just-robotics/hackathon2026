import sys
import unittest
from math import cos, hypot, pi, sin
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1] / "src"
for package in ("hsl_decision", "hsl_planning", "hsl_debug_control", "hsl_sim_adapter"):
    sys.path.insert(0, str(ROOT / package))

from hsl_debug_control.core import follow, safe_follow, safe_mpc_command
from hsl_decision.core import (CAPTURE, EVADE, GOAL, PURSUE, SEARCH, STOP,
                               WAIT, DecisionPolicy, Observation, Pose2 as DecisionPose,
                               distance_to_polygon, intercept_point)
from hsl_planning.core import (Pose2, VoxelWorld, astar, capture_goal, coverage_target,
                               curved_guidance,
                               local_guidance, local_rollout, reachable_target,
                               reachable_intercept,
                               recovery_heading, recovery_step,
                               reusable_local_guidance, reusable_route,
                               route_curve_guidance, safe_segment)
from hsl_sim_adapter.cloud import transform
from hsl_sim_adapter.patrol import patrol_command
from hsl_sim_adapter.visibility import StaticGrid, opponent_visible
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
        self.assertLess(result.target.x, 0)
        far = self.observation(now=12, opponent=DecisionPose(3, 0))
        self.assertEqual(policy.step(far).behavior, GOAL)

    def test_explorer_keeps_escaping_until_clear_then_avoids_danger_on_goal_route(self):
        policy = DecisionPolicy("explorer", self.area)
        first = policy.step(self.observation(opponent=DecisionPose(0.8, 0)))
        self.assertEqual(first.behavior, EVADE)
        self.assertLess(first.target.x, 0)
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

    def test_guardian_searches_then_pursues_and_captures(self):
        policy = DecisionPolicy("guardian", self.area)
        initial = policy.step(self.observation())
        self.assertEqual(initial.behavior, SEARCH)
        self.assertEqual((initial.target.x, initial.target.y), (4.0, -2.0))
        self.assertEqual(policy.step(self.observation(now=12, opponent=DecisionPose(2, 0))).behavior,
                         PURSUE)
        capture = policy.step(self.observation(now=14, opponent=DecisionPose(0.5, 0)))
        self.assertEqual(capture.behavior, CAPTURE)
        self.assertLess(capture.tolerance, 0.03)
        self.assertGreater(abs(capture.target.x - 0.5), 0.4)
        self.assertLess(abs(capture.target.x - 0.5), 0.45)
        self.assertLess(capture.max_speed, 0.2)

    def test_guardian_brakes_before_capture_range(self):
        speeds = []
        for distance in (0.8, 0.6, 0.5, 0.42):
            policy = DecisionPolicy("guardian", self.area)
            speeds.append(policy.step(self.observation(
                opponent=DecisionPose(distance, 0))).max_speed)
        self.assertTrue(all(a > b for a, b in zip(speeds, speeds[1:])))
        self.assertGreater(speeds[1], 0.2)
        self.assertLess(speeds[2], 0.12)
        self.assertLessEqual(speeds[3], 0.08)

    def test_guardian_searches_last_seen_position_after_track_expires(self):
        policy = DecisionPolicy("guardian", self.area)
        lost = self.observation(now=12, opponent=DecisionPose(2, 1))
        lost = Observation(lost.now, lost.own, lost.own_stamp, lost.opponent, 10,
                           lost.scan_stamp, lost.map_stamp, lost.allowed)
        result = policy.step(lost)
        self.assertEqual(result.behavior, SEARCH)
        self.assertEqual((result.target.x, result.target.y), (2, 1))

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

    def test_local_straight_reference_is_stable_until_done_or_blocked(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([], [], None, map_bounds=(-1, -1, 4, 1))
        path = local_guidance(world, Pose2(0, 0),
                              [Pose2(0.5, 0), Pose2(1.5, 0)])
        self.assertIs(reusable_local_guidance(world, Pose2(0.4, 0.02), path), path)
        self.assertFalse(reusable_local_guidance(world, Pose2(0.4, 0.2), path))
        self.assertFalse(reusable_local_guidance(world, Pose2(1.25, 0), path))
        world.update([(0.8, 0, 0.3)], [], None, map_bounds=(-1, -1, 4, 1))
        self.assertFalse(reusable_local_guidance(world, Pose2(0.4, 0), path))

    def test_smooth_curve_starts_forward_and_reuses_safe_geometry(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([], [], None, map_bounds=(-1, -1, 3, 2))
        own = Pose2(0, 0, 0.4)
        straight = local_guidance(world, own, [Pose2(1.5, 0)])
        decisions = {}
        curve = curved_guidance(world, own, straight, diagnostics=decisions)
        self.assertEqual(decisions, {"accepted": 1})
        self.assertNotEqual(curve, straight)
        self.assertGreater(curve[1].x, own.x)
        self.assertGreater(curve[1].y, own.y)
        self.assertAlmostEqual(
            (curve[1].y - own.y) / (curve[1].x - own.x),
            sin(own.yaw) / cos(own.y), delta=0.1)
        self.assertIs(reusable_local_guidance(world, curve[8], curve), curve)
        world.update([(curve[14].x, curve[14].y, 0.3)], [], None,
                     map_bounds=(-1, -1, 3, 2))
        self.assertFalse(reusable_local_guidance(world, curve[8], curve))

    def test_curve_handles_moderate_turn_but_falls_back_when_blocked(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([], [], None, map_bounds=(-1, -1, 3, 2))
        own = Pose2(0, 0, 0.7)
        straight = local_guidance(world, own, [Pose2(1.5, 0)])
        curve = curved_guidance(world, own, straight)
        self.assertNotEqual(curve, straight)
        obstacle = curve[len(curve) // 2]
        world.update([(obstacle.x, obstacle.y, 0.3)], [], None,
                     map_bounds=(-1, -1, 3, 2))
        self.assertEqual(curved_guidance(world, own, straight), straight)

    def test_curve_requires_tracking_room_beside_wall(self):
        world = VoxelWorld(0.1, 0.2)
        wall = [(x * 0.1, 0.35, 0.3) for x in range(0, 17)]
        world.update(wall, [], None, map_bounds=(-1, -1, 3, 2))
        own = Pose2(0, 0, 0.7)
        straight = local_guidance(world, own, [Pose2(1.5, 0)])
        self.assertTrue(straight)
        self.assertEqual(curved_guidance(world, own, straight), straight)

    def test_checked_curve_can_pass_the_next_global_bend(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([(0.7, 0, 0.3)], [], None,
                     map_bounds=(-1, -1, 3, 2))
        own = Pose2(0, 0, 0.6)
        route = [Pose2(0.2, 0), Pose2(0.4, 0), Pose2(0.6, 0.15),
                 Pose2(0.8, 0.4), Pose2(1, 0.6), Pose2(1.2, 0.6),
                 Pose2(1.4, 0.6)]
        straight = local_guidance(world, own, route)
        self.assertLess(straight[-1].x, 0.4)
        curve = route_curve_guidance(world, own, route, straight)
        self.assertGreater(curve[-1].x, 1.0)
        self.assertTrue(all(safe_segment(world, a, b) for a, b in
                            zip(curve, curve[1:])))
        world.update([(curve[8].x, curve[8].y, 0.3), (0.7, 0, 0.3)],
                     [], None, map_bounds=(-1, -1, 3, 2))
        self.assertFalse(route_curve_guidance(world, own, route, straight))

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

    def test_local_path_must_increase_clearance_if_already_near_wall(self):
        world = VoxelWorld(0.1, 0.2)
        wall = [(x * 0.1, 0.27, 0.3) for x in range(16)]
        world.update(wall, [], None, map_bounds=(-1, -1, 3, 2))
        own = Pose2(0, 0)
        self.assertFalse(safe_segment(world, own, Pose2(1, 0)))
        self.assertTrue(safe_segment(world, own, Pose2(0.3, -0.3)))

    def test_local_guidance_does_not_cut_wall_and_recovery_turns_inward(self):
        world = VoxelWorld(0.1, 0.2)
        wall = [(0.6, y * 0.1, 0.3) for y in range(-5, 6)]
        world.update(wall, [], None, map_bounds=(-1, -1, 2, 1))
        self.assertFalse(local_guidance(world, Pose2(0, 0),
                                        [Pose2(1, 0)]))
        near_boundary = Pose2(1.72, 0, 0)
        heading = recovery_heading(world, near_boundary)
        self.assertIsNotNone(heading)
        self.assertLess(cos(heading), 0)

    def test_recovery_can_choose_a_short_step_in_a_tight_free_pocket(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([], [], None, map_bounds=(-0.45, -0.45, 0.45, 0.45))
        self.assertIsNotNone(recovery_heading(world, Pose2(0, 0)))
        self.assertTrue(local_guidance(world, Pose2(0, 0),
                                       [Pose2(0.09, 0)], min_step=0.04))

    def test_recovery_uses_the_length_it_actually_checked(self):
        world = VoxelWorld(0.1, 0.2)
        walls = [(x, y, 0.3) for x, y in
                 ((0.28, 0), (-0.43, 0), (0, 0.28), (0, -0.28))]
        world.update(walls, [], None, map_bounds=(-1, -1, 1, 1))
        own = Pose2(0, 0, pi)
        step = recovery_step(world, own)
        self.assertIsNotNone(step)
        heading, length = step
        self.assertLessEqual(length, 0.15)
        target = Pose2(length * cos(heading), length * sin(heading))
        self.assertTrue(local_guidance(world, own, [target], min_step=0.04))

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
        local = local_rollout(world, own, route, opponent,
                              clearance=0.85, weight=6.0)
        self.assertGreater(local[-1].x, own.x)

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

    def test_local_rollout_recovers_after_tracking_crosses_safety_margin(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([], [], None, map_bounds=(-0.5, -0.5, 1.0, 0.5))
        own = Pose2(0.75, 0.0, 3.14159265359)
        self.assertFalse(world.inside_map(own.x, own.y, 0.3))
        route = local_rollout(world, own, [own, Pose2(0.1, 0)],
                              max_speed=0.3)
        self.assertTrue(route)
        self.assertLess(route[-1].x, own.x)

    def test_guardian_coverage_target_is_free_and_reachable(self):
        world = VoxelWorld(0.15, 0.2)
        world.update([], [], Pose2(0, 0),
                     {world.cell(x * 0.15, y * 0.15)
                      for x in range(25) for y in range(15)})
        target = coverage_target(world, Pose2(0, 0), [Pose2(0, 0)])
        self.assertIsNotNone(target)
        self.assertIn(world.cell(target.x, target.y), world.free)
        self.assertTrue(astar(world, Pose2(0, 0), target))

    def test_local_rollout_avoids_immediate_obstacle(self):
        world = VoxelWorld(0.1, 0.2)
        world.update([(0.45, 0, 0.3)], [], None)
        path = local_rollout(world, Pose2(0, 0), [Pose2(0, 0), Pose2(1, 0)],
                             max_speed=0.4)
        self.assertTrue(path)
        self.assertTrue(all(not world.blocked(p.x, p.y) for p in path))

    def test_local_rollout_reports_blocked_when_already_facing_obstacle(self):
        world = VoxelWorld(0.15, 0.23)
        world.update([(0.25, 0, 0.3)], [], None)
        self.assertFalse(local_rollout(
            world, Pose2(0, 0, 0), [Pose2(0, 0), Pose2(1, 0)],
            max_speed=0.5))

    def test_local_rollout_can_escape_opponent_clearance(self):
        world = VoxelWorld(0.1, 0.18)
        world.update([], [], None)
        path = local_rollout(world, Pose2(0, 0, 3.14159265359),
                             [Pose2(0, 0), Pose2(-1, 0)],
                             Pose2(0.4, 0), clearance=0.65,
                             max_speed=0.4)
        self.assertLess(path[-1].x, -0.1)

    def test_local_rollout_leaves_quantized_occupied_start(self):
        world = VoxelWorld(0.15, 0.23)
        world.occupied.add(world.cell(0, 0))
        path = local_rollout(world, Pose2(0, 0), [Pose2(0, 0), Pose2(1, 0)],
                             max_speed=0.3)
        self.assertGreater(path[-1].x, 0.15)
        self.assertTrue(all(not world.blocked(p.x, p.y)
                            for p in path if world.cell(p.x, p.y) != world.cell(0, 0)))

    def test_astar_and_rollout_escape_soft_inflation_near_wall(self):
        world = VoxelWorld(0.15, 0.23)
        world.update([(x * 0.05, 1.1, 0.3) for x in range(20)], [], None)
        own = Pose2(0.6, 0.98, -1.57079632679)
        self.assertTrue(world.blocked(own.x, own.y))
        self.assertLess(world.obstacle_clearance(own.x, own.y), 0.19)
        route = astar(world, own, Pose2(0.6, 0.5))
        self.assertTrue(route)
        local = local_rollout(world, own, route, max_speed=0.5)
        self.assertLess(local[-1].y, own.y - 0.1)
        self.assertGreater(world.obstacle_clearance(local[-1].x, local[-1].y),
                           world.obstacle_clearance(own.x, own.y))

    def test_local_rollout_uses_safe_prefix_before_obstacle(self):
        world = VoxelWorld(0.15, 0.23)
        world.occupied.add(world.cell(0.45, 0))
        path = local_rollout(world, Pose2(0, 0), [Pose2(0, 0), Pose2(1, 0)],
                             max_speed=0.5)
        self.assertGreater(path[-1].x, 0.0)
        self.assertTrue(all(not world.blocked(p.x, p.y) for p in path))

    def test_local_rollout_issues_stable_heading_when_turning(self):
        world = VoxelWorld(0.15, 0.23)
        path = local_rollout(world, Pose2(0, 0), [Pose2(0, 0), Pose2(0, 1)],
                             max_speed=0)
        self.assertEqual(len(path), 2)
        self.assertAlmostEqual(path[-1].yaw, 1.57079632679)

    def test_local_rollout_targets_corner_before_far_side(self):
        world = VoxelWorld(0.15, 0.23)
        world.occupied.update({world.cell(x, 0) for x in (0.15, 0.3, 0.45)})
        route = [Pose2(0, 0), Pose2(0, -0.15), Pose2(0.15, -0.15),
                 Pose2(0.3, -0.15), Pose2(0.45, -0.15)]
        path = local_rollout(world, Pose2(0, 0), route, max_speed=0.5)
        self.assertEqual(len(path), 2)
        self.assertAlmostEqual(path[-1].yaw, -1.57079632679)
        aligned = local_rollout(world, Pose2(0, 0, -1.57079632679),
                                route, max_speed=0.5)
        self.assertLess(aligned[-1].y, -0.05)

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

    def test_opponent_requires_clear_map_line_and_lidar_cluster(self):
        data = [0] * (40 * 30)
        grid = StaticGrid(0.1, 40, 30, 0, 0, data)
        own, enemy = (0.5, 1.0), (1.5, 1.0)
        hits = [(1.4, 0.9, 0.3), (1.4, 1.0, 0.3), (1.4, 1.1, 0.3)]
        self.assertFalse(opponent_visible(grid, own, enemy, []))
        self.assertTrue(opponent_visible(grid, own, enemy, hits))
        for row in range(30):
            data[row * 40 + 10] = 100
        self.assertFalse(opponent_visible(grid, own, enemy, hits))

    def test_static_wall_returns_are_not_opponent_detections(self):
        data = [0] * (40 * 30)
        data[10 * 40 + 10] = 100
        grid = StaticGrid(0.1, 40, 30, 0, 0, data)
        wall_hits = [(1.04, 1.04, 0.3)] * 3
        self.assertFalse(opponent_visible(grid, (0.5, 1.0), (0.85, 1.0), wall_hits))

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
