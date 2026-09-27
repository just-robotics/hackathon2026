import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1] / "src"
for package in ("hsl_decision", "hsl_planning", "hsl_debug_control", "hsl_sim_adapter"):
    sys.path.insert(0, str(ROOT / package))

from hsl_debug_control.core import follow, safe_follow, safe_mpc_command
from hsl_decision.core import (CAPTURE, EVADE, GOAL, PURSUE, SEARCH, STOP,
                               WAIT, DecisionPolicy, Observation, Pose2 as DecisionPose,
                               distance_to_polygon)
from hsl_planning.core import (Pose2, VoxelWorld, astar, capture_goal, coverage_target,
                               local_rollout, reachable_target)
from hsl_sim_adapter.cloud import transform
from hsl_sim_adapter.patrol import patrol_command
from hsl_sim_adapter.visibility import StaticGrid, opponent_visible
from hsl_sim_adapter.metrics import RunMetrics, capture_possible, timing_summary


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

    def test_guardian_searches_then_pursues_and_captures(self):
        policy = DecisionPolicy("guardian", self.area)
        initial = policy.step(self.observation())
        self.assertEqual(initial.behavior, SEARCH)
        self.assertEqual((initial.target.x, initial.target.y), (4.0, -2.0))
        self.assertEqual(policy.step(self.observation(now=12, opponent=DecisionPose(2, 0))).behavior,
                         PURSUE)
        self.assertEqual(policy.step(self.observation(now=14, opponent=DecisionPose(0.5, 0))).behavior,
                         CAPTURE)

    def test_guardian_searches_last_seen_position_after_track_expires(self):
        policy = DecisionPolicy("guardian", self.area)
        lost = self.observation(now=12, opponent=DecisionPose(2, 1))
        lost = Observation(lost.now, lost.own, lost.own_stamp, lost.opponent, 10,
                           lost.scan_stamp, lost.map_stamp, lost.allowed)
        result = policy.step(lost)
        self.assertEqual(result.behavior, SEARCH)
        self.assertEqual((result.target.x, result.target.y), (2, 1))

    def test_guardian_sweeps_after_reaching_last_seen_and_stops_on_capture(self):
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
        self.assertEqual(policy.step(captured).reason, "opponent captured")

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
        run.contact(11, "wall")
        run.contact(11.1, "wall")
        run.contact(11.2, None)
        run.contact(11.7, "robot")
        run.stop(12)
        report = run.snapshot(12)
        self.assertAlmostEqual(report["mean_speed_mps"], 0.175)
        self.assertEqual(report["collisions"], 2)
        self.assertEqual(report["wall_collisions"], 1)
        self.assertEqual(report["robot_collisions"], 1)
        self.assertEqual(report["opponent_visible_fraction"], 0.5)
        self.assertGreater(report["angular_accel_rms_radps2"], 0)

    def test_capture_requires_distance_heading_and_clear_line(self):
        data = [0] * (20 * 20)
        grid = StaticGrid(0.1, 20, 20, 0, 0, data)
        self.assertTrue(capture_possible((0.5, 1.0, 0), (0.9, 1.0, 0), grid))
        self.assertFalse(capture_possible((0.5, 1.0, 3.14), (0.9, 1.0, 0), grid))
        self.assertFalse(capture_possible((0.5, 1.0, 0), (1.0, 1.0, 0), grid))
        data[10 * 20 + 7] = 100
        self.assertFalse(capture_possible((0.5, 1.0, 0), (0.9, 1.0, 0), grid))

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
