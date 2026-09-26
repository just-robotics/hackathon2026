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
from hsl_planning.core import (Pose2, VoxelWorld, astar, capture_goal,
                               local_rollout, reachable_target)
from hsl_sim_adapter.cloud import transform


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
        self.assertEqual(policy.step(self.observation()).behavior, SEARCH)
        self.assertEqual(policy.step(self.observation(now=12, opponent=DecisionPose(2, 0))).behavior,
                         PURSUE)
        self.assertEqual(policy.step(self.observation(now=14, opponent=DecisionPose(0.5, 0))).behavior,
                         CAPTURE)

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


if __name__ == "__main__":
    unittest.main()
