"""ROS transport and bounded refresh for the pure planner."""

from copy import deepcopy
import json
import struct
import random
from math import atan2, cos, hypot, sin
from time import perf_counter

import rclpy
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float32, String

from .obstacle_memory import ObstacleMemory
from .obstacle_filter import SmallBoxFilter
from hsl_sim_adapter.cloud import make_cloud

from .core import (Pose2, VoxelWorld, astar, moving_capture_goal, coverage_target, reachable_frontier_route,
                   reachable_intercept, navigation_obstacles, evade_target, evade_objective_route,
                   path_heading_error, reachable_target,
                   checked_recovery_target, turn_alignment_is_progress, safe_segment,
                   reusable_route, continuous_short_goal_route,
                   smooth_intercept_target)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def odom_pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return Pose2(p.x, p.y, yaw)


def read_xyz(msg, limit=16000):
    fields = {field.name: field for field in msg.fields}
    if any(name not in fields or fields[name].datatype != PointField.FLOAT32
           for name in ("x", "y", "z")):
        return []
    endian = ">" if msg.is_bigendian else "<"
    count = msg.width * msg.height
    stride = max(1, count // limit)
    unpack = struct.Struct(endian + "f").unpack_from
    points = []
    data = msg.data
    for index in range(0, count, stride):
        row, col = divmod(index, msg.width)
        offset = row * msg.row_step + col * msg.point_step
        try:
            x, y, z = (unpack(data, offset + fields[name].offset)[0]
                       for name in ("x", "y", "z"))
        except (struct.error, ValueError):
            continue
        if all(abs(value) < 10000 for value in (x, y, z)):
            points.append((x, y, z))
    return points


def make_path(node, poses):
    path = Path()
    path.header.stamp = node.get_clock().now().to_msg()
    path.header.frame_id = node.frame
    for item in poses:
        stamped = PoseStamped()
        stamped.header = path.header
        stamped.pose.position.x = item.x
        stamped.pose.position.y = item.y
        stamped.pose.orientation.z = sin(item.yaw / 2)
        stamped.pose.orientation.w = cos(item.yaw / 2)
        path.poses.append(stamped)
    return path


class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__("trajectory_planner")
        self.declare_parameter("planning_frame", "map")
        self.declare_parameter("resolution", 0.15)
        self.declare_parameter("robot_radius", 0.23)
        self.declare_parameter("pose_timeout", 1.2)
        self.declare_parameter("scan_timeout", 1.8)
        self.declare_parameter("intent_timeout", 1.0)
        self.declare_parameter("random_seed", 0)
        self.declare_parameter("require_match_active", False)
        self.require_match_active = self.get_parameter("require_match_active").value
        self.match_state = None
        self.declare_parameter("role", "explorer")
        self.frame = self.get_parameter("planning_frame").value
        self.role = self.get_parameter("role").value
        if self.role not in ("explorer", "guardian"):
            raise ValueError("role must be explorer or guardian")
        self.local_safety_margin = 0.0  # robot_radius already includes .052 m clearance
        self.pose_timeout = self.get_parameter("pose_timeout").value
        self.scan_timeout = self.get_parameter("scan_timeout").value
        self.intent_timeout = self.get_parameter("intent_timeout").value
        self.random_seed = int(self.get_parameter("random_seed").value)
        self.declare_parameter("max_speed", 0.5)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.rng = random.Random(self.random_seed)
        self.world = VoxelWorld(self.get_parameter("resolution").value,
                                self.get_parameter("robot_radius").value)
        self.own = None
        self.measured_omega = 0.0
        self.opponent = None
        self.evade_waypoint = None
        self.alignment_until = 0.0
        self.nominal_retry = 0
        self.intent = None
        self.scan_stamp = 0.0
        self.map_points = []
        self.grid_points = []
        self.grid_free = set()
        self.grid_bounds = None
        default_arena_bounds = [-1000000.0, -1000000.0,
                                1000000.0, 1000000.0]
        self.declare_parameter("arena_bounds", default_arena_bounds)
        configured_bounds = list(self.get_parameter("arena_bounds").value)
        if (len(configured_bounds) != 4 or
                configured_bounds[0] >= configured_bounds[2] or
                configured_bounds[1] >= configured_bounds[3]):
            raise ValueError("arena_bounds must be [min_x,min_y,max_x,max_y]")
        self.arena_bounds = (tuple(configured_bounds)
                             if configured_bounds != default_arena_bounds else None)
        self.scan_points = []
        self.obstacle_memory = ObstacleMemory()
        self.small_box_filter = SmallBoxFilter()
        self.static_grid = None
        self.dirty = True
        self.global_path = []
        self.global_target = None
        self.pursuit_target = None
        self.route_behavior = None
        self.search_waypoint = None
        self.search_visited = []
        self.progress_pose = None
        self.progress_since = None
        self.progress_behavior = None
        self.progress_heading_error = None
        self.recovery_avoid = None
        self.recovery_until = 0.0
        self.recovery_attempt = 0
        self.recovery_goal = None
        self.recovery_origin = None
        self.create_subscription(Odometry, "navigation/self", self.on_own, 10)
        self.create_subscription(Odometry, "navigation/opponent", self.on_opponent, 10)
        self.create_subscription(PlanningIntent, "navigation/intent", self.on_intent, 10)
        self.create_subscription(PointCloud2, "navigation/map_points", self.on_map,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "navigation/scan", self.on_scan,
                                 qos_profile_sensor_data)
        grid_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.obstacle_scan_pub = self.create_publisher(PointCloud2, "navigation/obstacle_scan", qos_profile_sensor_data)
        self.obstacle_filter_diag_pub = self.create_publisher(String, "navigation/obstacle_filter_diagnostics", 10)
        self.obstacle_grid_pub = self.create_publisher(OccupancyGrid, "navigation/obstacle_grid", grid_qos)
        self.create_timer(.2, self.publish_obstacle_grid)
        if self.require_match_active:
            self.create_subscription(Bool, "/match/active", self.on_match_active, grid_qos)
        self.create_subscription(OccupancyGrid, "navigation/known_grid",
                                 self.on_known_grid, grid_qos)
        self.global_pub = self.create_publisher(Path, "navigation/global_path", 10)
        self.native_reference_pub = self.create_publisher(Path, "navigation/nav2_reference", 10)
        self.status_pub = self.create_publisher(String, "navigation/global_status", 10)
        self.planning_diag_pub = self.create_publisher(String, "navigation/planning_diagnostics", 10)
        self.planning_diagnostics = {}
        self.cycle_pub = self.create_publisher(Float32, "navigation/planner_cycle_ms", 10)
        self.create_timer(0.2, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_match_active(self, msg):
        self.match_state = (msg.data, self.now())

    def on_own(self, msg):
        if msg.header.frame_id == self.frame:
            self.own = (odom_pose(msg), seconds(msg.header.stamp))
            self.measured_omega = float(msg.twist.twist.angular.z)

    def on_opponent(self, msg):
        if msg.header.frame_id == self.frame:
            self.opponent = (odom_pose(msg), seconds(msg.header.stamp), msg.twist.twist)

    def on_intent(self, msg):
        if msg.header.frame_id == self.frame:
            if self.intent is not None and msg.behavior != self.intent.behavior:
                # A manoeuvre selected for another task is no longer authoritative.
                self.recovery_goal = None
                self.recovery_origin = None
                self.recovery_avoid = None
                self.recovery_until = 0.0
                self.evade_waypoint = None
                self.nominal_retry = 0
            if msg.behavior != 5:
                self.search_waypoint = None
                self.search_visited = []
            self.intent = msg

    def on_map(self, msg):
        if msg.header.frame_id == self.frame:
            points, _ = self.small_box_filter.filter(read_xyz(msg), seconds(msg.header.stamp))
            self.map_points = points.tolist()
            self.dirty = True

    def on_scan(self, msg):
        if msg.header.frame_id == self.frame:
            raw = read_xyz(msg, 5000)
            scan_stamp = seconds(msg.header.stamp)
            protected = ([[self.opponent[0].x, self.opponent[0].y]]
                if self.opponent and 0 <= scan_stamp-self.opponent[1] <= .3 else [])
            points, diagnostic = self.small_box_filter.filter(raw, scan_stamp, protected)
            self.scan_points = points.tolist()
            self.obstacle_memory.forget(self.small_box_filter.ignored)
            self.obstacle_scan_pub.publish(make_cloud(msg.header, self.scan_points))
            self.obstacle_filter_diag_pub.publish(String(data=json.dumps(dict(
                stamp_s=seconds(msg.header.stamp), **diagnostic))))
            self.scan_stamp = seconds(msg.header.stamp)
            if self.own and abs(self.scan_stamp-self.own[1]) <= self.pose_timeout:
                self.obstacle_memory.update(self.scan_points, self.own[0], self.scan_stamp)
                self.publish_obstacle_grid()
            self.dirty = True

    def on_known_grid(self, msg):
        if msg.header.frame_id != self.frame or msg.info.width <= 0 or msg.info.resolution <= 0:
            return
        self.static_grid = deepcopy(msg)
        self.small_box_filter.set_grid(msg.info.resolution,
            [msg.info.origin.position.x,msg.info.origin.position.y],
            msg.info.width,msg.info.height,msg.data)
        resolution = msg.info.resolution
        width = msg.info.width
        origin = msg.info.origin.position
        self.grid_bounds = self.arena_bounds or (
            origin.x, origin.y,
            origin.x + width * resolution,
            origin.y + msg.info.height * resolution)
        points = []
        free = set()
        for index, value in enumerate(msg.data):
            row, col = divmod(index, width)
            x = origin.x + (col + 0.5) * resolution
            y = origin.y + (row + 0.5) * resolution
            if value >= 50:
                points.append((x, y, 0.3))
            elif value == 0:
                free.add(self.world.cell(x, y))
        self.grid_points = points
        self.grid_free = free
        self.dirty = True

    def observed_obstacles(self):
        enemy = (self.opponent[0] if self.role == "guardian" and self.opponent
                 and 0 <= self.now()-self.opponent[1] <= 1.0 else None)
        return self.obstacle_memory.points(self.now(), exclude=enemy)

    def publish_obstacle_grid(self):
        if self.static_grid is None:
            return
        grid = deepcopy(self.static_grid)
        grid.header.stamp = self.get_clock().now().to_msg()
        origin, resolution = grid.info.origin.position, grid.info.resolution
        for x,y,_ in self.observed_obstacles():
            col = int((x-origin.x)//resolution)
            row = int((y-origin.y)//resolution)
            if 0 <= col < grid.info.width and 0 <= row < grid.info.height:
                grid.data[row*grid.info.width+col] = 100
        self.obstacle_grid_pub.publish(grid)

    def publish_empty(self, reason):
        self.planning_diagnostics["global_status"] = reason
        self.global_path = []
        self.global_pub.publish(make_path(self, []))
        self.native_reference_pub.publish(make_path(self, []))
        self.status_pub.publish(String(data=reason))

    def tick(self):
        started = perf_counter()
        self.planning_diagnostics = {"sim_t_s": self.now(), "role": self.role}
        try:
            self._tick()
        finally:
            self.planning_diagnostics.update({
                "route_cells": len(self.global_path),
                "route_end": ([self.global_path[-1].x, self.global_path[-1].y]
                              if self.global_path else None),
                "recovery_goal": ([self.recovery_goal.x, self.recovery_goal.y]
                                  if self.recovery_goal else None)})
            self.planning_diag_pub.publish(String(data=json.dumps(self.planning_diagnostics)))
            self.cycle_pub.publish(Float32(data=(perf_counter() - started) * 1000))

    def _tick(self):
        now = self.now()
        if (self.require_match_active and
                not (self.match_state and self.match_state[0] and
                     0 <= now - self.match_state[1] <= 0.5)):
            self.progress_pose = None
            self.progress_since = None
            self.nominal_retry = 0
            self.publish_empty("WAIT_OR_STOP")
            return
        if not self.own or not self.intent or self.intent.behavior in (0, 1):
            self.publish_empty("WAIT_OR_STOP")
            return
        if (now - self.own[1] > self.pose_timeout
                or now - self.scan_stamp > self.scan_timeout
                or now - seconds(self.intent.header.stamp) > self.intent_timeout):
            self.publish_empty("STALE_INPUT")
            return
        own = self.own[0]
        intent = self.intent
        self.planning_diagnostics.update({
            "behavior": int(intent.behavior), "own": [own.x, own.y],
            "own_stamp_s": self.own[1], "intent_stamp_s": seconds(intent.header.stamp),
            "raw_target": ([intent.target.position.x, intent.target.position.y]
                           if intent.has_target else None),
            "cached_target": ([self.global_target.x, self.global_target.y]
                              if self.global_target else None),
            "cached_route_end": ([self.global_path[-1].x, self.global_path[-1].y]
                                 if self.global_path else None)})
        pending_native_recovery = False
        heading_error = path_heading_error(own, self.global_path)
        if (self.progress_pose is None or self.progress_since is None or
                now < self.progress_since or intent.behavior != self.progress_behavior or
                hypot(own.x - self.progress_pose.x,
                      own.y - self.progress_pose.y) >= 0.2):
            self.progress_pose = own
            self.progress_since = now
            self.nominal_retry = 0
            self.progress_behavior = intent.behavior
            self.alignment_until = now + 8.0
            self.progress_heading_error = heading_error
        else:
            turning_progress = (
                turn_alignment_is_progress(self.role, intent.behavior) and
                now < self.alignment_until and abs(self.measured_omega) > 0.08 and
                heading_error is not None and
                self.progress_heading_error is not None and
                heading_error < self.progress_heading_error - 0.02)
            if turning_progress:
                self.progress_pose = own
                self.progress_since = now
            self.progress_heading_error = heading_error
        if (self.progress_since is not None and
                now - self.progress_since >= 4.0):
            blocked_ahead = next((point for point in self.global_path
                                  if hypot(point.x - own.x,
                                           point.y - own.y) >= 0.35), None)
            # A watchdog timeout is not evidence of an obstacle. Do not
            # blacklist a free corridor or the actual mission goal.
            candidate = blocked_ahead
            self.recovery_avoid = candidate if candidate is not None and self.world.blocked(candidate.x,candidate.y) else None
            self.recovery_until = now + 8.0
            self.recovery_attempt += 1
            self.global_path = []
            # First retry the nominal route/warm start. A useful turn in free
            # space must not immediately hand authority to an arbitrary escape.
            self.recovery_goal = None
            pending_native_recovery = self.nominal_retry > 0 and self.recovery_avoid is not None
            self.nominal_retry += 1
            self.recovery_origin = own
            self.progress_pose = own
            self.progress_since = now
            self.progress_heading_error = heading_error
            self.get_logger().warn("No translation for 4 s; retrying route (avoid only confirmed obstacles)")
        recovery_avoid = self.recovery_avoid if now < self.recovery_until else None
        enemy = (self.opponent[0] if self.opponent and now - self.opponent[1] <= 2.0
                 else None)
        enemy_velocity = (0.0, 0.0)
        if enemy:
            twist = self.opponent[2]
            enemy_velocity = (cos(enemy.yaw) * twist.linear.x - sin(enemy.yaw) * twist.linear.y,
                              sin(enemy.yaw) * twist.linear.x + cos(enemy.yaw) * twist.linear.y)
            enemy_future = Pose2(enemy.x + enemy_velocity[0],
                                 enemy.y + enemy_velocity[1], enemy.yaw)
        else:
            enemy_future = None
        self.planning_diagnostics.update({
            "enemy": [enemy.x, enemy.y] if enemy else None,
            "enemy_stamp_s": self.opponent[1] if enemy else None,
            "enemy_velocity": list(enemy_velocity),
            "enemy_prediction": [enemy_future.x, enemy_future.y] if enemy_future else None})
        if self.dirty:
            static_points, scan_points = navigation_obstacles(
                self.grid_points + self.observed_obstacles(), self.map_points, self.scan_points, enemy)
            self.world.update(static_points, scan_points, own,
                              self.grid_free, self.grid_bounds)
            self.dirty = False
        if pending_native_recovery or self.recovery_goal is not None:
            previous = self.recovery_goal
            self.recovery_goal = checked_recovery_target(
                self.world, own, previous, enemy, intent.opponent_clearance,
                self.local_safety_margin)
            if (self.recovery_goal is not None and
                    (previous is None or hypot(self.recovery_goal.x - previous.x,
                                               self.recovery_goal.y - previous.y) > 0.01)):
                self.recovery_origin = own
        if intent.behavior == 7 and enemy:
            target, capture_enemy = moving_capture_goal(
                self.world, own, enemy, enemy_velocity,
                pursuer_speed=self.max_speed or 0.3)
            self.planning_diagnostics["capture_enemy_prediction"] = [capture_enemy.x, capture_enemy.y]
            self.planning_diagnostics["capture_strategy"] = (
                "direct" if capture_enemy == enemy else "lead")
            target = target or Pose2(
                intent.target.position.x, intent.target.position.y)
        elif intent.has_target:
            q = intent.target.orientation
            target = Pose2(intent.target.position.x, intent.target.position.y,
                           atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))
            if intent.behavior == 6 and enemy:
                raw_intercept = target
                target = reachable_intercept(self.world, own, enemy, target)
                self.planning_diagnostics["validated_intercept"] = (
                    [target.x, target.y] if target else None)
                if target is not None and hypot(target.x - raw_intercept.x,
                                                 target.y - raw_intercept.y) > 0.001:
                    self.planning_diagnostics["intercept_rejection_geometry"] = {
                        "endpoint_blocked": self.world.blocked(raw_intercept.x, raw_intercept.y),
                        "endpoint_inside": self.world.inside_map(
                            raw_intercept.x, raw_intercept.y, self.world.robot_radius + 0.07),
                        "segment_safe_default": safe_segment(self.world, enemy, raw_intercept),
                        "segment_safe_without_recovery_margin": safe_segment(
                            self.world, enemy, raw_intercept, safety_margin=0.0)}
                target = smooth_intercept_target(self.pursuit_target, target)
                self.planning_diagnostics["smoothed_intercept"] = (
                    [target.x, target.y] if target else None)
                self.pursuit_target = target
            else:
                self.pursuit_target = None
        else:
            if intent.behavior == 5:
                if (self.search_waypoint is None or
                        hypot(own.x - self.search_waypoint.x,
                              own.y - self.search_waypoint.y) < 0.4):
                    self.search_visited.append(own)
                    self.search_waypoint = coverage_target(
                        self.world, own, self.search_visited,
                        rng=self.rng, tie_seed=self.random_seed)
                target = self.search_waypoint
            else:
                target = self.world.frontier(own, tie_seed=self.random_seed)
        objective_route = []
        if intent.behavior == 4 and enemy:
            separation = hypot(own.x - enemy.x, own.y - enemy.y)
            if separation < intent.opponent_clearance + 0.3:
                objective_route = evade_objective_route(
                    self.world, own, enemy,
                    target if target is not None and
                        self.world.cell(target.x, target.y) in self.world.free else None,
                    enemy_future, intent.opponent_clearance, intent.opponent_cost_weight,
                    tie_seed=self.random_seed + self.recovery_attempt,
                    candidate=(reusable_route(
                        self.world, own, self.global_path, self.global_target, target,
                        enemy_future, intent.opponent_clearance)
                        if self.route_behavior == intent.behavior else []))
                if objective_route:
                    if self.evade_waypoint is not None:
                        # Drop recovery for the replaced departure, but retain
                        # watchdog recovery while following the same objective.
                        self.recovery_goal = None
                    self.evade_waypoint = None
                previous = self.evade_waypoint
                if not objective_route and (previous is None or hypot(previous.x - own.x, previous.y - own.y) < 0.15 or
                        not self.world.inside_map(previous.x, previous.y) or
                        not safe_segment(self.world, own, previous, enemy,
                                         intent.opponent_clearance, self.local_safety_margin) or
                        (previous.x - own.x) * (own.x - enemy.x) +
                        (previous.y - own.y) * (own.y - enemy.y) < 0):
                    self.evade_waypoint = evade_target(
                        self.world, own, enemy, target, intent.opponent_clearance,
                        self.local_safety_margin)
                if self.evade_waypoint is not None:
                    target = self.evade_waypoint
            else:
                self.evade_waypoint = None
        requested_target = target
        self.planning_diagnostics["requested_target"] = (
            [target.x, target.y] if target else None)
        target = reachable_target(self.world, own, target,
                                  explore=intent.behavior == 3,
                                  tie_seed=self.random_seed + self.recovery_attempt,
                                  avoid=recovery_avoid)
        if (target is not None and requested_target is not None and
                intent.behavior in (2, 4, 6, 7) and
                hypot(target.x - requested_target.x,
                      target.y - requested_target.y) > 0.3 and
                hypot(target.x - own.x, target.y - own.y) <= intent.target_tolerance):
            target = (self.world.frontier(
                own, requested_target,
                tie_seed=self.random_seed + self.recovery_attempt,
                min_travel=max(0.6, intent.target_tolerance + 0.2),
                avoid=target) or target)
        if target is None and intent.behavior == 3:
            target = Pose2(own.x, own.y, own.yaw + 1.2)
        if target is None:
            self.publish_empty("NO_TARGET_OR_FRONTIER")
            return
        self.planning_diagnostics["reachable_target"] = [target.x, target.y]
        route = objective_route or (reusable_route(self.world, own, self.global_path,
                                self.global_target, target, enemy_future,
                                intent.opponent_clearance)
                 if self.route_behavior == intent.behavior else [])
        self.planning_diagnostics["route_source"] = (
            "objective" if objective_route else "reuse" if route else "search")
        if not route:
            route = astar(self.world, own, target, enemy_future,
                          intent.opponent_clearance, intent.opponent_cost_weight,
                          tie_seed=self.random_seed + self.recovery_attempt,
                          avoid=recovery_avoid)
            if not route:
                route = reachable_frontier_route(
                    self.world, own, target, enemy_future,
                    intent.opponent_clearance, intent.opponent_cost_weight,
                    tie_seed=self.random_seed + self.recovery_attempt,
                    avoid=recovery_avoid)
            self.global_path = route
            self.global_target = target
            self.route_behavior = intent.behavior
        else:
            self.global_path = route
            if objective_route:
                self.global_target = target
                self.route_behavior = intent.behavior
        # Keep the explorer objective continuous too: the raster endpoint can
        # lie outside the 8 cm center tolerance. Never extend a frontier route
        # across an unchecked segment or replace a temporary escape waypoint.
        explorer_center_route = (intent.behavior in (2, 4) and target is not None
            and hypot(target.x - intent.target.position.x,
                      target.y - intent.target.position.y) < 1e-6
            and self.global_path and hypot(self.global_path[-1].x - target.x,
                self.global_path[-1].y - target.y) <= self.world.resolution * 1.5)
        if explorer_center_route and len(self.global_path) >= 2 and safe_segment(
                self.world, self.global_path[-2], target,
                safety_margin=self.local_safety_margin):
            self.global_path[-1] = target
        if intent.behavior in (6, 7) or explorer_center_route:
            self.global_path = continuous_short_goal_route(
                self.world, own, self.global_path, target,
                safety_margin=self.local_safety_margin)
        if intent.behavior == 7 and target is not None and self.global_path:
            # Keep the continuous capture pose and its facing constraint. A*
            # raster centres can otherwise stop outside the capture radius.
            if len(self.global_path) >= 2 and safe_segment(self.world, self.global_path[-2], target,
                              safety_margin=self.local_safety_margin):
                self.global_path[-1] = target
        if (self.recovery_goal is not None and self.recovery_origin is not None and
                (hypot(own.x - self.recovery_origin.x,
                       own.y - self.recovery_origin.y) >= 0.45 or
                 hypot(own.x - self.recovery_goal.x,
                       own.y - self.recovery_goal.y) < 0.06)):
            self.recovery_goal = None
        if not self.global_path and self.recovery_goal is None:
            self.recovery_goal = checked_recovery_target(
                self.world, own, None, enemy, intent.opponent_clearance,
                self.local_safety_margin)
            if self.recovery_goal is not None:
                self.recovery_origin = own
            else:
                self.publish_empty("NO_GLOBAL_PATH")
                return
        if self.recovery_goal is not None:
            reference = [own, self.recovery_goal]
            status = "RECOVERY_ROUTE"
        else:
            reference = list(self.global_path)
            if reference:
                last = reference[-1]
                # Navigation goals have no required final orientation;
                # only capture needs the explicit facing constraint.
                heading = target.yaw
                if intent.behavior != 7 and len(reference) >= 2:
                    before = reference[-2]
                    heading = atan2(last.y - before.y, last.x - before.x)
                reference[-1] = Pose2(last.x, last.y, heading)
            status = "OK"
        self.planning_diagnostics["global_status"] = status
        self.global_pub.publish(make_path(self, self.global_path))
        self.native_reference_pub.publish(make_path(self, reference))
        self.status_pub.publish(String(data=status))
        return

def main():
    rclpy.init()
    node = TrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
