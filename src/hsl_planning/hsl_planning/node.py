"""ROS transport and bounded refresh for the pure planner."""

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
from std_msgs.msg import Float32, String

from .core import (Pose2, VoxelWorld, angle_error, astar, capture_goal, coverage_target,
                   curved_guidance,
                   dynamic_path_speed_limit,
                   route_curve_guidance,
                   reachable_intercept,
                   local_guidance, path_heading_error, reachable_target,
                   local_rollout, regulated_pure_pursuit_guidance,
                   recovery_step, turn_alignment_is_progress,
                   reusable_local_guidance, reusable_route,
                   smooth_intercept_target, slew_speed_limit)


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
        self.declare_parameter("role", "explorer")
        self.declare_parameter("speed_limit_max", 1.0)
        self.declare_parameter("speed_limit_min", 0.12)
        self.declare_parameter("speed_limit_lateral_accel", 0.08)
        self.declare_parameter("speed_limit_clearance_ramp", 0.32)
        self.declare_parameter("speed_limit_accel", 0.45)
        self.declare_parameter("speed_limit_decel", 0.45)
        self.frame = self.get_parameter("planning_frame").value
        self.role = self.get_parameter("role").value
        if self.role not in ("explorer", "guardian"):
            raise ValueError("role must be explorer or guardian")
        self.local_safety_margin = 0.14 if self.role == "explorer" else 0.12
        self.pose_timeout = self.get_parameter("pose_timeout").value
        self.scan_timeout = self.get_parameter("scan_timeout").value
        self.intent_timeout = self.get_parameter("intent_timeout").value
        self.random_seed = int(self.get_parameter("random_seed").value)
        self.rng = random.Random(self.random_seed)
        self.world = VoxelWorld(self.get_parameter("resolution").value,
                                self.get_parameter("robot_radius").value)
        self.own = None
        self.measured_speed = 0.0
        self.opponent = None
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
        self.dirty = True
        self.global_path = []
        self.global_target = None
        self.pursuit_target = None
        self.route_behavior = None
        self.local_path = []
        self.local_target = None
        self.local_behavior = None
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
        self.last_rollout_omega = None
        self.curve_diagnostics = {}
        self.curve_diagnostics_until = 0.0
        self.create_subscription(Odometry, "navigation/self", self.on_own, 10)
        self.create_subscription(Odometry, "navigation/opponent", self.on_opponent, 10)
        self.create_subscription(PlanningIntent, "navigation/intent", self.on_intent, 10)
        self.create_subscription(PointCloud2, "navigation/map_points", self.on_map,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "navigation/scan", self.on_scan,
                                 qos_profile_sensor_data)
        grid_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "navigation/known_grid",
                                 self.on_known_grid, grid_qos)
        self.global_pub = self.create_publisher(Path, "navigation/global_path", 10)
        self.local_pub = self.create_publisher(Path, "navigation/local_path", 10)
        self.speed_limit_pub = self.create_publisher(
            Float32, "navigation/speed_limit", 10)
        self.speed_clearance_pub = self.create_publisher(
            Float32, "navigation/speed_clearance", 10)
        self.speed_curvature_pub = self.create_publisher(
            Float32, "navigation/speed_curvature", 10)
        self.speed_alignment_pub = self.create_publisher(
            Float32, "navigation/speed_alignment", 10)
        self.status_pub = self.create_publisher(String, "navigation/planner_status", 10)
        self.cycle_pub = self.create_publisher(Float32, "navigation/planner_cycle_ms", 10)
        self.speed_limit_value = 0.0
        self.speed_limit_stamp = self.now()
        self.create_timer(0.2, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_own(self, msg):
        if msg.header.frame_id == self.frame:
            self.own = (odom_pose(msg), seconds(msg.header.stamp))
            self.measured_speed = abs(float(msg.twist.twist.linear.x))

    def on_opponent(self, msg):
        if msg.header.frame_id == self.frame:
            self.opponent = (odom_pose(msg), seconds(msg.header.stamp), msg.twist.twist)

    def on_intent(self, msg):
        if msg.header.frame_id == self.frame:
            if self.local_behavior is not None and msg.behavior != self.local_behavior:
                self.last_rollout_omega = None
            if msg.behavior != 5:
                self.search_waypoint = None
                self.search_visited = []
            self.intent = msg

    def on_map(self, msg):
        if msg.header.frame_id == self.frame:
            self.map_points = read_xyz(msg)
            self.dirty = True

    def on_scan(self, msg):
        if msg.header.frame_id == self.frame:
            self.scan_points = read_xyz(msg, 5000)
            self.scan_stamp = seconds(msg.header.stamp)
            self.dirty = True

    def on_known_grid(self, msg):
        if msg.header.frame_id != self.frame or msg.info.width <= 0 or msg.info.resolution <= 0:
            return
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

    def publish_empty(self, reason):
        self.global_path = []
        self.local_path = []
        self.speed_limit_value = 0.0
        self.speed_limit_stamp = self.now()
        self.speed_limit_pub.publish(Float32(data=0.0))
        for publisher in (self.speed_clearance_pub, self.speed_curvature_pub,
                          self.speed_alignment_pub):
            publisher.publish(Float32(data=0.0))
        self.global_pub.publish(make_path(self, []))
        self.local_pub.publish(make_path(self, []))
        self.status_pub.publish(String(data=reason))

    def publish_speed_limit(self, path, intent, now):
        requested = min(float(intent.max_speed),
                        float(self.get_parameter("speed_limit_max").value))
        profile = {}
        target = dynamic_path_speed_limit(
            self.world, path, requested, self.local_safety_margin,
            max_speed=float(self.get_parameter("speed_limit_max").value),
            min_speed=float(self.get_parameter("speed_limit_min").value),
            lateral_accel=float(
                self.get_parameter("speed_limit_lateral_accel").value),
            clearance_ramp=float(
                self.get_parameter("speed_limit_clearance_ramp").value),
            heading_error=path_heading_error(path[0], path)
            if path else None,
            diagnostics=profile)
        self.speed_clearance_pub.publish(
            Float32(data=float(profile["clearance_speed"])))
        self.speed_curvature_pub.publish(
            Float32(data=float(profile["curvature_speed"])))
        self.speed_alignment_pub.publish(
            Float32(data=float(profile["alignment_speed"])))
        dt = max(0.0, min(0.5, now - self.speed_limit_stamp))
        self.speed_limit_value = slew_speed_limit(
            self.speed_limit_value, target, dt,
            float(self.get_parameter("speed_limit_accel").value),
            float(self.get_parameter("speed_limit_decel").value))
        self.speed_limit_stamp = now
        self.speed_limit_pub.publish(
            Float32(data=float(self.speed_limit_value)))

    def tick(self):
        started = perf_counter()
        try:
            self._tick()
        finally:
            self.cycle_pub.publish(Float32(data=(perf_counter() - started) * 1000))

    def _tick(self):
        now = self.now()
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
        heading_error = path_heading_error(own, self.local_path)
        if (self.progress_pose is None or self.progress_since is None or
                now < self.progress_since or intent.behavior != self.progress_behavior or
                hypot(own.x - self.progress_pose.x,
                      own.y - self.progress_pose.y) >= 0.2):
            self.progress_pose = own
            self.progress_since = now
            self.progress_behavior = intent.behavior
            self.progress_heading_error = heading_error
        else:
            turning_progress = (
                turn_alignment_is_progress(self.role, intent.behavior) and
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
            self.recovery_avoid = blocked_ahead or self.global_target
            self.recovery_until = now + 8.0
            self.recovery_attempt += 1
            self.global_path = []
            step = recovery_step(self.world, own,
                                 safety_margin=self.local_safety_margin)
            self.recovery_goal = (Pose2(own.x + step[1] * cos(step[0]),
                                        own.y + step[1] * sin(step[0]))
                                  if step else None)
            self.recovery_origin = own
            self.progress_pose = own
            self.progress_since = now
            self.progress_heading_error = heading_error
            self.get_logger().warn("No translation for 4 s; retrying another corridor")
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
        if self.dirty:
            static_points = self.map_points + self.grid_points
            scan_points = self.scan_points
            if enemy:
                static_points = [p for p in static_points
                                 if hypot(p[0] - enemy.x, p[1] - enemy.y) > 0.45]
                scan_points = [p for p in scan_points
                               if hypot(p[0] - enemy.x, p[1] - enemy.y) > 0.45]
            self.world.update(static_points, scan_points, own,
                              self.grid_free, self.grid_bounds)
            self.dirty = False
        if intent.behavior == 7 and enemy:
            target = capture_goal(self.world, own, enemy) or Pose2(
                intent.target.position.x, intent.target.position.y)
        elif intent.has_target:
            q = intent.target.orientation
            target = Pose2(intent.target.position.x, intent.target.position.y,
                           atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))
            if intent.behavior == 6 and enemy:
                target = reachable_intercept(self.world, own, enemy, target)
                target = smooth_intercept_target(self.pursuit_target, target)
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
        requested_target = target
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
        route = (reusable_route(self.world, own, self.global_path,
                                self.global_target, target, enemy_future,
                                intent.opponent_clearance)
                 if self.route_behavior == intent.behavior else [])
        if not route:
            route = astar(self.world, own, target, enemy_future,
                          intent.opponent_clearance, intent.opponent_cost_weight,
                          tie_seed=self.random_seed + self.recovery_attempt,
                          avoid=recovery_avoid)
            if not route:
                frontier = (self.world.frontier(
                    own, target, tie_seed=self.random_seed, min_travel=0.6)
                    or self.world.frontier(own, target,
                                           tie_seed=self.random_seed))
                if frontier:
                    route = astar(self.world, own, frontier, enemy_future,
                                  intent.opponent_clearance, intent.opponent_cost_weight,
                                  tie_seed=self.random_seed + self.recovery_attempt,
                                  avoid=recovery_avoid)
            self.global_path = route
            self.global_target = target
            self.route_behavior = intent.behavior
        else:
            self.global_path = route
        if (self.recovery_goal is not None and self.recovery_origin is not None and
                (hypot(own.x - self.recovery_origin.x,
                       own.y - self.recovery_origin.y) >= 0.45 or
                 hypot(own.x - self.recovery_goal.x,
                       own.y - self.recovery_goal.y) < 0.06)):
            self.recovery_goal = None
        if not self.global_path and self.recovery_goal is None:
            step = recovery_step(self.world, own, enemy,
                                 intent.opponent_clearance,
                                 safety_margin=self.local_safety_margin)
            if step:
                self.recovery_goal = Pose2(own.x + step[1] * cos(step[0]),
                                           own.y + step[1] * sin(step[0]))
                self.recovery_origin = own
            else:
                self.publish_empty("NO_GLOBAL_PATH")
                return
        if self.recovery_goal is not None:
            self.local_path = []
            local = local_guidance(self.world, own, [self.recovery_goal], enemy,
                                   intent.opponent_clearance, min_step=0.04,
                                   safety_margin=self.local_safety_margin)
            if not local:
                self.recovery_goal = None
        elif hypot(own.x - target.x, own.y - target.y) <= intent.target_tolerance:
            self.local_path = []
            local = [own, Pose2(own.x, own.y, target.yaw)]
        else:
            same_target = (self.local_target is not None and
                           hypot(target.x - self.local_target.x,
                                 target.y - self.local_target.y) < 0.3)
            local = regulated_pure_pursuit_guidance(
                self.world, own, self.global_path, self.measured_speed,
                enemy, intent.opponent_clearance,
                safety_margin=self.local_safety_margin + 0.06,
                diagnostics=self.curve_diagnostics)
            if local:
                self.last_rollout_omega = None
            else:
                local = (reusable_local_guidance(
                    self.world, own, self.local_path, enemy,
                    intent.opponent_clearance, self.local_safety_margin)
                    if same_target and self.local_behavior == intent.behavior
                    else [])
            if not local:
                straight = local_guidance(self.world, own, self.global_path, enemy,
                                          intent.opponent_clearance,
                                          safety_margin=self.local_safety_margin)
                local = route_curve_guidance(
                    self.world, own, self.global_path, straight, enemy,
                    intent.opponent_clearance,
                    diagnostics=self.curve_diagnostics,
                    max_distance=2.5)
                if not local:
                    local = curved_guidance(self.world, own, straight, enemy,
                                            intent.opponent_clearance,
                                            diagnostics=self.curve_diagnostics)
                entry_error = path_heading_error(own, local) if local else None
                if entry_error is not None and entry_error > 0.55:
                    rollout = local_rollout(
                        self.world, own, self.global_path, enemy_future,
                        intent.opponent_clearance, intent.opponent_cost_weight,
                        max_speed=min(
                            float(intent.max_speed),
                            float(self.get_parameter("speed_limit_max").value)),
                        horizon=2.4, dt=0.2,
                        opponent_velocity=enemy_velocity,
                        previous_omega=self.last_rollout_omega,
                        safety_margin=self.local_safety_margin + 0.06)
                    if (len(rollout) >= 3 and
                            hypot(rollout[-1].x - own.x,
                                  rollout[-1].y - own.y) >= 0.15):
                        local = rollout
                        self.last_rollout_omega = angle_error(
                            rollout[-1].yaw - rollout[-2].yaw, 0.0) / 0.2
                        self.curve_diagnostics["kinematic_rollout"] = (
                            self.curve_diagnostics.get("kinematic_rollout", 0) + 1)
            if now >= self.curve_diagnostics_until:
                self.get_logger().info(
                    f"Local curve decisions (10 sim s): {self.curve_diagnostics}")
                self.curve_diagnostics.clear()
                self.curve_diagnostics_until = now + 10.0
            self.local_path = local
            self.local_target = target
            self.local_behavior = intent.behavior
        if not local:
            self.local_path = []
            self.global_path = []
            step = recovery_step(self.world, own, enemy,
                                 intent.opponent_clearance,
                                 safety_margin=self.local_safety_margin)
            if step:
                self.recovery_goal = Pose2(own.x + step[1] * cos(step[0]),
                                           own.y + step[1] * sin(step[0]))
                self.recovery_origin = own
                local = local_guidance(self.world, own, [self.recovery_goal],
                                       enemy, intent.opponent_clearance,
                                       min_step=0.04,
                                       safety_margin=self.local_safety_margin)
                if not local:
                    local = [own, Pose2(own.x, own.y, step[0])]
                self.publish_speed_limit(local, intent, now)
                self.local_pub.publish(make_path(self, local))
                self.status_pub.publish(String(data="RECOVERY_ESCAPE"))
                return
            self.speed_limit_value = 0.0
            self.speed_limit_stamp = now
            self.speed_limit_pub.publish(Float32(data=0.0))
            for publisher in (self.speed_clearance_pub,
                              self.speed_curvature_pub,
                              self.speed_alignment_pub):
                publisher.publish(Float32(data=0.0))
            self.local_pub.publish(make_path(self, []))
            self.status_pub.publish(String(data="NO_LOCAL_PATH"))
            return
        self.publish_speed_limit(local, intent, now)
        self.global_pub.publish(make_path(self, self.global_path))
        self.local_pub.publish(make_path(self, local))
        self.status_pub.publish(String(data="OK"))


def main():
    rclpy.init()
    node = TrajectoryPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
