"""ROS transport and bounded refresh for the pure planner."""

import json
import struct
import random
import numpy as np
from math import atan2, cos, hypot, isfinite, pi, sin
from time import perf_counter

import rclpy
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, String

from .mppi import mppi_local_guidance
from .core import (Pose2, VoxelWorld, astar, capture_goal, coverage_target,
                   reachable_intercept, navigation_obstacles, evade_target,
                   local_guidance, path_heading_error, reachable_target,
                   recovery_step, checked_recovery_target, turn_alignment_is_progress, safe_segment,
                   reusable_route,
                   smooth_control_route,
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
        self.declare_parameter("mppi_batch_size", 192)
        self.declare_parameter("mppi_iterations", 2)
        self.declare_parameter("mppi_horizon", 3.0)
        self.declare_parameter("mppi_model_dt", 0.15)
        self.declare_parameter("mppi_temperature", 0.3)
        self.declare_parameter("control_mode", "mppi")
        self.declare_parameter("local_backend", "python")
        self.declare_parameter("mpc_path_source", "local")
        self.declare_parameter("role", "explorer")
        self.frame = self.get_parameter("planning_frame").value
        self.role = self.get_parameter("role").value
        self.control_mode = self.get_parameter("control_mode").value
        self.local_backend = self.get_parameter("local_backend").value
        if self.local_backend not in ("python", "nav2_cpp"):
            raise ValueError("local_backend must be python or nav2_cpp")
        if self.local_backend == "nav2_cpp" and self.control_mode != "mppi":
            raise ValueError("nav2_cpp requires control_mode=mppi")
        self.mpc_path_source = self.get_parameter("mpc_path_source").value
        if self.control_mode not in ("mpc", "mppi"):
            raise ValueError("control_mode must be mpc or mppi")
        if self.mpc_path_source not in ("local", "global", "smoothed"):
            raise ValueError("mpc_path_source must be local, global or smoothed")
        if self.role not in ("explorer", "guardian"):
            raise ValueError("role must be explorer or guardian")
        self.local_safety_margin = 0.14 if self.role == "explorer" else 0.12
        self.pose_timeout = self.get_parameter("pose_timeout").value
        self.scan_timeout = self.get_parameter("scan_timeout").value
        self.intent_timeout = self.get_parameter("intent_timeout").value
        self.random_seed = int(self.get_parameter("random_seed").value)
        self.mppi_config = {
            "batch_size": int(self.get_parameter("mppi_batch_size").value),
            "iterations": int(self.get_parameter("mppi_iterations").value),
            "horizon": float(self.get_parameter("mppi_horizon").value),
            "dt": float(self.get_parameter("mppi_model_dt").value),
            "temperature": float(self.get_parameter("mppi_temperature").value),
        }
        if self.control_mode == "mppi":
            # Nav2 uses a model step no shorter than the command period.
            # Match this node's 0.2 s timer and stock actuator bounds.
            self.mppi_config.update(dt=0.2, linear_accel=0.5,
                                    angular_accel=2.0)
        self.mppi_max_speed = 0.5 if self.control_mode == "mppi" else None
        self.rng = random.Random(self.random_seed)
        self.mppi_rng = np.random.default_rng(self.random_seed)
        self.world = VoxelWorld(self.get_parameter("resolution").value,
                                self.get_parameter("robot_radius").value)
        self.own = None
        self.measured_speed = 0.0
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
        self.dirty = True
        self.global_path = []
        self.global_target = None
        self.pursuit_target = None
        self.route_behavior = None
        self.local_path = []
        self.local_target = None
        self.local_behavior = None
        self.mppi_controls = None
        self.direct_controls = None
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
        self.mppi_diagnostics = {}
        self.mppi_diagnostics_until = 0.0
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
        self.local_pub = (self.create_publisher(Path, "navigation/local_path", 10)
                          if self.local_backend == "python" else None)
        self.native_reference_pub = None
        if self.local_backend == "nav2_cpp":
            self.native_reference_pub = self.create_publisher(
                Path, "navigation/nav2_reference", 10)
            self.create_subscription(Path, "navigation/local_path",
                                     self.on_native_path, 10)
        self.mpc_path_pub = self.create_publisher(Path, "navigation/mpc_path", 10)
        self.direct_cmd_pub = (self.create_publisher(Twist, "navigation/mppi_cmd_vel", 10)
                               if self.local_backend == "python" else None)
        self.mppi_diag_pub = (self.create_publisher(String, "navigation/mppi_diagnostics", 10)
                              if self.local_backend == "python" else None)
        self.status_pub = self.create_publisher(
            String, "navigation/global_status" if self.local_backend == "nav2_cpp"
            else "navigation/planner_status", 10)
        self.cycle_pub = self.create_publisher(Float32, "navigation/planner_cycle_ms", 10)
        self.create_timer(0.2, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_own(self, msg):
        if msg.header.frame_id == self.frame:
            self.own = (odom_pose(msg), seconds(msg.header.stamp))
            self.measured_speed = abs(float(msg.twist.twist.linear.x))
            self.measured_omega = float(msg.twist.twist.angular.z)

    def on_opponent(self, msg):
        if msg.header.frame_id == self.frame:
            self.opponent = (odom_pose(msg), seconds(msg.header.stamp), msg.twist.twist)

    def on_native_path(self, msg):
        if msg.header.frame_id == self.frame:
            self.local_path = [Pose2(p.pose.position.x, p.pose.position.y,
                                    atan2(2 * (p.pose.orientation.w * p.pose.orientation.z +
                                               p.pose.orientation.x * p.pose.orientation.y),
                                          1 - 2 * (p.pose.orientation.y ** 2 +
                                                   p.pose.orientation.z ** 2)))
                               for p in msg.poses]

    def on_intent(self, msg):
        if msg.header.frame_id == self.frame:
            if self.local_behavior is not None and msg.behavior != self.local_behavior:
                self.mppi_controls = None
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
        self.mppi_controls = None
        self.direct_controls = None
        self.global_pub.publish(make_path(self, []))
        if self.local_pub is not None:
            self.local_pub.publish(make_path(self, []))
        if self.native_reference_pub is not None:
            self.native_reference_pub.publish(make_path(self, []))
        if self.mpc_path_source == "smoothed":
            self.mpc_path_pub.publish(make_path(self, []))
        self.status_pub.publish(String(data=reason))

    def recovery_path(self, own, enemy, enemy_velocity, intent):
        """Try a short checked arc before a straight escape or in-place turn."""
        goal = self.recovery_goal
        if goal is None:
            return []
        config = dict(self.mppi_config, horizon=1.65, batch_size=128)
        arc, controls, diagnostics = mppi_local_guidance(
            self.world, own, [own, goal],
            max_speed=min(float(intent.max_speed), self.mppi_max_speed)
            if self.mppi_max_speed is not None else float(intent.max_speed),
            measured_speed=self.measured_speed,
            measured_omega=self.measured_omega,
            opponent=enemy, opponent_velocity=enemy_velocity,
            opponent_clearance=float(intent.opponent_clearance),
            safety_margin=self.local_safety_margin,
            rng=self.mppi_rng, **config)
        self.publish_mppi_diagnostics(own, enemy, diagnostics, controls,
                                      recovery=True)
        if arc and (hypot(arc[-1].x - own.x, arc[-1].y - own.y) >= 0.08 or
                    abs((arc[-1].yaw - own.yaw + pi) % (2 * pi) - pi) >= 0.3):
            self.direct_controls = controls
            return arc
        line = local_guidance(self.world, own, [goal], enemy,
                              intent.opponent_clearance, min_step=0.04,
                              safety_margin=self.local_safety_margin)
        if line:
            dx, dy = goal.x - own.x, goal.y - own.y
            if dx * cos(own.yaw) + dy * sin(own.yaw) < -0.04:
                return [own, Pose2(own.x, own.y, atan2(dy, dx))]
        return line

    def publish_mppi_diagnostics(self, own, enemy, diagnostics, controls,
                                 recovery=False):
        clearance = self.world.obstacle_clearance(own.x, own.y)
        self.mppi_diag_pub.publish(String(data=json.dumps({
            "result": diagnostics.get("result"),
            "recovery": recovery,
            "recovery_goal": ([self.recovery_goal.x, self.recovery_goal.y]
                              if recovery and self.recovery_goal else None),
            "valid_samples": diagnostics.get("valid_samples"),
            "batch_size": diagnostics.get("batch_size"),
            "selected_progress_m": diagnostics.get("selected_progress"),
            "furthest_progress_m": diagnostics.get("furthest_progress"),
            "path_deviation_m": diagnostics.get("path_deviation"),
            "selected_type": diagnostics.get("selected_type"),
            "clearance_m": clearance if isfinite(clearance) else None,
            "opponent_distance_m": (hypot(own.x - enemy.x, own.y - enemy.y)
                                    if enemy is not None else None),
            "first_speed_mps": controls[0][0] if controls else None,
            "first_omega_radps": controls[0][1] if controls else None,
        })))

    def tick(self):
        started = perf_counter()
        self.direct_controls = None
        try:
            self._tick()
        finally:
            command = Twist()
            if self.control_mode == "mppi" and self.direct_controls:
                command.linear.x = float(self.direct_controls[0][0])
                command.angular.z = float(self.direct_controls[0][1])
            if self.direct_cmd_pub is not None:
                self.direct_cmd_pub.publish(command)
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
            self.recovery_avoid = blocked_ahead or self.global_target
            self.recovery_until = now + 8.0
            self.recovery_attempt += 1
            self.global_path = []
            # First retry the nominal route/warm start. A useful turn in free
            # space must not immediately hand authority to an arbitrary escape.
            self.mppi_controls = None
            self.recovery_goal = None
            pending_native_recovery = self.nominal_retry > 0
            self.nominal_retry += 1
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
            static_points, scan_points = navigation_obstacles(
                self.grid_points, self.map_points, self.scan_points, enemy)
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
        if intent.behavior == 4 and enemy:
            separation = hypot(own.x - enemy.x, own.y - enemy.y)
            if separation < intent.opponent_clearance + 0.3:
                previous = self.evade_waypoint
                if (previous is None or hypot(previous.x - own.x, previous.y - own.y) < 0.15 or
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
        if intent.behavior == 7 and target is not None and self.global_path:
            # Keep the continuous capture pose and its facing constraint. A*
            # raster centres can otherwise stop outside the capture radius.
            if len(self.global_path) == 1:
                self.global_path = [own, target]
            elif safe_segment(self.world, self.global_path[-2], target,
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
        if self.local_backend == "nav2_cpp":
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
            self.global_pub.publish(make_path(self, self.global_path))
            self.native_reference_pub.publish(make_path(self, reference))
            self.status_pub.publish(String(data=status))
            return
        if self.recovery_goal is not None:
            self.mppi_controls = None
            self.local_path = []
            local = self.recovery_path(own, enemy, enemy_velocity, intent)
            self.local_path = local
            if not local:
                self.recovery_goal = None
        elif hypot(own.x - target.x, own.y - target.y) <= intent.target_tolerance:
            self.mppi_controls = None
            self.local_path = []
            local = [own, Pose2(own.x, own.y, target.yaw)]
            if intent.behavior == 7:
                self.local_path = local
                self.local_pub.publish(make_path(self, local))
                self.status_pub.publish(String(data="CAPTURE_ALIGNMENT"))
                return
        else:
            same_target = (self.local_target is not None and
                           hypot(target.x - self.local_target.x,
                                 target.y - self.local_target.y) < 0.3)
            previous_controls = (
                self.mppi_controls if same_target and
                self.local_behavior == intent.behavior else None)
            local, self.mppi_controls, diagnostics = mppi_local_guidance(
                self.world, own, self.global_path,
                max_speed=min(float(intent.max_speed), self.mppi_max_speed)
                if self.mppi_max_speed is not None else float(intent.max_speed),
                measured_speed=self.measured_speed,
                measured_omega=self.measured_omega,
                opponent=enemy,
                opponent_velocity=enemy_velocity,
                opponent_clearance=float(intent.opponent_clearance),
                safety_margin=self.local_safety_margin,
                previous_controls=previous_controls,
                rng=self.mppi_rng,
                **self.mppi_config)
            self.direct_controls = self.mppi_controls if local else None
            self.publish_mppi_diagnostics(own, enemy, diagnostics,
                                          self.mppi_controls)
            reason = diagnostics.get("result", "unknown")
            self.mppi_diagnostics[reason] = (
                self.mppi_diagnostics.get(reason, 0) + 1)
            if now >= self.mppi_diagnostics_until:
                self.get_logger().info(
                    f"Local MPPI decisions (10 sim s): {self.mppi_diagnostics}; "
                    f"last={diagnostics}")
                self.mppi_diagnostics.clear()
                self.mppi_diagnostics_until = now + 10.0
            self.local_path = local
            self.local_target = target
            self.local_behavior = intent.behavior
        if not local:
            self.local_path = []
            self.global_path = []
            self.recovery_goal = checked_recovery_target(
                self.world, own, None, enemy, intent.opponent_clearance,
                self.local_safety_margin)
            if self.recovery_goal is not None:
                self.recovery_origin = own
                local = self.recovery_path(own, enemy, enemy_velocity, intent)
                if not local:
                    local = [own, Pose2(own.x, own.y, self.recovery_goal.yaw)]
                self.local_path = local
                self.local_pub.publish(make_path(self, local))
                if self.mpc_path_source == "smoothed":
                    self.mpc_path_pub.publish(make_path(self, local))
                self.status_pub.publish(String(data="RECOVERY_ESCAPE"))
                return
            self.local_pub.publish(make_path(self, []))
            self.status_pub.publish(String(data="NO_LOCAL_PATH"))
            return
        self.global_pub.publish(make_path(self, self.global_path))
        self.local_pub.publish(make_path(self, local))
        if self.mpc_path_source == "smoothed":
            control_route = (local if self.recovery_goal is not None else
                             smooth_control_route(self.world, self.global_path,
                                                  enemy, intent.opponent_clearance,
                                                  self.local_safety_margin))
            self.mpc_path_pub.publish(make_path(self, control_route))
        status = ("RECOVERY_MPPI" if self.recovery_goal is not None and
                  self.control_mode == "mppi" and self.direct_controls else
                  "OK" if self.control_mode == "mpc" or self.direct_controls
                  else "RECOVERY_FALLBACK")
        self.status_pub.publish(String(data=status))


def main():
    rclpy.init()
    node = TrajectoryPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
