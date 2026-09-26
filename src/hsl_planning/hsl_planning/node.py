"""ROS transport and bounded refresh for the pure planner."""

import struct
from math import atan2, cos, hypot, sin

import rclpy
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from .core import Pose2, VoxelWorld, astar, capture_goal, local_rollout, reachable_target


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
        self.frame = self.get_parameter("planning_frame").value
        self.pose_timeout = self.get_parameter("pose_timeout").value
        self.scan_timeout = self.get_parameter("scan_timeout").value
        self.intent_timeout = self.get_parameter("intent_timeout").value
        self.world = VoxelWorld(self.get_parameter("resolution").value,
                                self.get_parameter("robot_radius").value)
        self.own = None
        self.opponent = None
        self.intent = None
        self.scan_stamp = 0.0
        self.map_points = []
        self.grid_points = []
        self.grid_free = set()
        self.scan_points = []
        self.dirty = True
        self.global_path = []
        self.last_global = -1e9
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
        self.status_pub = self.create_publisher(String, "navigation/planner_status", 10)
        self.create_timer(0.2, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_own(self, msg):
        if msg.header.frame_id == self.frame:
            self.own = (odom_pose(msg), seconds(msg.header.stamp))

    def on_opponent(self, msg):
        if msg.header.frame_id == self.frame:
            self.opponent = (odom_pose(msg), seconds(msg.header.stamp), msg.twist.twist)

    def on_intent(self, msg):
        if msg.header.frame_id == self.frame:
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
        self.global_pub.publish(make_path(self, []))
        self.local_pub.publish(make_path(self, []))
        self.status_pub.publish(String(data=reason))

    def tick(self):
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
                                 if hypot(p[0] - enemy.x, p[1] - enemy.y) > 0.4]
                scan_points = [p for p in scan_points
                               if hypot(p[0] - enemy.x, p[1] - enemy.y) > 0.3]
            self.world.update(static_points, scan_points, own, self.grid_free)
            self.dirty = False
        intent = self.intent
        if intent.behavior in (6, 7) and enemy:
            target = capture_goal(self.world, own, enemy) or Pose2(
                intent.target.position.x, intent.target.position.y)
        elif intent.has_target:
            q = intent.target.orientation
            target = Pose2(intent.target.position.x, intent.target.position.y,
                           atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))
        else:
            target = self.world.frontier(own)
        target = reachable_target(self.world, own, target,
                                  explore=intent.behavior in (3, 5))
        if target is None and intent.behavior in (3, 5):
            target = Pose2(own.x, own.y, own.yaw + 1.2)
        if target is None:
            self.publish_empty("NO_TARGET_OR_FRONTIER")
            return
        if now - self.last_global >= 1.0 or not self.global_path:
            route = astar(self.world, own, target, enemy_future,
                          intent.opponent_clearance, intent.opponent_cost_weight)
            if not route:
                frontier = self.world.frontier(own, target)
                if frontier:
                    route = astar(self.world, own, frontier, enemy_future,
                                  intent.opponent_clearance, intent.opponent_cost_weight)
            self.global_path = route
            self.last_global = now
        if not self.global_path:
            self.publish_empty("NO_GLOBAL_PATH")
            return
        if hypot(own.x - target.x, own.y - target.y) <= intent.target_tolerance:
            local = [own, Pose2(own.x, own.y, target.yaw)]
        else:
            local = local_rollout(self.world, own, self.global_path, enemy,
                                  intent.opponent_clearance, intent.opponent_cost_weight,
                                  max_speed=intent.max_speed,
                                  opponent_velocity=enemy_velocity)
        if not local:
            self.local_pub.publish(make_path(self, []))
            self.status_pub.publish(String(data="NO_LOCAL_PATH"))
            return
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
