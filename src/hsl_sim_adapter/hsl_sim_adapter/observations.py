"""Gazebo Classic ground truth and lidar converted to navigation inputs."""

from collections import OrderedDict
from copy import deepcopy
from math import hypot

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from .cloud import make_cloud, read_xyz, transform
from .visibility import StaticGrid, opponent_visible


class SimObservations(Node):
    def __init__(self):
        super().__init__("sim_observations")
        self.declare_parameter("own_spawn_x", -0.34)
        self.declare_parameter("own_spawn_y", -0.18)
        self.declare_parameter("own_odom_topic", "/odom")
        self.declare_parameter("opponent_odom_topic", "/opponent/odom")
        self.declare_parameter("own_truth_topic", "/localization/pose")
        self.declare_parameter("opponent_truth_topic", "/opponent/localization/pose")
        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("self_filter_radius", 0.25)
        self.spawn = (self.get_parameter("own_spawn_x").value,
                      self.get_parameter("own_spawn_y").value)
        self.self_filter_radius = self.get_parameter("self_filter_radius").value
        self.own_odom = None
        self.opponent_odom = None
        self.own_truth = None
        self.opponent_truth = None
        self.grid = None
        self.visible = False
        self.last_seen = float("-inf")
        self.voxels = OrderedDict()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, self.get_parameter("own_odom_topic").value,
                                 lambda msg: setattr(self, "own_odom", msg), 10)
        self.create_subscription(Odometry, self.get_parameter("opponent_odom_topic").value,
                                 lambda msg: setattr(self, "opponent_odom", msg), 10)
        self.create_subscription(Odometry, self.get_parameter("own_truth_topic").value,
                                 lambda msg: setattr(self, "own_truth", msg), 10)
        self.create_subscription(Odometry, self.get_parameter("opponent_truth_topic").value,
                                 lambda msg: setattr(self, "opponent_truth", msg), 10)
        self.create_subscription(PointCloud2, self.get_parameter("lidar_topic").value,
                                 self.on_lidar, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_known_map, map_qos)
        self.own_pub = self.create_publisher(Odometry, "navigation/self", 10)
        self.opponent_pub = self.create_publisher(Odometry, "navigation/opponent", 10)
        self.visible_pub = self.create_publisher(Bool, "navigation/opponent_visible", 10)
        self.scan_pub = self.create_publisher(PointCloud2, "navigation/scan",
                                              qos_profile_sensor_data)
        self.map_pub = self.create_publisher(PointCloud2, "navigation/map_points",
                                             qos_profile_sensor_data)
        self.known_map_pub = self.create_publisher(OccupancyGrid, "navigation/known_grid", map_qos)
        self.create_timer(0.05, self.publish_poses)
        self.create_timer(0.5, self.publish_map)

    def on_known_map(self, msg):
        if (msg.header.frame_id == "map" and msg.info.resolution > 0
                and msg.info.width > 0 and msg.info.height > 0):
            self.grid = StaticGrid(msg.info.resolution, msg.info.width,
                                   msg.info.height, msg.info.origin.position.x,
                                   msg.info.origin.position.y, msg.data)
            self.known_map_pub.publish(msg)

    def as_map_odom(self, truth, odom, child_frame):
        if truth is None:
            return None
        result = Odometry()
        result.header = deepcopy(truth.header)
        result.header.frame_id = "map"
        result.child_frame_id = child_frame
        result.pose = deepcopy(truth.pose)
        result.pose.pose.position.x -= self.spawn[0]
        result.pose.pose.position.y -= self.spawn[1]
        result.twist = deepcopy(odom.twist if odom else truth.twist)
        return result

    def opponent_xy(self):
        if self.opponent_truth:
            p = self.opponent_truth.pose.pose.position
            return p.x - self.spawn[0], p.y - self.spawn[1]
        return float("inf"), float("inf")

    def publish_poses(self):
        own = self.as_map_odom(self.own_truth, self.own_odom, "base_footprint")
        if own:
            self.own_pub.publish(own)
        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - self.last_seen
        self.visible_pub.publish(Bool(data=self.visible and 0 <= age <= 0.3))

    def on_lidar(self, msg):
        if self.own_truth is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform("map", msg.header.frame_id, Time())
        except Exception:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        own = self.own_truth.pose.pose.position
        sx, sy = own.x - self.spawn[0], own.y - self.spawn[1]
        points = []
        for point in read_xyz(msg):
            mapped = transform(point, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            # The Classic ray sensor returns points from Kobuki's own plates.
            # Remove only the measured body radius before scan/map publication.
            if hypot(mapped[0] - sx, mapped[1] - sy) >= self.self_filter_radius:
                points.append(mapped)
        header = deepcopy(msg.header)
        header.frame_id = "map"
        self.scan_pub.publish(make_cloud(header, points))
        ex, ey = self.opponent_xy()
        self.visible = opponent_visible(self.grid, (sx, sy), (ex, ey), points)
        if self.visible:
            enemy = self.as_map_odom(self.opponent_truth, self.opponent_odom,
                                     "opponent/base_footprint")
            self.opponent_pub.publish(enemy)
            self.last_seen = self.get_clock().now().nanoseconds * 1e-9
        for point in points:
            if (self.grid and 0.08 <= point[2] <= 0.60
                    and self.grid.matches_static(point[0], point[1])):
                key = tuple(round(value / 0.15) for value in point)
                self.voxels[key] = point
        while len(self.voxels) > 50000:
            self.voxels.popitem(last=False)

    def publish_map(self):
        from std_msgs.msg import Header

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self.map_pub.publish(make_cloud(header, self.voxels.values()))


def main():
    rclpy.init()
    node = SimObservations()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
