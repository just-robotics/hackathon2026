"""Simulation-only conversion into future localization/perception interfaces."""

from collections import OrderedDict
from copy import deepcopy
from math import cos, hypot, sin

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from .cloud import make_cloud, read_xyz, transform


class SimObservations(Node):
    def __init__(self):
        super().__init__("sim_observations")
        self.declare_parameter("own_odom_topic", "/odom")
        self.declare_parameter("opponent_odom_topic", "/opponent/odom")
        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("own_model", "kobuki")
        self.declare_parameter("opponent_model", "opponent")
        self.declare_parameter("opponent_offset", [4.0, -2.0])
        self.declare_parameter("opponent_initial_yaw", 3.14159)
        self.own_model = self.get_parameter("own_model").value
        self.opponent_model = self.get_parameter("opponent_model").value
        self.offset = self.get_parameter("opponent_offset").value
        self.opponent_initial_yaw = self.get_parameter("opponent_initial_yaw").value
        self.own_odom = None
        self.opponent_odom = None
        self.world_poses = {}
        self.voxels = OrderedDict()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, self.get_parameter("own_odom_topic").value,
                                 self.on_own, 10)
        self.create_subscription(Odometry, self.get_parameter("opponent_odom_topic").value,
                                 self.on_opponent, 10)
        self.create_subscription(TFMessage, "/gazebo/world_poses", self.on_world_poses, 10)
        self.create_subscription(PointCloud2, self.get_parameter("lidar_topic").value,
                                 self.on_lidar, qos_profile_sensor_data)
        self.own_pub = self.create_publisher(Odometry, "navigation/self", 10)
        self.opponent_pub = self.create_publisher(Odometry, "navigation/opponent", 10)
        self.scan_pub = self.create_publisher(PointCloud2, "navigation/scan",
                                              qos_profile_sensor_data)
        self.map_pub = self.create_publisher(PointCloud2, "navigation/map_points",
                                             qos_profile_sensor_data)
        self.create_timer(0.05, self.publish_poses)
        self.create_timer(0.5, self.publish_map)

    def on_own(self, msg):
        self.own_odom = msg

    def on_opponent(self, msg):
        self.opponent_odom = msg

    def on_world_poses(self, msg):
        for item in msg.transforms:
            name = item.child_frame_id.strip("/")
            for model in (self.own_model, self.opponent_model):
                if name == model or name.endswith("::" + model) or name.endswith("/" + model):
                    self.world_poses[model] = (item.transform, self.get_clock().now().nanoseconds * 1e-9)

    def model_truth(self, model):
        item = self.world_poses.get(model)
        if item and self.get_clock().now().nanoseconds * 1e-9 - item[1] <= 0.5:
            return item[0]
        return None

    def opponent_xy(self):
        truth = self.model_truth(self.opponent_model)
        if truth:
            return truth.translation.x, truth.translation.y
        if self.opponent_odom:
            odom = self.as_map_odom(self.opponent_odom, self.opponent_model,
                                    self.offset, self.opponent_initial_yaw)
            return odom.pose.pose.position.x, odom.pose.pose.position.y
        return float("inf"), float("inf")

    def as_map_odom(self, original, model, fallback_offset=(0.0, 0.0), fallback_yaw=0.0):
        if original is None:
            return None
        result = Odometry()
        result.header.stamp = original.header.stamp
        result.header.frame_id = "map"
        result.child_frame_id = "base_footprint"
        result.pose = deepcopy(original.pose)
        result.twist = deepcopy(original.twist)
        truth = self.model_truth(model)
        if truth:
            result.pose.pose.position.x = truth.translation.x
            result.pose.pose.position.y = truth.translation.y
            result.pose.pose.position.z = truth.translation.z
            result.pose.pose.orientation = truth.rotation
        else:
            x, y = result.pose.pose.position.x, result.pose.pose.position.y
            result.pose.pose.position.x = fallback_offset[0] + cos(fallback_yaw) * x - sin(fallback_yaw) * y
            result.pose.pose.position.y = fallback_offset[1] + sin(fallback_yaw) * x + cos(fallback_yaw) * y
            q = result.pose.pose.orientation
            half = fallback_yaw / 2
            q.z, q.w = q.z * cos(half) + q.w * sin(half), q.w * cos(half) - q.z * sin(half)
        return result

    def publish_poses(self):
        own = self.as_map_odom(self.own_odom, self.own_model)
        enemy = self.as_map_odom(self.opponent_odom, self.opponent_model,
                                 self.offset, self.opponent_initial_yaw)
        if own:
            self.own_pub.publish(own)
        if enemy:
            self.opponent_pub.publish(enemy)

    def on_lidar(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform("map", msg.header.frame_id, Time())
        except Exception:
            return
        translation = (tf.transform.translation.x, tf.transform.translation.y,
                       tf.transform.translation.z)
        q = tf.transform.rotation
        quaternion = (q.x, q.y, q.z, q.w)
        points = [transform(point, translation, quaternion) for point in read_xyz(msg)]
        header = deepcopy(msg.header)
        header.frame_id = "map"
        self.scan_pub.publish(make_cloud(header, points))
        ex, ey = self.opponent_xy()
        for point in points:
            if hypot(point[0] - ex, point[1] - ey) < 0.45:
                continue
            if 0.08 <= point[2] <= 0.60:
                key = tuple(round(value / 0.15) for value in point)
                self.voxels[key] = point
        while len(self.voxels) > 50000:
            self.voxels.popitem(last=False)

    def publish_map(self):
        from std_msgs.msg import Header

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        ex, ey = self.opponent_xy()
        points = [p for p in self.voxels.values()
                  if hypot(p[0] - ex, p[1] - ey) >= 0.45]
        self.map_pub.publish(make_cloud(header, points))


def main():
    rclpy.init()
    node = SimObservations()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
