"""Real odometry/cloud transport. No Gazebo truth or simulator clocks."""
from copy import deepcopy
from math import hypot
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, Bool
from tf2_ros import Buffer, TransformListener
from hsl_sim_adapter.cloud import read_xyz, make_cloud, transform


class RealObservations(Node):
    def __init__(self):
        super().__init__('real_observations')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('require_localization', False)
        self.localized = not self.get_parameter('require_localization').value
        self.localized_at = None
        self.create_subscription(Bool, '/localization/ready', self.on_localized, 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.own = None
        self.pending_cloud = None
        self.pose_pub = self.create_publisher(Odometry, 'navigation/self', 10)
        self.scan_pub = self.create_publisher(PointCloud2, 'navigation/scan', qos_profile_sensor_data)
        self.points_pub = self.create_publisher(PointCloud2, 'navigation/map_points', qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'navigation/known_grid', qos)
        self.create_subscription(OccupancyGrid, '/map', self.grid_pub.publish, qos)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(PointCloud2, self.get_parameter('lidar_topic').value,
                                 lambda msg: setattr(self, "pending_cloud", msg), qos_profile_sensor_data)
        self.create_timer(.02, self.process_cloud)
        # Empty accumulated cloud: current scan drives obstacle projection.
        # Do not turn moving rivals into a permanent static map.
        self.create_timer(.5, lambda: self.points_pub.publish(make_cloud(
            Header(frame_id='map', stamp=self.get_clock().now().to_msg()), [])))

    def on_localized(self, msg):
        self.localized = msg.data
        self.localized_at = self.get_clock().now()
        if not msg.data:
            self.own = None

    def localization_ready(self):
        if not self.get_parameter('require_localization').value:
            return True
        return self.localized and self.localized_at is not None and (
            self.get_clock().now()-self.localized_at).nanoseconds < 1_000_000_000

    def on_odom(self, msg):
        if not self.localization_ready():
            return
        try:
            tf = self.tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:
            return
        from math import atan2, sin, cos
        q, t = tf.transform.rotation, tf.transform.translation
        own_q = msg.pose.pose.orientation
        yaw = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        own_yaw = atan2(2*(own_q.w*own_q.z+own_q.x*own_q.y),
                        1-2*(own_q.y*own_q.y+own_q.z*own_q.z))
        result = deepcopy(msg)
        p = result.pose.pose.position
        p.x, p.y, p.z = transform((p.x,p.y,p.z), (t.x,t.y,t.z), (q.x,q.y,q.z,q.w))
        result.header.frame_id = 'map'
        result.pose.pose.orientation.x = result.pose.pose.orientation.y = 0.
        result.pose.pose.orientation.z = sin((yaw+own_yaw)/2)
        result.pose.pose.orientation.w = cos((yaw+own_yaw)/2)
        self.own = result
        self.pose_pub.publish(result)

    def process_cloud(self):
        msg = self.pending_cloud
        if msg is None or self.own is None or not self.localization_ready():
            return
        try:
            tf = self.tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:
            return
        self.pending_cloud = None
        t,q = tf.transform.translation,tf.transform.rotation
        p = self.own.pose.pose.position
        points = [transform(x, (t.x,t.y,t.z), (q.x,q.y,q.z,q.w)) for x in read_xyz(msg)]
        points = [x for x in points if hypot(x[0]-p.x,x[1]-p.y) >= .25]
        header = deepcopy(msg.header)
        header.frame_id = 'map'
        self.scan_pub.publish(make_cloud(header,points))


def main():
    rclpy.init()
    node = RealObservations()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
