"""Real odometry/cloud transport. No Gazebo truth or simulator clocks."""
from collections import deque
from copy import deepcopy
import json
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, Bool, String
from tf2_ros import Buffer, TransformListener
from hsl_sim_adapter.cloud import make_cloud, transform
from hsl_perception.cloud import cloud_xyz
from .cloud_geometry import map_xyz


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
        self.pending_clouds = deque(maxlen=8)
        self.pending_odom = deque(maxlen=50)
        self.pose_pub = self.create_publisher(Odometry, 'navigation/self', 10)
        self.scan_pub = self.create_publisher(PointCloud2, 'navigation/scan', qos_profile_sensor_data)
        self.points_pub = self.create_publisher(PointCloud2, 'navigation/map_points', qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'navigation/known_grid', qos)
        self.create_subscription(OccupancyGrid, '/map', self.grid_pub.publish, qos)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(PointCloud2, self.get_parameter('lidar_topic').value,
                                 self.pending_clouds.append, qos_profile_sensor_data)
        self.create_timer(.02, self.process_observations)
        self.observation_diag = self.create_publisher(String, "navigation/observation_diagnostics", 10)
        self.counts = {"poses":0,"scans":0,"expired_poses":0,"expired_scans":0,"superseded_scans":0}
        self.create_timer(.5, lambda: self.observation_diag.publish(String(data=json.dumps(dict(
            self.counts, pending_poses=len(self.pending_odom), pending_scans=len(self.pending_clouds))))))
        # Empty accumulated cloud: current scan drives obstacle projection.
        # Do not turn moving rivals into a permanent static map.
        self.create_timer(.5, lambda: self.points_pub.publish(make_cloud(
            Header(frame_id='map', stamp=self.get_clock().now().to_msg()), [])))

    def on_localized(self, msg):
        self.localized = msg.data
        self.localized_at = self.get_clock().now()
        if not msg.data:
            self.own = None
            self.pending_clouds.clear()
            self.pending_odom.clear()

    def localization_ready(self):
        if not self.get_parameter('require_localization').value:
            return True
        return self.localized and self.localized_at is not None and (
            self.get_clock().now()-self.localized_at).nanoseconds < 1_000_000_000

    def on_odom(self, msg):
        self.pending_odom.append(msg)

    def publish_odom(self, msg):
        try:
            tf = self.tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:
            return False
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
        self.counts['poses'] += 1
        return True

    def process_observations(self):
        if not self.localization_ready():
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        def age(msg):
            return now-msg.header.stamp.sec-msg.header.stamp.nanosec*1e-9
        for queue,key in [(self.pending_odom,'expired_poses'),(self.pending_clouds,'expired_scans')]:
            while queue and age(queue[0])>.8:
                queue.popleft();self.counts[key]+=1
        # Publish the newest pose with a transform on its own stamp, retaining
        # later messages until their TF arrives. Never restamp old observations.
        for index in range(len(self.pending_odom)-1,-1,-1):
            if self.publish_odom(self.pending_odom[index]):
                for _ in range(index+1):self.pending_odom.popleft()
                break
        self.process_cloud()

    def process_cloud(self):
        if not self.pending_clouds or self.own is None:
            return
        # Prefer the newest transformable measurement. Processing an old
        # backlog delays every downstream consumer while a newer scan exists.
        for index in range(len(self.pending_clouds)-1,-1,-1):
            msg = self.pending_clouds[index]
            try:
                tf = self.tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
                base = self.tf.lookup_transform('map', self.own.child_frame_id, Time.from_msg(msg.header.stamp))
            except Exception:
                continue
            break
        else:
            return
        self.counts['superseded_scans'] += index
        for _ in range(index+1):self.pending_clouds.popleft()
        t,q = tf.transform.translation,tf.transform.rotation
        p = base.transform.translation
        # Uniform raw-cloud decimation discarded sparse real box faces before
        # any classifier saw them. Transform every measured point in one array;
        # keep the exact scan-time base pose for the unchanged body mask.
        points = map_xyz(cloud_xyz(msg), (t.x,t.y,t.z),
                         (q.x,q.y,q.z,q.w), (p.x,p.y))
        header = deepcopy(msg.header)
        header.frame_id = 'map'
        self.scan_pub.publish(make_cloud(header,points))
        self.counts['scans'] += 1


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
