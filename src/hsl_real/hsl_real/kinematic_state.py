"""Map-corrected FAST-LIO state. AMCL owns map->lio_odom; no TF is published here."""
from collections import deque
from copy import deepcopy
from math import atan2, cos, sin
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformException
from .localization import uncertainty_ok


def corrected_state(source, transform, map_covariance):
    """Planar map correction; twist stays in the unchanged base child frame."""
    q = transform.transform.rotation
    t = transform.transform.translation
    yaw = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = cos(yaw), sin(yaw)
    out = deepcopy(source)
    p = out.pose.pose.position
    x, y = p.x, p.y
    p.x, p.y, p.z = c*x-s*y+t.x, s*x+c*y+t.y, p.z+t.z
    own = source.pose.pose.orientation
    heading = atan2(2*(own.w*own.z+own.x*own.y), 1-2*(own.y*own.y+own.z*own.z))
    out.pose.pose.orientation.x = out.pose.pose.orientation.y = 0.
    out.pose.pose.orientation.z = sin((heading+yaw)/2)
    out.pose.pose.orientation.w = cos((heading+yaw)/2)
    out.header.frame_id = 'map'
    # AMCL uncertainty in map plus a rotated local covariance floor. This is
    # an approximation, not an independent-fusion accuracy guarantee.
    rotation = np.eye(6)
    rotation[:2,:2] = [[c,-s],[s,c]]
    local = np.asarray(source.pose.covariance).reshape(6,6)
    out.pose.covariance = (np.asarray(map_covariance).reshape(6,6) +
                           rotation @ local @ rotation.T).reshape(-1).tolist()
    return out


class MapKinematicState(Node):
    def __init__(self):
        super().__init__('map_kinematic_state')
        for name, value in [('input_topic','/localization/lio_odometry'),
                            ('output_topic','/localization/kinematic_state'),
                            ('max_pose_age',3.), ('max_input_age',.8),
                            ('max_position_std',.20), ('max_yaw_std',.35)]:
            self.declare_parameter(name,value)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.amcl = None
        self.pending = deque(maxlen=50)
        self.last_stamp = None
        self.pub = self.create_publisher(Odometry,self.get_parameter('output_topic').value,10)
        self.create_subscription(Odometry,self.get_parameter('input_topic').value,self.pending.append,10)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_amcl,10)
        self.create_timer(.02,self.process)

    def on_amcl(self, msg):
        self.amcl = msg

    def process(self):
        now = self.get_clock().now().nanoseconds*1e-9
        def seconds(msg):
            return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.amcl is None or self.amcl.header.frame_id != 'map':
            self.pending.clear()
            return
        age = now-seconds(self.amcl)
        if not 0 <= age <= self.get_parameter('max_pose_age').value or not uncertainty_ok(
                self.amcl.pose.covariance,self.get_parameter('max_position_std').value,
                self.get_parameter('max_yaw_std').value):
            self.pending.clear()
            return
        while self.pending:
            msg = self.pending[0]
            stamp = seconds(msg)
            if stamp > now:
                return
            if now-stamp > self.get_parameter('max_input_age').value or (
                    self.last_stamp is not None and stamp <= self.last_stamp):
                self.pending.popleft()
                continue
            if msg.header.frame_id != 'lio_odom' or msg.child_frame_id != 'base_footprint':
                self.pending.popleft()
                self.get_logger().error('LIO input must be lio_odom -> base_footprint')
                continue
            try:
                tf = self.tf.lookup_transform('map','lio_odom',Time.from_msg(msg.header.stamp))
            except TransformException:
                return  # Retry on a later timer tick, never substitute latest TF.
            self.pub.publish(corrected_state(msg,tf,self.amcl.pose.covariance))
            self.last_stamp = stamp
            self.pending.popleft()


def main():
    rclpy.init()
    node = MapKinematicState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
