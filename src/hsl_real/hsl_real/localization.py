"""AMCL readiness: uncertainty, current sensors and a live map->odom transform."""
import json
from math import isfinite, sqrt
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def uncertainty_ok(covariance, position_std, yaw_std):
    # Largest eigenvalue of the XY covariance: a rotated uncertainty ellipse
    # must not pass just because its diagonal happens to be smaller.
    a, b, c, yaw = covariance[0], covariance[1], covariance[7], covariance[35]
    if not all(isfinite(v) for v in (a, b, c, yaw)) or min(a, c, yaw) < 0:
        return False
    largest = (a+c+sqrt((a-c)**2+4*b*b))/2
    return bool(largest <= position_std**2 and yaw <= yaw_std**2)


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')
        for name, value in [('odom_topic','/odom'), ('max_position_std',.20),
                            ('max_yaw_std',.35), ('max_pose_age',3.), ('max_sensor_age',1.)]:
            self.declare_parameter(name,value)
        self.pose = self.scan = self.odom = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:setattr(self,'pose',m),10)
        self.create_subscription(LaserScan,'/localization/scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,self.get_parameter('odom_topic').value,lambda m:setattr(self,'odom',m),10)
        self.pub = self.create_publisher(Bool,'/localization/ready',10)
        self.diagnostics = self.create_publisher(String,'/localization/status',10)
        self.client = self.create_client(Empty,'/request_nomotion_update')
        self.pending = None
        self.create_timer(.5,self.request_update)
        self.create_timer(.1,self.tick)

    def request_update(self):
        # AMCL normally updates the pose only after sufficient motion. Obtain
        # fresh estimates while stationary too, without driving to initialize.
        if self.client.service_is_ready() and (self.pending is None or self.pending.done()):
            self.pending = self.client.call_async(Empty.Request())

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        def fresh(msg,limit):
            if msg is None: return False
            age = now-msg.header.stamp.sec-msg.header.stamp.nanosec*1e-9
            return -.5 <= age <= limit
        sensors = all(fresh(m,self.get_parameter('max_sensor_age').value) for m in (self.scan,self.odom))
        pose_fresh = fresh(self.pose,self.get_parameter('max_pose_age').value)
        confidence = self.pose is not None and uncertainty_ok(self.pose.pose.covariance,
            self.get_parameter('max_position_std').value,self.get_parameter('max_yaw_std').value)
        transform = False
        try:
            tf = self.tf.lookup_transform('map','odom',Time())
            stamp = tf.header.stamp
            age = now-stamp.sec-stamp.nanosec*1e-9
            transform = -.6 <= age <= self.get_parameter('max_sensor_age').value
        except Exception:
            pass
        ready = sensors and pose_fresh and confidence and transform
        self.pub.publish(Bool(data=ready))
        self.diagnostics.publish(String(data=json.dumps(dict(ready=ready,sensors_fresh=sensors,
            pose_fresh=pose_fresh,uncertainty_ok=confidence,transform_fresh=transform))))


def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
