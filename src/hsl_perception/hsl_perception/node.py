"""Existing hsl ROS interface for feature/detector's shape detector."""
import os
# Tiny Kalman matrices do not benefit from a BLAS thread pool per robot.
for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[variable] = "1"
from collections import deque
import dataclasses
import json
import math
from time import perf_counter
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from .core import Detector, StaticBackground
from .segmentation import RobotModel
from .tracker import TrackerConfig


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def cloud_xyz(msg):
    """Vectorized XYZ, respecting organized row padding and endianness."""
    fields = {f.name: f for f in msg.fields}
    if not msg.width or not msg.height:
        return np.empty((0, 3))
    if any(n not in fields or fields[n].datatype != PointField.FLOAT32 for n in ('x','y','z')):
        raise ValueError('navigation/scan must contain FLOAT32 XYZ')
    dtype = np.dtype({'names': ['x','y','z'], 'formats': [('>' if msg.is_bigendian else '<')+'f4']*3,
                      'offsets': [fields[n].offset for n in ('x','y','z')], 'itemsize': msg.point_step})
    data = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step))
    return np.stack([data[n].ravel() for n in ('x','y','z')], axis=1)


class OpponentDetector(Node):
    def __init__(self):
        super().__init__('opponent_detector')
        max_height = self.declare_parameter('opponent_max_height', .46).value
        if not math.isfinite(max_height) or not .08 <= max_height <= .60:
            raise ValueError('opponent_max_height must be in [0.08,0.60]')
        def parameters(cls, prefix):
            return cls(**{f.name: self.declare_parameter(prefix+'.'+f.name, f.default).value
                          for f in dataclasses.fields(cls)})
        model = parameters(RobotModel, 'robot')
        model.max_height = max_height
        self.detector = Detector(model, parameters(TrackerConfig, 'tracker'),
            strong_arc_min_span_deg=self.declare_parameter('strong_arc_min_span_deg', 0.).value,
            allow_merged_strong=self.declare_parameter('allow_merged_strong', True).value,
            strong_min_inlier_fraction=self.declare_parameter('strong_min_inlier_fraction', 0.).value)
        self.static = None
        self.own = None
        self.visible_stamp = -math.inf
        self.last_cloud = -math.inf
        self.pending = deque(maxlen=8)
        prefix = self.get_namespace().strip('/')
        self.sensor_frame = self.declare_parameter('sensor_frame',
            (prefix+'/' if prefix else '')+'livox_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, 'navigation/known_grid', self.on_grid,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, 'navigation/self', self.on_own, 10)
        self.create_subscription(PointCloud2, 'navigation/scan', self.on_scan, qos_profile_sensor_data)
        self.opponent_pub = self.create_publisher(Odometry, 'navigation/opponent', 10)
        self.visible_pub = self.create_publisher(Bool, 'navigation/opponent_visible', 10)
        self.timing_pub = self.create_publisher(Float32, 'navigation/detector_cycle_ms', 10)
        self.diagnostics_pub = self.create_publisher(String, 'navigation/detector_diagnostics', 10)
        self.create_timer(.05, self.publish_visible)
        self.create_timer(.02, self.flush)

    def on_grid(self, msg):
        if msg.header.frame_id == 'map':
            info = msg.info
            self.static = StaticBackground(info.resolution,
                [info.origin.position.x, info.origin.position.y], info.width, info.height, msg.data)

    def on_own(self, msg):
        if msg.header.frame_id == 'map':
            self.own = msg

    def publish_visible(self):
        age = self.get_clock().now().nanoseconds*1e-9-self.visible_stamp
        self.visible_pub.publish(Bool(data=0 <= age <= .3))

    def on_scan(self, msg):
        if self.pending and seconds(msg.header.stamp) < seconds(self.pending[-1].header.stamp):
            self.pending.clear()
            self.visible_stamp = -math.inf
        self.pending.append(msg)

    def flush(self):
        if not self.pending:
            return
        msg = self.pending[0]
        age = self.get_clock().now().nanoseconds*1e-9-seconds(msg.header.stamp)
        if age > .5 or age < 0:
            self.pending.popleft()
            return
        if self.static is None or self.own is None:
            return
        if not self.tf_buffer.can_transform('map', self.sensor_frame, Time.from_msg(msg.header.stamp)):
            return
        self.pending.popleft()
        self.process_scan(msg)

    def process_scan(self, msg):
        stamp = seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds*1e-9
        if stamp < self.last_cloud:
            self.visible_stamp = -math.inf
        if (self.static is None or self.own is None or msg.header.frame_id != 'map'
                or not 0 <= now-stamp <= .5
                or abs(stamp-seconds(self.own.header.stamp)) > .3):
            return
        try:
            transform = self.tf_buffer.lookup_transform('map', self.sensor_frame, Time.from_msg(msg.header.stamp))
        except TransformException:
            return  # Never replace scan-time geometry with latest TF.
        sensor = transform.transform.translation
        started = perf_counter()
        try:
            points = cloud_xyz(msg)
        except (ValueError, TypeError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.)
            return
        track, diagnostics = self.detector.step(points, [sensor.x, sensor.y, sensor.z], self.static, stamp)
        self.last_cloud = stamp
        detected = track is not None and abs(track.last_update-stamp) < 1e-6
        if detected:
            self.visible_stamp = stamp
        if detected:
            enemy = Odometry()
            enemy.header = msg.header
            enemy.child_frame_id = 'tracked_opponent/base_footprint'
            x, y, yaw, speed, omega = track.state
            enemy.pose.pose.position.x, enemy.pose.pose.position.y = float(x), float(y)
            enemy.pose.pose.orientation.z, enemy.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            # Coasting is explicitly stamped at the last observation; it must not
            # keep a lost target fresh in decision/planning indefinitely.
            t = track.last_update
            enemy.header.stamp.sec = int(t)
            enemy.header.stamp.nanosec = int((t-int(t))*1e9)
            vx, vy = track.mean[2:]
            enemy.twist.twist.linear.x = float(math.cos(yaw)*vx+math.sin(yaw)*vy)
            enemy.twist.twist.linear.y = float(-math.sin(yaw)*vx+math.cos(yaw)*vy)
            enemy.twist.twist.angular.z = float(omega)
            enemy.pose.covariance[0] = float(track.covariance[0,0])
            enemy.pose.covariance[7] = float(track.covariance[1,1])
            enemy.pose.covariance[35] = float(track.cov[2,2])
            self.opponent_pub.publish(enemy)
        elapsed = (perf_counter()-started)*1000
        diagnostics.update(stamp_s=stamp, cycle_ms=elapsed, detected=detected,
                           coasting=track is not None and not detected,
                           track_xy=track.mean[:2].tolist() if track else None)
        self.diagnostics_pub.publish(String(data=json.dumps(diagnostics, ensure_ascii=False)))
        self.timing_pub.publish(Float32(data=elapsed))


def main():
    rclpy.init()
    node = OpponentDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
