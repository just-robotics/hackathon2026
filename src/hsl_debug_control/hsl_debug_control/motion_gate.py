"""Sole final cmd_vel publisher for safety-checked MPPI commands."""

from time import perf_counter

import rclpy
from geometry_msgs.msg import Twist
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, String

from .core import safe_motion_command, select_control_command, match_is_active


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MotionGate(Node):
    def __init__(self):
        super().__init__("hsl_motion_gate")
        self.pose_stamp = 0.0
        self.scan_stamp = 0.0
        self.path = None
        self.intent = None
        self.command = None
        self.planner_status = None
        self.declare_parameter("require_match_active", False)
        self.require_match_active = self.get_parameter("require_match_active").value
        self.match_state = None
        if self.require_match_active:
            qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, "/match/active", self.on_match_active, qos)
        self.create_subscription(Odometry, "navigation/self", self.on_pose, 10)
        self.create_subscription(PointCloud2, "navigation/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Path, "navigation/local_path", self.on_path, 10)
        self.create_subscription(PlanningIntent, "navigation/intent", self.on_intent, 10)
        self.create_subscription(Twist, "navigation/mppi_cmd_vel", self.on_command, 10)
        self.create_subscription(String, "navigation/planner_status", self.on_status, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cycle_pub = self.create_publisher(Float32, "navigation/control_cycle_ms", 10)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_match_active(self, msg):
        self.match_state = (msg.data, self.now())

    def on_pose(self, msg):
        self.pose_stamp = seconds(msg.header.stamp)

    def on_scan(self, msg):
        self.scan_stamp = seconds(msg.header.stamp)

    def on_path(self, msg):
        self.path = (msg.poses, seconds(msg.header.stamp))

    def on_intent(self, msg):
        self.intent = (msg.behavior, msg.max_speed, seconds(msg.header.stamp))

    def on_command(self, msg):
        self.command = (msg.linear.x, msg.angular.z, self.now())

    def on_status(self, msg):
        self.planner_status = msg.data

    def tick(self):
        started = perf_counter()
        try:
            self._tick()
        finally:
            self.cycle_pub.publish(Float32(data=(perf_counter() - started) * 1000))

    def _tick(self):
        now = self.now()
        if self.require_match_active and not match_is_active(now, self.match_state):
            self.pub.publish(Twist())
            return
        command = select_control_command(self.planner_status, self.command)
        linear, angular = safe_motion_command(
            now, self.pose_stamp, self.scan_stamp, self.path, self.intent, command)
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MotionGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
