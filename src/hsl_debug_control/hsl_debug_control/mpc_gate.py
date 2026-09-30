"""Sole cmd_vel publisher for the decision planner and the external MPC."""

from math import atan2, hypot
from time import perf_counter

import rclpy
from geometry_msgs.msg import Twist
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String

from .core import (DIRECT_MPPI_STATUSES, angle_error, path_turning_decision,
                   safe_mpc_command, select_control_command)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MpcGate(Node):
    def __init__(self):
        super().__init__("hsl_mpc_gate")
        self.pose_stamp = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.scan_stamp = 0.0
        self.path = None
        self.intent = None
        self.command = None
        self.mppi_command = None
        self.planner_status = None
        self.declare_parameter("control_mode", "mppi")
        self.control_mode = self.get_parameter("control_mode").value
        if self.control_mode not in ("mpc", "mppi"):
            raise ValueError("control_mode must be mpc or mppi")
        self.turning_to_path = False
        self.create_subscription(Odometry, "navigation/self", self.on_pose, 10)
        self.create_subscription(PointCloud2, "navigation/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Path, "navigation/local_path", self.on_path, 10)
        self.create_subscription(PlanningIntent, "navigation/intent", self.on_intent, 10)
        self.create_subscription(Twist, "navigation/mpc_cmd_vel", self.on_command, 10)
        self.create_subscription(Twist, "navigation/mppi_cmd_vel", self.on_mppi_command, 10)
        self.create_subscription(String, "navigation/planner_status", self.on_status, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cycle_pub = self.create_publisher(Float32, "navigation/control_cycle_ms", 10)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg):
        self.pose_stamp = seconds(msg.header.stamp)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))

    def on_scan(self, msg):
        self.scan_stamp = seconds(msg.header.stamp)

    def on_path(self, msg):
        self.path = (msg.poses, seconds(msg.header.stamp))

    def on_intent(self, msg):
        self.intent = (msg.behavior, msg.max_speed, seconds(msg.header.stamp))

    def on_command(self, msg):
        self.command = (msg.linear.x, msg.angular.z, self.now())

    def on_mppi_command(self, msg):
        self.mppi_command = (msg.linear.x, msg.angular.z, self.now())

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
        rotation_error = None
        if self.path and len(self.path[0]) >= 2:
            start = self.path[0][0].pose.position
            end_pose = self.path[0][-1].pose
            path_span = hypot(end_pose.position.x - start.x,
                              end_pose.position.y - start.y)
            if path_span < 0.02:
                q = end_pose.orientation
                desired = atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z))
                rotation_error = angle_error(desired, self.yaw)
            elif (self.control_mode != "mppi" or
                  self.planner_status not in DIRECT_MPPI_STATUSES):
                points = [pose.pose.position for pose in self.path[0]]
                (rotation_error, self.turning_to_path, _) = path_turning_decision(
                    [(point.x, point.y) for point in points],
                    (self.x, self.y), self.yaw, self.turning_to_path)
        else:
            self.turning_to_path = False
        command = select_control_command(
            self.control_mode, self.planner_status,
            self.command, self.mppi_command)
        linear, angular = safe_mpc_command(now, self.pose_stamp,
                                           self.scan_stamp, self.path,
                                           self.intent, command,
                                           rotation_error=rotation_error)
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MpcGate()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
