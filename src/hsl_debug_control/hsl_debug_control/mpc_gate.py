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
from std_msgs.msg import Float32

from .core import angle_error, safe_mpc_command


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
        self.turning_to_path = False
        self.create_subscription(Odometry, "navigation/self", self.on_pose, 10)
        self.create_subscription(PointCloud2, "navigation/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Path, "navigation/local_path", self.on_path, 10)
        self.create_subscription(PlanningIntent, "navigation/intent", self.on_intent, 10)
        self.create_subscription(Twist, "navigation/mpc_cmd_vel", self.on_command, 10)
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

    def tick(self):
        started = perf_counter()
        try:
            self._tick()
        finally:
            self.cycle_pub.publish(Float32(data=(perf_counter() - started) * 1000))

    def _tick(self):
        rotation_error = None
        endpoint_distance = None
        if self.path and len(self.path[0]) >= 2:
            start = self.path[0][0].pose.position
            end_pose = self.path[0][-1].pose
            path_span = hypot(end_pose.position.x - start.x,
                              end_pose.position.y - start.y)
            endpoint_distance = hypot(end_pose.position.x - self.x,
                                      end_pose.position.y - self.y)
            if path_span < 0.02:
                q = end_pose.orientation
                desired = atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z))
                rotation_error = angle_error(desired, self.yaw)
            else:
                points = [pose.pose.position for pose in self.path[0]]
                chord_x = end_pose.position.x - start.x
                chord_y = end_pose.position.y - start.y
                deviation = max(abs((point.x - start.x) * chord_y -
                                    (point.y - start.y) * chord_x) / path_span
                                for point in points)
                if deviation > 0.035:
                    nearest = min(range(len(points)),
                                  key=lambda i: hypot(points[i].x - self.x,
                                                      points[i].y - self.y))
                    before = points[max(0, nearest - 2)]
                    after = points[min(len(points) - 1, nearest + 3)]
                    desired = atan2(after.y - before.y, after.x - before.x)
                else:
                    desired = atan2(chord_y, chord_x)
                error = angle_error(desired, self.yaw)
                if abs(error) > (0.35 if self.turning_to_path else 0.65):
                    self.turning_to_path = True
                    rotation_error = error
                else:
                    self.turning_to_path = False
        else:
            self.turning_to_path = False
        linear, angular = safe_mpc_command(self.now(), self.pose_stamp,
                                           self.scan_stamp, self.path,
                                           self.intent, self.command,
                                           rotation_error=rotation_error)
        if endpoint_distance is not None and rotation_error is None:
            linear = min(linear, max(0.08, 0.8 * endpoint_distance), 0.3)
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
