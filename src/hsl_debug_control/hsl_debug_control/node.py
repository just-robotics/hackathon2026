from math import atan2

import rclpy
from geometry_msgs.msg import Twist
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from .core import safe_follow


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def xyz_yaw(pose):
    p, q = pose.position, pose.orientation
    return (p.x, p.y, atan2(2 * (q.w * q.z + q.x * q.y),
                            1 - 2 * (q.y * q.y + q.z * q.z)))


class DebugFollower(Node):
    def __init__(self):
        super().__init__("debug_follower")
        self.declare_parameter("pose_timeout", 1.2)
        self.declare_parameter("path_timeout", 1.0)
        self.declare_parameter("intent_timeout", 1.0)
        self.own = None
        self.path = None
        self.intent = None
        self.create_subscription(Odometry, "navigation/self", self.on_own, 10)
        self.create_subscription(Path, "navigation/local_path", self.on_path, 10)
        self.create_subscription(PlanningIntent, "navigation/intent", self.on_intent, 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_own(self, msg):
        self.own = (xyz_yaw(msg.pose.pose), seconds(msg.header.stamp))

    def on_path(self, msg):
        self.path = ([(xyz_yaw(item.pose)) for item in msg.poses],
                     seconds(msg.header.stamp))

    def on_intent(self, msg):
        self.intent = msg

    def tick(self):
        command = Twist()
        now = self.now()
        intent = (self.intent.behavior, self.intent.max_speed,
                  seconds(self.intent.header.stamp)) if self.intent else None
        command.linear.x, command.angular.z = safe_follow(
            now, self.own, self.path, intent,
            pose_timeout=self.get_parameter("pose_timeout").value,
            path_timeout=self.get_parameter("path_timeout").value,
            intent_timeout=self.get_parameter("intent_timeout").value)
        self.cmd_pub.publish(command)


def main():
    rclpy.init()
    node = DebugFollower()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
