"""A repeatable opponent patrolling one clear corridor of maze.world."""

from math import atan2

import rclpy
from geometry_msgs.msg import Twist
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .patrol import patrol_command


class ScriptedOpponent(Node):
    def __init__(self):
        super().__init__("scripted_opponent")
        self.declare_parameter("patrol_end_x", 1.0)
        self.declare_parameter("patrol_end_y", 2.5)
        self.end = (self.get_parameter("patrol_end_x").value,
                    self.get_parameter("patrol_end_y").value)
        self.allowed = False
        self.start = None
        self.target_end = True
        self.pose = None
        self.pub = self.create_publisher(Twist, "/opponent/cmd_vel", 10)
        self.create_subscription(PlanningIntent, "/navigation/intent", self.on_intent, 10)
        self.create_subscription(Odometry, "/opponent/localization/pose", self.on_pose, 10)
        self.create_timer(0.1, self.tick)

    def on_intent(self, msg):
        self.allowed = msg.behavior not in (PlanningIntent.WAIT, PlanningIntent.STOP)

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = atan2(2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pose = (p.x, p.y, yaw, stamp)
        if self.start is None:
            self.start = (p.x, p.y)

    def tick(self):
        command = Twist()
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.allowed and self.pose and now - self.pose[3] <= 0.6:
            goal = self.end if self.target_end else self.start
            linear, angular, reached = patrol_command(*self.pose[:3], *goal)
            if reached:
                self.target_end = not self.target_end
            else:
                command.linear.x = linear
                command.angular.z = angular
        self.pub.publish(command)


def main():
    rclpy.init()
    node = ScriptedOpponent()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
