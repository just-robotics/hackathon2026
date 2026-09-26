"""A repeatable, deliberately simple opponent for early Gazebo tests."""

import rclpy
from geometry_msgs.msg import Twist
from hsl_interfaces.msg import PlanningIntent
from rclpy.node import Node


class ScriptedOpponent(Node):
    def __init__(self):
        super().__init__("scripted_opponent")
        self.allowed = False
        self.start = None
        self.pub = self.create_publisher(Twist, "/opponent/cmd_vel", 10)
        self.create_subscription(PlanningIntent, "/navigation/intent", self.on_intent, 10)
        self.create_timer(0.1, self.tick)

    def on_intent(self, msg):
        self.allowed = msg.behavior not in (PlanningIntent.WAIT, PlanningIntent.STOP)
        if not self.allowed:
            self.start = None

    def tick(self):
        command = Twist()
        if self.allowed:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.start is None:
                self.start = now
            phase = (now - self.start) % 16.0
            if phase < 5.0:
                command.linear.x = 0.25
            elif phase < 8.0:
                command.angular.z = 0.7
            elif phase < 13.0:
                command.linear.x = 0.25
            else:
                command.angular.z = 0.7
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
