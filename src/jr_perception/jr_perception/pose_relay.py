#!/usr/bin/env python3
"""Поза робота во фрейме map для robot_detector.py: /odom + map -> odom из /tf.

/navigation/self в бэгах идёт с дырами до 10 с, а детектор ждёт позу на
момент каждого скана. /odom идёт ровно ~20 Гц, поправку AMCL (map -> odom)
берём последнюю из /tf. Буфер TF не нужен: при повторе бэга (-l) время идёт
назад, и tf2 отбросил бы такие данные как старые.

    /odom (odom -> base_footprint)  +  /tf map -> odom  ->  /detector/self_pose
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseRelay(Node):
    def __init__(self):
        super().__init__("pose_relay")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.map_to_odom = None
        self.publisher = self.create_publisher(Odometry, "/detector/self_pose", 50)
        self.create_subscription(
            TFMessage, "/tf", self.on_tf, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)

    def on_tf(self, message: TFMessage):
        for transform in message.transforms:
            if (
                transform.header.frame_id == self.map_frame
                and transform.child_frame_id == self.odom_frame
            ):
                t = transform.transform.translation
                self.map_to_odom = (t.x, t.y, yaw_of(transform.transform.rotation))

    def on_odom(self, message: Odometry):
        if self.map_to_odom is None:
            self.get_logger().info("Жду map -> odom в /tf", throttle_duration_sec=5.0)
            return

        mx, my, myaw = self.map_to_odom
        p = message.pose.pose.position
        yaw = yaw_of(message.pose.pose.orientation)
        cos, sin = math.cos(myaw), math.sin(myaw)

        out = Odometry()
        out.header.stamp = message.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = message.child_frame_id
        out.pose.pose.position.x = mx + cos * p.x - sin * p.y
        out.pose.pose.position.y = my + sin * p.x + cos * p.y
        out.pose.pose.orientation.z = math.sin((myaw + yaw) / 2.0)
        out.pose.pose.orientation.w = math.cos((myaw + yaw) / 2.0)
        out.twist = message.twist
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = PoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
