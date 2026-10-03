#!/usr/bin/env python3
"""Сравнение позы FAST-LIO2 в симуляции с точной позой Gazebo.

Эталон -- TF map -> base_footprint от gazebo_ros_p3d. Раз в report_period
секунд пишет в лог текущую и максимальную ошибки по положению и курсу и
публикует текущую в /localization/fastlio_sim_error (x, y -- ошибка по осям
карты, z -- ошибка курса, рад).
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, ExtrapolationException, TransformException, TransformListener


def yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class FastLioSimCheck(Node):
    def __init__(self):
        super().__init__("fastlio_sim_check")
        topic = self.declare_parameter("odometry_topic", "/localization/fastlio_sim").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        period = self.declare_parameter("report_period", 5.0).value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Vector3Stamped, "/localization/fastlio_sim_error", 10)
        self.create_subscription(Odometry, topic, self.on_odometry, 50)
        self.create_timer(period, self.report)
        # поза FAST-LIO2 датирована концом скана и новее последнего TF Gazebo:
        # ждёт в очереди, пока TF не догонит
        self.pending = deque(maxlen=200)
        self.create_timer(0.05, self.process)
        self.current = None
        self.worst = 0.0
        self.worst_yaw = 0.0
        self.count = 0

    def on_odometry(self, message: Odometry):
        self.pending.append(message)

    def process(self):
        while self.pending:
            message = self.pending[0]
            try:
                truth = self.buffer.lookup_transform(
                    self.map_frame, self.base_frame, Time.from_msg(message.header.stamp)
                )
            except ExtrapolationException:
                newest = self.buffer.get_latest_common_time(self.map_frame, self.base_frame) \
                    if self.buffer.can_transform(self.map_frame, self.base_frame, Time()) else None
                if newest is not None and Time.from_msg(message.header.stamp) < newest:
                    self.pending.popleft()  # старше буфера TF
                    continue
                return
            except TransformException:
                return
            self.pending.popleft()
            self.compare(message, truth)

    def compare(self, message: Odometry, truth):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        t = truth.transform
        dx, dy = p.x - t.translation.x, p.y - t.translation.y
        dyaw = math.atan2(math.sin(yaw(q) - yaw(t.rotation)), math.cos(yaw(q) - yaw(t.rotation)))
        self.current = (dx, dy, dyaw)
        self.worst = max(self.worst, math.hypot(dx, dy))
        self.worst_yaw = max(self.worst_yaw, abs(dyaw))
        self.count += 1
        out = Vector3Stamped()
        out.header = message.header
        out.vector.x, out.vector.y, out.vector.z = dx, dy, dyaw
        self.publisher.publish(out)

    def report(self):
        if self.current is None:
            self.get_logger().warning("Нет поз FAST-LIO2 или TF Gazebo", throttle_duration_sec=30.0)
            return
        dx, dy, dyaw = self.current
        self.get_logger().info(
            f"ошибка: {math.hypot(dx, dy) * 100:.1f} см (dx {dx * 100:+.1f}, dy {dy * 100:+.1f}), "
            f"курс {math.degrees(dyaw):+.2f} град; максимум {self.worst * 100:.1f} см, "
            f"{math.degrees(self.worst_yaw):.2f} град; поз {self.count}"
        )


def main():
    rclpy.init()
    node = FastLioSimCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
