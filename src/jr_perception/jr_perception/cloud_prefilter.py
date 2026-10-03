#!/usr/bin/env python3
"""Фильтры облака лидара до сегментации земли.

Облако переводится в base_frame по статическому TF лидара на базе (у
base_footprint z вверх и пол на z = 0) и фильтруется там:

- свой робот -- вертикальный цилиндр вокруг оси базы: радиус self_radius,
  от пола до self_height (корпус, пластины, стойки, фантомы от них);
- всё выше max_height над полом;
- всё дальше max_range от оси базы по горизонтали;
- точки с нечисловыми координатами.

Результат публикуется в base_frame: дальше его берёт
linefit_ground_segmentation с sensor_height 0 (пол уже на z = 0), а облако
препятствий от него -- детектор.

    cloud_topic (PointCloud2, фрейм лидара) -> output_topic (PointCloud2, base_frame)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from jr_perception.robot_detector import cloud_to_xyz, quaternion_matrix, xyz_to_cloud


class CloudPrefilter(Node):
    def __init__(self):
        super().__init__("cloud_prefilter")
        source = self.declare_parameter("cloud_topic", "/livox/lidar").value
        target = self.declare_parameter("output_topic", "/perception/cloud_prefiltered").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        # Kobuki R=0.178 с запасом на пластины и шум; по высоте -- до верха
        # лидара с креплением
        self.self_radius = self.declare_parameter("self_radius", 0.25).value
        self.self_height = self.declare_parameter("self_height", 0.60).value
        self.max_height = self.declare_parameter("max_height", 0.70).value
        # 0 -- без обрезки по дальности
        self.max_range = self.declare_parameter("max_range", 6.0).value

        self.offset = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # reliable: годится и для reliable-, и для best effort-подписчиков
        self.publisher = self.create_publisher(PointCloud2, target, 5)
        self.create_subscription(PointCloud2, source, self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f"{source} -> {target} ({self.base_frame}): свой робот R {self.self_radius} м "
            f"до {self.self_height} м, выше {self.max_height} м и дальше "
            f"{self.max_range} м -- прочь"
        )

    def on_cloud(self, message: PointCloud2):
        if self.offset is None:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, message.header.frame_id, Time()
                )
            except TransformException as error:
                self.get_logger().warning(
                    f"Нет TF {self.base_frame} -> {message.header.frame_id}: {error}",
                    throttle_duration_sec=5.0,
                )
                return
            tr, q = t.transform.translation, t.transform.rotation
            # лидар на базе стоит неподвижно: трансформ берётся один раз
            self.offset = (
                quaternion_matrix([q.x, q.y, q.z, q.w]),
                np.array([tr.x, tr.y, tr.z]),
            )

        rotation, translation = self.offset
        points = cloud_to_xyz(message) @ rotation.T + translation
        horizontal = np.hypot(points[:, 0], points[:, 1])
        own = (horizontal <= self.self_radius) & (points[:, 2] <= self.self_height)
        keep = ~own & (points[:, 2] <= self.max_height)
        if self.max_range > 0.0:
            keep &= horizontal <= self.max_range

        header = Header(stamp=message.header.stamp, frame_id=self.base_frame)
        self.publisher.publish(xyz_to_cloud(header, points[keep]))


def main():
    rclpy.init()
    node = CloudPrefilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
