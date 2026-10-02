#!/usr/bin/env python3

"""Карта занятости по описанию мира Gazebo.

Стены лабиринта заданы в SDF box-коллизиями с известными позами, поэтому
карту не нужно накапливать лидаром: она считается аналитически из того же
файла, который грузит симулятор. Это ground truth -- без дрейфа, артефактов
и необходимости объезжать лабиринт.

Публикует nav_msgs/OccupancyGrid в /map с transient_local QoS, чтобы
подписчики, поднявшиеся позже, получили карту без перезапуска.
"""

import numpy as np
import rclpy

from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


from jr_map.sdf_geometry import OCCUPIED, collect_boxes, rasterize

class SdfMapServer(Node):
    """Нода, публикующая карту по SDF-миру"""

    def __init__(self):
        super().__init__("sdf_map_server")

        self.declare_parameter("world", "")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("padding", 1.0)
        self.declare_parameter("z_slice", 0.25)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/map")
        # Сдвиг начала карты; по умолчанию она совпадает с координатами мира.
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)

        world = self.get_parameter("world").value
        resolution = self.get_parameter("resolution").value
        padding = self.get_parameter("padding").value
        z_slice = self.get_parameter("z_slice").value
        frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value
        spawn = (
            self.get_parameter("spawn_x").value,
            self.get_parameter("spawn_y").value,
        )

        if not world:
            self.get_logger().error("Параметр world не задан")
            raise SystemExit(1)

        boxes = collect_boxes(world, z_slice)

        if not boxes:
            self.get_logger().error(
                f"В {world} нет box-коллизий на высоте {z_slice} м. "
                "Карта не построена"
            )
            raise SystemExit(1)

        grid, origin_x, origin_y = rasterize(
            boxes, resolution, padding, spawn
        )

        message = OccupancyGrid()
        message.header.frame_id = frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = resolution
        message.info.width = grid.shape[1]
        message.info.height = grid.shape[0]
        message.info.origin.position.x = origin_x
        message.info.origin.position.y = origin_y
        message.info.origin.orientation.w = 1.0
        message.data = grid.reshape(-1).tolist()

        # transient_local: карта публикуется один раз, но доходит и до тех,
        # кто подписался позже
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(OccupancyGrid, topic, qos)
        self.publisher.publish(message)

        occupied = int(np.count_nonzero(grid == OCCUPIED))
        self.get_logger().info(
            f"Карта по {world}: {message.info.width}x{message.info.height} "
            f"ячеек, {resolution} м/ячейка, боксов {len(boxes)}, "
            f"занято {occupied} ячеек "
            f"({occupied * resolution ** 2:.1f} м2), "
            f"origin ({origin_x:.2f}, {origin_y:.2f}) "
            f"от начала ({spawn[0]:.2f}, {spawn[1]:.2f}) -> {topic}"
        )


def main():
    rclpy.init()
    node = SdfMapServer()
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
