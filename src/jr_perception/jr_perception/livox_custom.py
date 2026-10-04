#!/usr/bin/env python3
"""Облако Livox PointCloud2 -> livox_ros_driver2/CustomMsg на лету, для FAST-LIO2.

FAST-LIO2 берёт настоящие времена точек Mid-360 только из CustomMsg
(offset_time); для PointCloud2 он восстанавливает их по углу, как у
вращающегося лидара, а для неповторяющегося паттерна это неверно. Стек
робота и бэги дают PointCloud2 от livox_ros_driver2 -- здесь он
переводится в CustomMsg на лету.
Сообщение собирается сразу в CDR и публикуется байтами: 20 тыс. объектов
CustomPoint на скан в Python не успевали бы за 10 Гц.

    /livox/lidar (PointCloud2)  ->  /livox/lidar_custom (CustomMsg)
"""

import struct

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

# CustomPoint в CDR: offset_time u32, x y z f32, reflectivity tag line u8 и
# байт выравнивания до 4 перед следующим u32
POINT = np.dtype(
    [
        ("offset_time", "<u4"),
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("reflectivity", "u1"),
        ("tag", "u1"),
        ("line", "u1"),
        ("pad", "u1"),
    ]
)


def align(buffer: bytearray, size: int):
    # выравнивание в CDR считается от начала данных, после 4 байт заголовка
    while (len(buffer) - 4) % size:
        buffer.append(0)


def serialize(cloud: PointCloud2) -> bytes:
    """CustomMsg в CDR по облаку livox_ros_driver2 (поля x y z intensity tag line timestamp)"""
    fields = {field.name: field.offset for field in cloud.fields}
    source = np.dtype(
        {
            "names": ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
            "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
            "offsets": [
                fields[name]
                for name in ("x", "y", "z", "intensity", "tag", "line", "timestamp")
            ],
            "itemsize": cloud.point_step,
        }
    )
    points = np.frombuffer(cloud.data, dtype=source, count=cloud.width * cloud.height)
    stamps = points["timestamp"].astype(np.uint64)
    timebase = int(stamps.min()) if len(points) else 0

    out = np.zeros(len(points), dtype=POINT)
    out["offset_time"] = (stamps - np.uint64(timebase)).astype(np.uint32)
    for name in ("x", "y", "z", "tag", "line"):
        out[name] = points[name]
    out["reflectivity"] = np.clip(points["intensity"], 0, 255).astype(np.uint8)

    frame = cloud.header.frame_id.encode() + b"\0"
    buffer = bytearray(b"\x00\x01\x00\x00")  # CDR little endian
    sec, nanosec = divmod(timebase, 1_000_000_000)
    buffer += struct.pack("<iI", sec, nanosec)
    buffer += struct.pack("<I", len(frame)) + frame
    align(buffer, 8)
    buffer += struct.pack("<Q", timebase)
    buffer += struct.pack("<I", len(points))
    buffer += struct.pack("<B3s", 0, b"\0\0\0")  # lidar_id, rsvd
    align(buffer, 4)
    buffer += struct.pack("<I", len(points))
    buffer += out.tobytes()
    # после последней точки выравнивание не пишется
    return bytes(buffer[:-1]) if len(points) else bytes(buffer)


class LivoxCustom(Node):
    def __init__(self):
        super().__init__("livox_custom")
        source = self.declare_parameter("cloud_topic", "/livox/lidar").value
        target = self.declare_parameter("custom_topic", "/livox/lidar_custom").value
        # reliable: FAST-LIO2 подписывается reliable, а с best effort он не получал бы ничего
        self.publisher = self.create_publisher(CustomMsg, target, 10)
        self.create_subscription(PointCloud2, source, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, message: PointCloud2):
        self.publisher.publish(serialize(message))


def main():
    rclpy.init()
    node = LivoxCustom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
