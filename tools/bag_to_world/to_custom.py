#!/usr/bin/env python3
"""Бэг livox_ros_driver2 PointCloud2 -> бэг с livox_ros_driver2/CustomMsg.

FAST-LIO2 берёт настоящие времена точек только из CustomMsg (offset_time),
а для PointCloud2 восстанавливает их по углу, как у вращающегося лидара --
для неповторяющегося паттерна Mid-360 это неверно.

    to_custom.py <входной бэг> <выходной бэг>
"""

import struct
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
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


def main():
    source, target = sys.argv[1], sys.argv[2]

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=source, storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=target, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    # для SLAM нужны только лидар и IMU
    keep = ("/livox/lidar", "/livox/imu")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(keep)))
    types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
        if topic.name in keep
    }
    for name, kind in types.items():
        if name == "/livox/lidar":
            kind = "livox_ros_driver2/msg/CustomMsg"
        writer.create_topic(
            rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format="cdr")
        )

    count = 0
    while reader.has_next():
        topic, data, moment = reader.read_next()
        if topic == "/livox/lidar":
            data = serialize(deserialize_message(data, PointCloud2))
            count += 1
        writer.write(topic, data, moment)

    print(f"{count} сканов переведено в CustomMsg -> {target}")


if __name__ == "__main__":
    main()
