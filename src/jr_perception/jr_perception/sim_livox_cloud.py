#!/usr/bin/env python3
"""Облако ray-лидара Gazebo -> PointCloud2 в формате livox_ros_driver2.

livox_custom.py читает поля x y z intensity tag line timestamp, которых у
gazebo_ros_ray_sensor нет. Здесь они достраиваются: line -- номер строки
вертикальной развёртки по модулю scan_line, timestamp -- время скана плюс
point_period_ns на точку. Ray-лидар Gazebo снимает весь скан в один момент,
поэтому период мал (дескью почти нулевой), но времена возрастают, как требует
FAST-LIO2.

    /livox/lidar (Gazebo PointCloud2)  ->  /livox/lidar_livox (формат Livox)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

OUT = np.dtype(
    {
        "names": ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
        "offsets": [0, 4, 8, 12, 16, 17, 24],
        "itemsize": 32,
    }
)
FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
    PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
    PointField(name="timestamp", offset=24, datatype=PointField.FLOAT64, count=1),
]


def convert(cloud: PointCloud2, scan_line: int, point_period_ns: int) -> PointCloud2:
    names = {field.name: field.offset for field in cloud.fields}
    source = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4"] * 3,
            "offsets": [names["x"], names["y"], names["z"]],
            "itemsize": cloud.point_step,
        }
    )
    count = cloud.width * cloud.height
    points = np.frombuffer(cloud.data, dtype=source, count=count)
    rows = np.arange(count) // max(cloud.width, 1)
    valid = np.isfinite(points["x"]) & np.isfinite(points["y"]) & np.isfinite(points["z"])
    points, rows = points[valid], rows[valid]

    out = np.zeros(len(points), dtype=OUT)
    for name in ("x", "y", "z"):
        out[name] = points[name]
    out["intensity"] = 100.0
    out["line"] = rows % scan_line
    begin = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
    out["timestamp"] = begin + np.arange(len(points), dtype=np.float64) * point_period_ns

    message = PointCloud2()
    message.header = cloud.header
    message.height = 1
    message.width = len(points)
    message.fields = FIELDS
    message.is_bigendian = False
    message.point_step = OUT.itemsize
    message.row_step = OUT.itemsize * len(points)
    message.data = out.tobytes()
    message.is_dense = True
    return message


class SimLivoxCloud(Node):
    def __init__(self):
        super().__init__("sim_livox_cloud")
        source = self.declare_parameter("cloud_topic", "/livox/lidar").value
        target = self.declare_parameter("livox_topic", "/livox/lidar_livox").value
        self.scan_line = self.declare_parameter("scan_line", 4).value
        self.point_period_ns = self.declare_parameter("point_period_ns", 200).value
        self.publisher = self.create_publisher(PointCloud2, target, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, source, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, message: PointCloud2):
        self.publisher.publish(convert(message, self.scan_line, self.point_period_ns))


def main():
    rclpy.init()
    node = SimLivoxCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
