"""Filter raw LiDAR independently of localization; preserve all point fields."""
from copy import deepcopy
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from .lidar_filter_core import DEFAULT_PARAMETERS, point_mask


def filter_cloud(msg, parameters):
    fields = {f.name: f for f in msg.fields}
    if any(name not in fields or fields[name].datatype != 7 for name in ('x', 'y', 'z')):
        raise ValueError('expected FLOAT32 XYZ fields')
    endian = '>' if msg.is_bigendian else '<'
    points = np.column_stack([np.ndarray((msg.height, msg.width), dtype=endian+'f4',
        buffer=msg.data, offset=fields[name].offset,
        strides=(msg.row_step, msg.point_step)).ravel() for name in ('x','y','z')])
    tags = None
    if 'tag' in fields:
        if fields['tag'].datatype != 2:
            raise ValueError('Livox tag must be UINT8')
        tags = np.ndarray((msg.height, msg.width), dtype='u1', buffer=msg.data,
            offset=fields['tag'].offset, strides=(msg.row_step, msg.point_step)).ravel()
    keep, diagnostics = point_mask(points, tags, **parameters)
    records = np.ndarray((msg.height, msg.width), dtype=f'V{msg.point_step}',
        buffer=msg.data, strides=(msg.row_step, msg.point_step)).ravel()
    result = deepcopy(msg)
    result.height = 1
    result.width = int(keep.sum())
    result.row_step = result.width * result.point_step
    result.data = records[keep].tobytes()
    result.is_dense = True
    diagnostics['tag_present'] = tags is not None
    return result, diagnostics


class LidarFilter(Node):
    def __init__(self):
        super().__init__('real_lidar_filter')
        topic = self.declare_parameter('lidar_topic', '/livox/lidar').value
        self.parameters = {key:self.declare_parameter(key,value).value
                           for key,value in DEFAULT_PARAMETERS.items()}
        angles = self.parameters['self_occlusion_centers_deg']
        width = self.parameters['self_occlusion_half_angle_deg']
        distance = self.parameters['self_occlusion_max_range']
        if (not all(math.isfinite(a) for a in angles) or not 0 < width < 45
                or not math.isfinite(distance) or not 0 <= distance <= 1.):
            raise ValueError('invalid LiDAR self-occlusion geometry')
        self.pub = self.create_publisher(PointCloud2, '/sensing/lidar/points_filtered', qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(String, '/sensing/lidar/filter_diagnostics', 10)
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, msg):
        try:
            filtered, diag = filter_cloud(msg, self.parameters)
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.)
            return
        self.pub.publish(filtered)
        diag['stamp_s'] = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.diagnostics.publish(String(data=json.dumps(diag)))


def main():
    rclpy.init(); node=LidarFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
