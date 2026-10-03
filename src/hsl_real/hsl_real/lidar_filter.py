"""Offline Python oracle for the production C++ field-preserving LiDAR filter."""
from array import array
from copy import deepcopy
import numpy as np
from sensor_msgs.msg import PointCloud2
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
    result = PointCloud2()
    result.header = deepcopy(msg.header)
    result.fields = deepcopy(msg.fields)
    result.is_bigendian = msg.is_bigendian
    result.point_step = msg.point_step
    result.height = 1
    result.width = int(keep.sum())
    result.row_step = result.width * result.point_step
    # ROS uint8[] setter accepts a typed array directly; bytes triggers two
    # Python loops over every byte before building the same array.
    result.data = array('B', records[keep].tobytes())
    result.is_dense = True
    diagnostics['tag_present'] = tags is not None
    return result, diagnostics
