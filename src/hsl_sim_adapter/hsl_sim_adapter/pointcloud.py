"""PointCloud2 decoding for observation transport and offline replay.

No detection or classification is performed here.
"""
import numpy as np
from sensor_msgs.msg import PointField


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def cloud_xyz(msg):
    """Vectorized XYZ, respecting organized row padding and endianness."""
    fields = {f.name: f for f in msg.fields}
    if not msg.width or not msg.height:
        return np.empty((0, 3))
    if any(n not in fields or fields[n].datatype != PointField.FLOAT32 for n in ('x','y','z')):
        raise ValueError('navigation/scan must contain FLOAT32 XYZ')
    dtype = np.dtype({'names': ['x','y','z'], 'formats': [('>' if msg.is_bigendian else '<')+'f4']*3,
                      'offsets': [fields[n].offset for n in ('x','y','z')], 'itemsize': msg.point_step})
    data = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step))
    return np.stack([data[n].ravel() for n in ('x','y','z')], axis=1)
