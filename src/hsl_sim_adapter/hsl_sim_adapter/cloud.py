"""Small PointCloud2 XYZ reader/writer and rigid transform helper."""

import struct
from math import isfinite


def read_xyz(msg, limit=6000):
    fields = {field.name: field for field in msg.fields}
    if any(name not in fields for name in ("x", "y", "z")):
        return []
    count = msg.width * msg.height
    stride = max(1, count // limit)
    unpack = struct.Struct((">" if msg.is_bigendian else "<") + "f").unpack_from
    points = []
    for index in range(0, count, stride):
        row, col = divmod(index, msg.width)
        offset = row * msg.row_step + col * msg.point_step
        try:
            point = tuple(unpack(msg.data, offset + fields[name].offset)[0]
                          for name in ("x", "y", "z"))
        except (struct.error, ValueError):
            continue
        if all(isfinite(value) for value in point):
            points.append(point)
    return points


def transform(point, translation, quaternion):
    """Rotate a point with a normalized quaternion, then translate it."""
    x, y, z = point
    qx, qy, qz, qw = quaternion
    tx = 2 * (qy * z - qz * y)
    ty = 2 * (qz * x - qx * z)
    tz = 2 * (qx * y - qy * x)
    rx = x + qw * tx + qy * tz - qz * ty
    ry = y + qw * ty + qz * tx - qx * tz
    rz = z + qw * tz + qx * ty - qy * tx
    return rx + translation[0], ry + translation[1], rz + translation[2]


def make_cloud(header, points):
    from sensor_msgs.msg import PointCloud2, PointField

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<fff", *point) for point in points)
    return msg
