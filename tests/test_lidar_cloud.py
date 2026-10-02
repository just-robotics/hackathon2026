"""ROS serialization checks; run inside the real image with unittest."""
import struct
import unittest

try:
    from sensor_msgs.msg import PointCloud2, PointField
    from hsl_real.lidar_filter import filter_cloud
    from hsl_real.lidar_filter_core import DEFAULT_PARAMETERS
except ImportError:
    PointCloud2 = None


@unittest.skipIf(PointCloud2 is None, 'requires sourced ROS environment')
class LidarCloudTest(unittest.TestCase):
    def test_preserves_fields_and_padding_in_both_byte_orders(self):
        for big_endian in (False, True):
            with self.subTest(big_endian=big_endian):
                endian = '>' if big_endian else '<'
                records = [struct.pack(endian + 'fffBd', x, 0., 0., tag, stamp)
                           for x, tag, stamp in [(1., 0, 123.), (.2, 32, 124.),
                                                 (2., 64, 125.), (3., 16, 126.)]]
                cloud = PointCloud2(height=2, width=2, is_bigendian=big_endian,
                    point_step=21, row_step=45,
                    data=b''.join(records[:2]) + b'pad' + b''.join(records[2:]) + b'pad')
                cloud.header.frame_id = 'livox'
                cloud.header.stamp.sec = 123
                cloud.fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                    for name, offset, kind in [('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
                                                ('tag', 12, 2), ('timestamp', 13, 8)]]
                filtered, diag = filter_cloud(cloud, dict(DEFAULT_PARAMETERS,
                    self_occlusion_max_range=0.))
                self.assertEqual(bytes(filtered.data), records[0] + records[2] + records[3])
                self.assertEqual(filtered.header, cloud.header)
                self.assertEqual(filtered.fields, cloud.fields)
                self.assertEqual((filtered.height, filtered.width, filtered.row_step), (1, 3, 63))
                self.assertEqual(diag['low_confidence_points'], 1)
                self.assertEqual(len(cloud.data), 90)


if __name__ == '__main__':
    unittest.main()
