"""Read-only ROS bag audit before offline static mapping; run in the real image."""
import argparse
from collections import Counter
import json
from math import isfinite, sqrt
from pathlib import Path
import statistics


def audit(path, lidar_topic='/livox/lidar', odom_topic='/odom', imu_topic='/livox/imu'):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import yaml

    path = Path(path).resolve()
    metadata = yaml.safe_load((path / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader = rosbag2_py.SequentialReader()
    metadata_fallback = False
    try:
        reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=metadata['storage_identifier']),
                    rosbag2_py.ConverterOptions('', ''))
    except RuntimeError:
        # Newer ROS metadata stores QoS as YAML lists; Humble expects strings.
        # A single MCAP is self-describing. Never rewrite the source metadata.
        files = metadata['relative_file_paths']
        if metadata['storage_identifier'] != 'mcap' or len(files) != 1:
            raise
        source = (path / files[0]).resolve()
        if not source.is_relative_to(path) or not source.is_file():
            raise ValueError('MCAP path must be inside the bag directory')
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(source), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        metadata_fallback = True
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    counts, timing, headers = Counter(), {}, {}
    frames, edges, norms, poses = {}, {}, [], []
    cloud_layout = None
    invalid_samples = Counter()
    expected = {lidar_topic: 'sensor_msgs/msg/PointCloud2',
                odom_topic: 'nav_msgs/msg/Odometry', imu_topic: 'sensor_msgs/msg/Imu',
                '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage'}
    errors, warnings = [], []
    for topic, kind in expected.items():
        if topic in types and types[topic] != kind:
            errors.append(f'{topic}: expected {kind}, got {types[topic]}')
    classes = {t: get_message(k) for t, k in expected.items() if types.get(t) == k}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        counts[topic] += 1
        times = timing.setdefault(topic, {'first_ns': timestamp, 'last_ns': timestamp})
        times['last_ns'] = timestamp
        if topic not in classes:
            continue
        msg = deserialize_message(data, classes[topic])
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            stats = headers.setdefault(topic, {'first_ns': stamp, 'last_ns': stamp,
                                              'zero_stamps': 0, 'backwards_stamps': 0,
                                              'record_offset_min_s': (timestamp-stamp)*1e-9,
                                              'record_offset_max_s': (timestamp-stamp)*1e-9})
            stats['zero_stamps'] += stamp == 0
            stats['backwards_stamps'] += stamp < stats['last_ns']
            stats['last_ns'] = stamp
            stats['record_offset_min_s'] = min(stats['record_offset_min_s'], (timestamp-stamp)*1e-9)
            stats['record_offset_max_s'] = max(stats['record_offset_max_s'], (timestamp-stamp)*1e-9)
            frames.setdefault(topic, Counter())[msg.header.frame_id] += 1
        if topic == lidar_topic:
            if cloud_layout is None:
                cloud_layout = {'point_step': msg.point_step,
                                'fields': [{'name': f.name, 'datatype': f.datatype,
                                            'offset': f.offset, 'count': f.count} for f in msg.fields],
                                'first_cloud_points': msg.width * msg.height}
                if not {'x', 'y', 'z'} <= {f.name for f in msg.fields}:
                    errors.append('LiDAR cloud has no complete XYZ fields')
        elif topic == odom_topic:
            p = msg.pose.pose.position
            if all(isfinite(v) for v in (p.x, p.y, p.z)):
                poses.append((p.x, p.y, p.z))
            else:
                invalid_samples[topic] += 1
        elif topic == imu_topic:
            a = msg.linear_acceleration
            if all(isfinite(v) for v in (a.x, a.y, a.z)):
                norms.append(sqrt(a.x*a.x + a.y*a.y + a.z*a.z))
            else:
                invalid_samples[topic] += 1
        elif topic in ('/tf', '/tf_static'):
            for tf in msg.transforms:
                key = f'{tf.header.frame_id} -> {tf.child_frame_id}'
                edges.setdefault(topic, Counter())[key] += 1
    if not counts[lidar_topic]:
        errors.append(f'No LiDAR messages on {lidar_topic}')
    if not counts[odom_topic]:
        warnings.append('No wheel odometry: an odometry-based map cannot be built from this bag')
    if not counts[imu_topic]:
        warnings.append('No LiDAR IMU: suitability for LiDAR/inertial SLAM needs investigation')
    if not counts['/tf_static']:
        warnings.append('No static TF: need recorded sensor extrinsics from the session configuration')
    for topic, stats in headers.items():
        if stats['zero_stamps'] or stats['backwards_stamps']:
            warnings.append(f'{topic}: zero or nonmonotonic header stamps')
    for topic, count in invalid_samples.items():
        warnings.append(f'{topic}: {count} nonfinite odometry/IMU samples')
    motion = None
    if poses:
        motion = {'axis_span_m': [max(p[i] for p in poses)-min(p[i] for p in poses) for i in range(3)],
                  'sampled_path_m': sum(sqrt(sum((a[i]-b[i])**2 for i in range(3)))
                                        for a, b in zip(poses, poses[1:]))}
        if motion['sampled_path_m'] < .5:
            warnings.append('Very little recorded translation; full maze coverage is not established')
    if norms and statistics.median(norms) < 2.:
        warnings.append('IMU acceleration norm is near g units; confirm scaling before SI-based SLAM')
    topics = {}
    for topic, kind in types.items():
        times = timing.get(topic, {})
        span = (times.get('last_ns', 0)-times.get('first_ns', 0))*1e-9
        topics[topic] = {'type': kind, 'messages': counts[topic], 'record_span_s': span,
                         'approx_hz': (counts[topic]-1)/span if span > 0 else None,
                         'record_times': times, 'header_times': headers.get(topic),
                         'frames': dict(frames.get(topic, {}))}
    return {'bag': str(path), 'storage': metadata['storage_identifier'],
            'metadata_compatibility_fallback': metadata_fallback, 'topics': topics,
            'cloud_layout': cloud_layout, 'tf_edges': {t: dict(v) for t, v in edges.items()},
            'odometry_motion': motion,
            'imu_acceleration_norm_median': statistics.median(norms) if norms else None,
            'errors': errors, 'warnings': warnings,
            'map_quality_verified': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='Directory containing metadata.yaml and the bag data files')
    parser.add_argument('--lidar-topic', default='/livox/lidar')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--imu-topic', default='/livox/imu')
    args = parser.parse_args()
    report = audit(args.bag, args.lidar_topic, args.odom_topic, args.imu_topic)
    print(json.dumps(report, indent=2, allow_nan=False))
    if report['errors']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
