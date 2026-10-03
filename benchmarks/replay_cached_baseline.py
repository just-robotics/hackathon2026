#!/usr/bin/env python3
"""Run the existing Python detector on the same cached real frames and poses.

This offline baseline does not publish ROS messages or use PoseRelay/latest TF.
It calls the unchanged detector's processing method at each filtered cloud's
recorded header timestamp and uses the cache's declared map or local pose.
"""

from __future__ import annotations

import argparse
import array
import csv
import ctypes
import ctypes.util
import hashlib
import json
from pathlib import Path
import time


REPOSITORY = Path(__file__).resolve().parents[1]
DETECTOR_SOURCES = (
    'src/jr_perception/jr_perception/robot_detector.py',
    'src/jr_perception/jr_perception/segmentation.py',
    'src/jr_perception/jr_perception/tracker.py',
    'src/jr_perception/jr_perception/map_background.py',
    'src/jr_perception/jr_perception/background.py',
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


class Zstd:
    def __init__(self):
        name = ctypes.util.find_library('zstd')
        if not name:
            raise RuntimeError('libzstd is required to read the real detector cache')
        self.lib = ctypes.CDLL(name)
        self.lib.ZSTD_decompress.argtypes = (ctypes.c_void_p, ctypes.c_size_t,
                                             ctypes.c_void_p, ctypes.c_size_t)
        self.lib.ZSTD_decompress.restype = ctypes.c_size_t
        self.lib.ZSTD_isError.argtypes = (ctypes.c_size_t,)
        self.lib.ZSTD_isError.restype = ctypes.c_uint

    def decompress(self, encoded, expected_size):
        output = ctypes.create_string_buffer(expected_size)
        source = ctypes.create_string_buffer(encoded)
        actual = self.lib.ZSTD_decompress(output, expected_size, source, len(encoded))
        if self.lib.ZSTD_isError(actual) or actual != expected_size:
            raise ValueError('corrupt zstd cloud block')
        return output.raw


class Collect:
    def __init__(self, callback=None):
        self.callback = callback

    def publish(self, message):
        if self.callback is not None:
            self.callback(message)


def occupancy_from_cache(cache):
    from nav_msgs.msg import OccupancyGrid
    info = json.loads((cache / 'map.json').read_text())
    raw = (cache / 'map.bin').read_bytes()
    if hashlib.sha256(raw).hexdigest() != info['occupancy_sha256']:
        raise ValueError('map.bin hash does not match map.json')
    message = OccupancyGrid()
    stamp_ns = info['header_ns']
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = info['frame_id']
    message.info.resolution = info['resolution']
    message.info.width = info['width']
    message.info.height = info['height']
    message.info.origin.position.x, message.info.origin.position.y, \
        message.info.origin.position.z = info['origin_translation']
    message.info.origin.orientation.x, message.info.origin.orientation.y, \
        message.info.origin.orientation.z, message.info.origin.orientation.w = info['origin_quaternion']
    signed = array.array('b')
    signed.frombytes(raw)
    message.data = signed
    return message


def cloud_from_cache(row, layout, payload):
    from sensor_msgs.msg import PointCloud2, PointField
    message = PointCloud2()
    stamp_ns = int(row['header_ns'])
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = row['frame_id']
    message.width = int(row['width'])
    message.height = int(row['height'])
    message.point_step = int(row['point_step'])
    message.row_step = int(row['row_step'])
    message.is_bigendian = bool(int(row['is_bigendian']))
    message.is_dense = bool(int(row['is_dense']))
    message.fields = [PointField(name=field['name'], offset=field['offset'],
                                 datatype=field['datatype'], count=field['count'])
                      for field in layout['fields']]
    message.data = array.array('B', payload)
    return message


def sensor_pose(row):
    import numpy as np
    from jr_perception.robot_detector import quaternion_matrix
    position = np.array([float(row[f'map_sensor_{axis}']) for axis in 'xyz'])
    quaternion = [float(row[f'map_sensor_q{axis}']) for axis in 'xyzw']
    return quaternion_matrix(quaternion), position


def make_detector(profile, *, local_mapless=False):
    import rclpy
    from jr_perception.robot_detector import RobotDetector
    args = ['--ros-args', '--params-file', str(profile),
            '-p', 'background_topic:=/map',
            '-p', f'world_frame:={"odom" if local_mapless else "map"}',
            '-p', 'pose_topic:=/cached/self_pose', '-p', 'base_frame:=base_footprint',
            '-p', 'cloud_topic:=/sensing/lidar/points_filtered',
            '-p', 'log_period:=1000000.0']
    rclpy.init(args=args)
    detector = RobotDetector()
    if local_mapless:
        # Constructor needs a background source. The raw-only recordings have
        # no anchored map, so this comparison deliberately runs the unchanged
        # geometry/tracker path without map subtraction or map visibility.
        # It is an ablation, not an estimate of production map-backed quality.
        detector.map_topic = ''
    return detector


def replay(cache, output, profile, limit):
    from jr_perception.robot_detector import stamp_seconds

    manifest = json.loads((cache / 'manifest.json').read_text())
    schema = manifest['schema']
    if schema not in ('real-detector-cache-v1', 'real-detector-cache-local-v1'):
        raise ValueError('unknown cache schema')
    local_mapless = schema == 'real-detector-cache-local-v1'
    if local_mapless and (manifest.get('world_frame') != 'odom' or manifest.get('map_available') is not False):
        raise ValueError('local cache must explicitly identify unanchored odom without map')
    if manifest['truncated'] and limit is None:
        raise ValueError('cache is truncated; set --limit-frames for a smoke run')
    artifacts = {item['name']: item for item in manifest['artifacts']}
    required = ('frames.tsv', 'clouds.zstbin', 'filter_stats.jsonl') if local_mapless else (
        'frames.tsv', 'clouds.zstbin', 'map.bin', 'map.json')
    for name in required:
        if sha256(cache / name) != artifacts[name]['sha256']:
            raise ValueError(f'{name} no longer matches manifest')

    detector = make_detector(profile, local_mapless=local_mapless)
    outputs = {}
    def collect_odom(message):
        outputs['odom'] = message
    def collect_health(message):
        outputs['health'] = json.loads(message.data)
    detector.odometry_publisher = Collect(collect_odom)
    detector.health_publisher = Collect(collect_health)
    for name in ('foreground_publisher', 'marker_publisher', 'robot_marker_publisher',
                 'box_marker_publisher', 'visible_publisher'):
        setattr(detector, name, Collect())
    original_step = detector.tracker.step
    def observed_step(moment, detections):
        outputs['candidates'] = [
            {'xy': detection.center.tolist(), 'strong': bool(detection.strong),
             'sigma': float(detection.sigma)} for detection in detections]
        selected = original_step(moment, detections)
        if selected is not None:
            outputs['selected_last_update_s'] = float(selected.last_update)
        return selected
    detector.tracker.step = observed_step
    if not local_mapless:
        detector.on_map(occupancy_from_cache(cache))

    zstd = Zstd()
    counts = {'filtered_frames': 0, 'processed': 0, 'measurement': 0,
              'prediction': 0, 'no_output': 0, 'missing_pose': 0}
    started = time.monotonic()
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with (cache / 'frames.tsv').open(newline='') as index, \
             (cache / 'clouds.zstbin').open('rb') as binary, \
             output.open('w') as destination:
            for row in csv.DictReader(index, delimiter='\t'):
                if row['topic'] != 'filtered':
                    continue
                if limit is not None and counts['filtered_frames'] >= limit:
                    break
                counts['filtered_frames'] += 1
                outputs.clear()
                stamp_ns = int(row['header_ns'])
                result = {'bag': manifest['session'],
                          'record_index': int(row['record_index']),
                          'stamp_ns': stamp_ns,
                          'stamp_s': stamp_ns / 1_000_000_000,
                          'pose_status': row['pose_status'],
                          'pose_source': row['pose_source'],
                          'processed': False, 'status': 'missing_pose',
                          'has_measurement': False, 'measurement_xy': None,
                          'has_prediction': False, 'prediction_xy': None,
                          'timing_ms': None, 'candidates': [],
                          'selected': False}
                if row['pose_status'] != 'ok':
                    counts['missing_pose'] += 1
                    destination.write(json.dumps(result, separators=(',', ':')) + '\n')
                    continue
                binary.seek(int(row['offset']))
                encoded = binary.read(int(row['compressed_size']))
                if len(encoded) != int(row['compressed_size']):
                    raise ValueError('truncated cloud cache')
                payload = zstd.decompress(encoded, int(row['uncompressed_size']))
                layout = manifest['layouts'][int(row['layout_id'])]
                cloud = cloud_from_cache(row, layout, payload)
                rotation, position = sensor_pose(row)
                detector.sensor_in_world = lambda _frame, _pose, rot=rotation, pos=position: (rot, pos)
                before = detector.stats['frames']
                frame_started = time.perf_counter()
                detector.process(cloud, stamp_seconds(cloud.header.stamp), (None, None))
                result['timing_ms'] = (time.perf_counter() - frame_started) * 1000
                result['processed'] = detector.stats['frames'] == before + 1
                result['candidates'] = outputs.get('candidates', [])
                result['candidate_count'] = len(result['candidates'])
                result['diagnostics'] = outputs.get('health', {})
                if result['processed']:
                    counts['processed'] += 1
                else:
                    result['status'] = 'detector_unprocessed'
                if 'odom' in outputs:
                    odom = outputs['odom']
                    x, y = float(odom.pose.pose.position.x), float(odom.pose.pose.position.y)
                    measured = abs(outputs.get('selected_last_update_s', -1e100) - result['stamp_s']) < 1e-5
                    result['selected'] = True
                    result['child_frame_id'] = odom.child_frame_id
                    result['pose_covariance_x'] = float(odom.pose.covariance[0])
                    result['pose_covariance_y'] = float(odom.pose.covariance[7])
                    result['twist_linear_x'] = float(odom.twist.twist.linear.x)
                    result['twist_linear_y'] = float(odom.twist.twist.linear.y)
                    result['selected_xy'] = [x, y]
                    result['selected_last_update_s'] = outputs.get('selected_last_update_s')
                    if measured:
                        result['status'] = 'measurement'
                        result['has_measurement'] = True
                        result['measurement_xy'] = [x, y]
                        counts['measurement'] += 1
                    else:
                        result['status'] = 'prediction'
                        result['has_prediction'] = True
                        result['prediction_xy'] = [x, y]
                        counts['prediction'] += 1
                elif result['processed']:
                    result['status'] = 'none'
                    counts['no_output'] += 1
                destination.write(json.dumps(result, separators=(',', ':')) + '\n')
    finally:
        detector.destroy_node()
        import rclpy
        rclpy.shutdown()
    report = {
        'schema': 'cached-baseline-v1', 'cache': str(cache), 'bag': manifest['session'],
        'cache_signature': manifest['signature'],
        'cache_manifest_sha256': sha256(cache / 'manifest.json'),
        'profile': str(profile),
        'source_sha256': {name: sha256(REPOSITORY / name)
                          for name in DETECTOR_SOURCES},
        'runner_sha256': sha256(Path(__file__)),
        'profile_sha256': sha256(profile),
        'output_sha256': sha256(output),
        'world_frame': 'odom' if local_mapless else 'map',
        'comparison_variant': 'mapless_local_odom_ablation' if local_mapless else 'recorded_map_anchored',
        'map_available': not local_mapless,
        'counts': counts, 'truncated': limit is not None or manifest['truncated'],
        'elapsed_wall_s': time.monotonic() - started,
        'method': ('unchanged RobotDetector.process geometry/tracker path on every offline-filtered real cloud, '
                   'recorded unanchored odom->sensor pose; map background and map visibility disabled because '
                   'the bag has no /map or map->odom; ablation, not production map-backed quality'
                   if local_mapless else
                   'unchanged RobotDetector.process on each filtered cloud, cached map_from_sensor pose, '
                   'no PoseRelay/latest TF; outputs are not ground-truth labels'),
    }
    output.with_suffix('.manifest.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'output': str(output), 'counts': counts,
                      'elapsed_wall_s': report['elapsed_wall_s']}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('caches', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path,
                        help='directory receiving one JSONL per bag')
    parser.add_argument('--profile', type=Path,
                        default=REPOSITORY / 'src/jr_perception/config/real.yaml')
    parser.add_argument('--limit-frames', type=int,
                        help='development smoke run only; output marked truncated')
    args = parser.parse_args()
    if args.limit_frames is not None and args.limit_frames <= 0:
        parser.error('--limit-frames must be positive')
    if len(args.caches) > 1 and args.output.suffix:
        parser.error('--output must be a directory')
    for cache in args.caches:
        if not cache.is_dir():
            raise FileNotFoundError(cache)
        output = args.output / (cache.name + '.jsonl')
        replay(cache, output, args.profile, args.limit_frames)


if __name__ == '__main__':
    main()
