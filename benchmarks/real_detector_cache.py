#!/usr/bin/env python3
"""Extract a deterministic, read-only offline cache from real MCAP bags.

Run under ROS 2 Humble (for rosbag2_py and message types). No ROS nodes are
started and no messages are published. The source bags are opened read-only.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import ctypes
import ctypes.util
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time


REPOSITORY = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = Path('/home/eddyswens/ROS/hsl2026Extra')
SESSIONS = (
    'recordings_last8/20261003T093554.608208Z-autonomous',
    'recordings_last8/20261003T094119.687783Z-autonomous',
    'recordings_last8/20261003T094252.665737Z-autonomous',
    'recordings_last8/20261003T095154.890891Z-bag',
    'recordings_last8/20261003T095433.714618Z-bag',
    'recordings_last8/20261003T095724.528830Z-bag',
    'recordings_last8/20261003T095806.834495Z-bag',
    'recordings_last8/20261003T095829.438480Z-bag',
    'robot_20261003_latest3/20261003T125740.521431Z-autonomous',
    'robot_20261003_latest3/20261003T125900.414332Z-autonomous',
    'robot_20261003_1815_1830/20261003T151344.712864Z-autonomous',
    'robot_20261003_1815_1830/20261003T151614.954087Z-autonomous',
    'robot_20261003_1815_1830/20261003T152604.245289Z-autonomous',
    'robot_20261003_1815_1830/20261003T152852.935550Z-autonomous',
    'robot_20261003_1815_1830/20261003T152957.624620Z-autonomous',
)
TOPICS = {
    '/livox/lidar': 'raw',
    '/sensing/lidar/points_filtered': 'filtered',
}
POSE_TOPICS = (
    '/odom', '/navigation/self', '/localization/fastlio/odometry',
    '/localization/kinematic_state',
)
FIRST_PASS_TOPICS = set(POSE_TOPICS) | {'/tf', '/tf_static', '/map', '/amcl_pose'}
ODOM_MAX_GAP_NS = 150_000_000
MAP_TF_MAX_GAP_NS = 300_000_000
SENSOR_TF_MAX_GAP_NS = 300_000_000
AMCL_COV_MAX_AGE_NS = 500_000_000
FIELDS = (
    'record_index', 'topic', 'bag_ns', 'header_ns', 'offset', 'compressed_size',
    'uncompressed_size', 'frame_id', 'width', 'height', 'point_step', 'row_step',
    'is_bigendian', 'is_dense', 'layout_id', 'x_offset', 'y_offset', 'z_offset',
    'intensity_offset', 'tag_offset', 'line_offset', 'timestamp_offset',
    'pose_status', 'pose_source', 'map_sensor_x', 'map_sensor_y', 'map_sensor_z',
    'map_sensor_qx', 'map_sensor_qy', 'map_sensor_qz', 'map_sensor_qw',
    'pose_var_x', 'pose_var_y', 'pose_var_yaw', 'odom_gap_ns', 'map_tf_gap_ns',
    'sensor_tf_gap_ns', 'amcl_cov_age_ns',
)


def ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def unit_quat(q):
    length = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(length) or length < 1e-9:
        raise ValueError('invalid quaternion')
    return tuple(v / length for v in q)


def quat_of(q):
    return unit_quat((q.x, q.y, q.z, q.w))


def vec_of(v):
    return (float(v.x), float(v.y), float(v.z))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return unit_quat((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def quat_inv(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def compose(a, b):
    p = rotate(a[1], b[0])
    return (tuple(a[0][i] + p[i] for i in range(3)), quat_mul(a[1], b[1]))


def inverse(t):
    q = quat_inv(t[1])
    return (rotate(q, tuple(-v for v in t[0])), q)


def slerp(a, b, fraction):
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b = tuple(-x for x in b)
        dot = -dot
    if dot > 0.9995:
        return unit_quat(tuple(x + fraction * (y - x) for x, y in zip(a, b)))
    theta = math.acos(max(-1.0, min(1.0, dot)))
    scale = math.sin(theta)
    wa = math.sin((1 - fraction) * theta) / scale
    wb = math.sin(fraction * theta) / scale
    return unit_quat(tuple(wa * x + wb * y for x, y in zip(a, b)))


def interpolate(samples, stamp_ns, max_gap_ns):
    """Return bracketed SE(3) transform, covariance and bracket gap."""
    if not samples:
        return None, 'missing', None
    index = bisect_left(samples, (stamp_ns,))
    if index < len(samples) and samples[index][0] == stamp_ns:
        sample = samples[index]
        return sample[1:], 'ok', 0
    if index == 0 or index == len(samples):
        return None, 'outside', None
    before, after = samples[index - 1], samples[index]
    gap = after[0] - before[0]
    if gap <= 0 or gap > max_gap_ns:
        return None, 'gap', gap
    alpha = (stamp_ns - before[0]) / gap
    position = tuple((1 - alpha) * x + alpha * y for x, y in zip(before[1], after[1]))
    orientation = slerp(before[2], after[2], alpha)
    cov_a, cov_b = before[3], after[3]
    covariance = None if cov_a is None or cov_b is None else tuple(
        (1 - alpha) * x + alpha * y for x, y in zip(cov_a, cov_b))
    return (position, orientation, covariance), 'ok', gap


class Zstd:
    def __init__(self):
        name = ctypes.util.find_library('zstd')
        if not name:
            raise RuntimeError('libzstd is required for the compact offline cache')
        self.lib = ctypes.CDLL(name)
        self.lib.ZSTD_compressBound.argtypes = (ctypes.c_size_t,)
        self.lib.ZSTD_compressBound.restype = ctypes.c_size_t
        self.lib.ZSTD_compress.argtypes = (ctypes.c_void_p, ctypes.c_size_t,
                                           ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int)
        self.lib.ZSTD_compress.restype = ctypes.c_size_t
        self.lib.ZSTD_isError.argtypes = (ctypes.c_size_t,)
        self.lib.ZSTD_isError.restype = ctypes.c_uint

    def compress(self, payload):
        size = len(payload)
        buffer = ctypes.create_string_buffer(self.lib.ZSTD_compressBound(size))
        source = ctypes.create_string_buffer(bytes(payload))
        written = self.lib.ZSTD_compress(buffer, len(buffer), source, size, 1)
        if self.lib.ZSTD_isError(written):
            raise RuntimeError('zstd compression failed')
        return buffer.raw[:written]


def open_reader(bag_dir, topics):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    available = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    selected = set(topics) & set(available)
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(selected)))
    return reader, {name: available[name] for name in selected}


def ros_messages(bag_dir, topics):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader, selected = open_reader(bag_dir, topics)
    types = {topic: get_message(kind) for topic, kind in selected.items()}
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        yield topic, deserialize_message(data, types[topic]), int(bag_ns)


def pose_record(topic, message, bag_ns):
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    return {'topic': topic, 'bag_ns': bag_ns, 'header_ns': ns(message.header.stamp),
            'frame_id': message.header.frame_id, 'child_frame_id': message.child_frame_id,
            'translation': vec_of(p), 'quaternion': quat_of(q),
            'pose_covariance': list(message.pose.covariance),
            'twist': {'linear': vec_of(message.twist.twist.linear),
                      'angular': vec_of(message.twist.twist.angular)},
            'twist_covariance': list(message.twist.covariance)}


def tf_record(topic, transform, bag_ns):
    p = transform.transform.translation
    q = transform.transform.rotation
    return {'topic': topic, 'bag_ns': bag_ns, 'header_ns': ns(transform.header.stamp),
            'parent': transform.header.frame_id, 'child': transform.child_frame_id,
            'translation': vec_of(p), 'quaternion': quat_of(q)}


def map_record(message, bag_ns):
    origin = message.info.origin
    raw = bytes(value & 255 for value in message.data)
    orientation = quat_of(origin.orientation)
    origin_yaw = math.atan2(2 * (orientation[3] * orientation[2] +
                                  orientation[0] * orientation[1]),
                            1 - 2 * (orientation[1] ** 2 + orientation[2] ** 2))
    info = {'bag_ns': bag_ns, 'header_ns': ns(message.header.stamp),
            'frame_id': message.header.frame_id,
            'map_load_time_ns': ns(message.info.map_load_time),
            'resolution': message.info.resolution, 'width': message.info.width,
            'height': message.info.height, 'origin_translation': vec_of(origin.position),
            'origin_quaternion': orientation, 'origin_yaw': origin_yaw,
            'occupancy_sha256': hashlib.sha256(raw).hexdigest(),
            'occupancy_encoding': 'int8 row-major; -1 unknown, 0 free, 100 occupied'}
    if len(raw) != info['width'] * info['height']:
        raise ValueError('invalid map occupancy size')
    return info, raw


def first_pass(bag_dir, temporary):
    odom = []
    amcl = []
    dynamic = defaultdict(list)
    static = {}
    maps = []
    counts = Counter()
    with (temporary / 'poses.jsonl').open('w') as poses_out, \
         (temporary / 'tf.jsonl').open('w') as tf_out, \
         (temporary / 'amcl_pose.jsonl').open('w') as amcl_out:
        for topic, message, bag_ns in ros_messages(bag_dir, FIRST_PASS_TOPICS):
            counts[topic] += 1
            if topic in POSE_TOPICS:
                item = pose_record(topic, message, bag_ns)
                poses_out.write(json.dumps(item, separators=(',', ':')) + '\n')
                if topic == '/odom' and item['frame_id'] == 'odom' and item['child_frame_id'] == 'base_footprint':
                    covariance = (item['pose_covariance'][0], item['pose_covariance'][7],
                                  item['pose_covariance'][35])
                    odom.append((item['header_ns'], item['translation'], item['quaternion'], covariance))
            elif topic in ('/tf', '/tf_static'):
                for transform in message.transforms:
                    item = tf_record(topic, transform, bag_ns)
                    tf_out.write(json.dumps(item, separators=(',', ':')) + '\n')
                    edge = (item['parent'], item['child'])
                    transform_value = (item['translation'], item['quaternion'])
                    if topic == '/tf_static':
                        if edge in static and static[edge] != transform_value:
                            raise ValueError('conflicting static TF edge: ' + str(edge))
                        static[edge] = transform_value
                    else:
                        dynamic[edge].append((item['header_ns'], *transform_value, None))
            elif topic == '/amcl_pose':
                covariance = list(message.pose.covariance)
                item = {'topic': topic, 'bag_ns': bag_ns,
                        'header_ns': ns(message.header.stamp),
                        'frame_id': message.header.frame_id,
                        'translation': vec_of(message.pose.pose.position),
                        'quaternion': quat_of(message.pose.pose.orientation),
                        'pose_covariance': covariance}
                amcl_out.write(json.dumps(item, separators=(',', ':')) + '\n')
                if item['frame_id'] == 'map':
                    amcl.append((item['header_ns'], covariance[0], covariance[7], covariance[35]))
            elif topic == '/map':
                maps.append(map_record(message, bag_ns))
    odom.sort(key=lambda x: x[0])
    amcl.sort(key=lambda x: x[0])
    for samples in dynamic.values():
        samples.sort(key=lambda x: x[0])
    if not maps:
        raise ValueError('bag has no /map; cannot compare in map frame')
    signature = {(item[0]['resolution'], item[0]['width'], item[0]['height'],
                  item[0]['origin_translation'], item[0]['origin_quaternion'],
                  item[0]['occupancy_sha256']) for item in maps}
    if len(signature) != 1:
        raise ValueError('bag contains changing /map; cache v1 requires one static map')
    map_info, map_data = maps[0]
    (temporary / 'map.bin').write_bytes(map_data)
    (temporary / 'map.json').write_text(json.dumps(map_info, indent=2) + '\n')
    (temporary / 'map.tsv').write_text(
        '\t'.join(('width', 'height', 'resolution', 'origin_x', 'origin_y', 'origin_z',
                   'origin_qx', 'origin_qy', 'origin_qz', 'origin_qw', 'origin_yaw',
                   'frame_id')) + '\n' +
        '\t'.join(map(str, (map_info['width'], map_info['height'], map_info['resolution'],
                            *map_info['origin_translation'], *map_info['origin_quaternion'],
                            map_info['origin_yaw'], map_info['frame_id']))) + '\n')
    return odom, amcl, dynamic, static, map_info, counts


def sensor_tf(static, dynamic, frame, stamp_ns):
    if frame == 'base_footprint':
        return ((0., 0., 0.), (0., 0., 0., 1.)), 'ok', 0
    adjacency = defaultdict(list)
    for edge, transform in static.items():
        adjacency[edge[0]].append((edge[1], transform, 0))
        adjacency[edge[1]].append((edge[0], inverse(transform), 0))
    for edge, samples in dynamic.items():
        # Localization edges are handled only by the explicitly selected pose path.
        if edge in (('map', 'odom'), ('map', 'lio_odom'), ('lio_odom', 'odom'),
                    ('odom', 'base_footprint')):
            continue
        result, status, gap = interpolate(samples, stamp_ns, SENSOR_TF_MAX_GAP_NS)
        if status == 'ok':
            transform = (result[0], result[1])
            adjacency[edge[0]].append((edge[1], transform, gap))
            adjacency[edge[1]].append((edge[0], inverse(transform), gap))
    queue = [('base_footprint', ((0., 0., 0.), (0., 0., 0., 1.)), 0)]
    seen = {'base_footprint'}
    for parent, accumulated, largest_gap in queue:
        for child, transform, gap in adjacency[parent]:
            if child in seen:
                continue
            combined = compose(accumulated, transform)
            next_gap = max(largest_gap, gap)
            if child == frame:
                return combined, 'ok', next_gap
            seen.add(child)
            queue.append((child, combined, next_gap))
    return None, 'missing_sensor_tf', None


def nearest_amcl_cov(samples, stamp_ns):
    if not samples:
        return None, None
    index = bisect_left(samples, (stamp_ns,))
    candidates = samples[max(0, index - 1):min(len(samples), index + 1)]
    best = min(candidates, key=lambda item: abs(item[0] - stamp_ns))
    age = abs(best[0] - stamp_ns)
    return (best[1:] if age <= AMCL_COV_MAX_AGE_NS else None), age


def derived_pose(stamp_ns, cloud_frame, odom, amcl, dynamic, static):
    blank = {'pose_source': 'recorded_odom+map_to_odom_AMCL',
             'map_from_sensor': None, 'pose_var': None, 'odom_gap_ns': None,
             'map_tf_gap_ns': None, 'sensor_tf_gap_ns': None, 'amcl_cov_age_ns': None}
    own, status, odom_gap = interpolate(odom, stamp_ns, ODOM_MAX_GAP_NS)
    blank['odom_gap_ns'] = odom_gap
    if status != 'ok':
        return dict(blank, pose_status='odom_' + status)
    correction, status, map_gap = interpolate(dynamic.get(('map', 'odom'), []),
                                               stamp_ns, MAP_TF_MAX_GAP_NS)
    blank['map_tf_gap_ns'] = map_gap
    if status != 'ok':
        return dict(blank, pose_status='map_odom_' + status)
    sensor, status, sensor_gap = sensor_tf(static, dynamic, cloud_frame, stamp_ns)
    blank['sensor_tf_gap_ns'] = sensor_gap
    if status != 'ok':
        return dict(blank, pose_status=status)
    map_base = compose((correction[0], correction[1]), (own[0], own[1]))
    map_sensor = compose(map_base, sensor)
    amcl_cov, age = nearest_amcl_cov(amcl, stamp_ns)
    blank['amcl_cov_age_ns'] = age
    # AMCL covariance describes corrected map pose; odom covariance may add
    # short-term uncertainty. Floors are conservative cache metadata, not truth.
    covariance = tuple(max(own[2][i] if own[2] else 0.,
                           amcl_cov[i] if amcl_cov else 0.,
                           (0.05 ** 2, 0.05 ** 2, math.radians(5) ** 2)[i])
                       for i in range(3))
    return dict(blank, pose_status='ok', map_from_sensor=map_sensor, pose_var=covariance)


def field_layout(message):
    return {'point_step': message.point_step, 'is_bigendian': bool(message.is_bigendian),
            'fields': [{'name': f.name, 'offset': f.offset,
                        'datatype': f.datatype, 'count': f.count} for f in message.fields]}


def field_offset(message, name):
    for field in message.fields:
        if field.name == name and field.count == 1:
            return field.offset
    return -1


def cloud_pass(bag_dir, temporary, odom, amcl, dynamic, static, map_bag_ns,
               cloud_topics, limit):
    zstd = Zstd()
    layouts = []
    layout_ids = {}
    counts = Counter()
    missing = Counter()
    matches = defaultdict(lambda: {'raw': [], 'filtered': []})
    with (temporary / 'clouds.zstbin').open('wb') as binary, \
         (temporary / 'frames.tsv').open('w') as index:
        index.write('\t'.join(FIELDS) + '\n')
        for topic, message, bag_ns in ros_messages(bag_dir, cloud_topics):
            record_index = counts['raw'] + counts['filtered']
            if limit is not None and record_index >= limit:
                break
            kind = TOPICS[topic]
            stamp_ns = ns(message.header.stamp)
            layout = field_layout(message)
            key = json.dumps(layout, sort_keys=True)
            if key not in layout_ids:
                layout_ids[key] = len(layouts)
                layouts.append(layout)
            payload = bytes(message.data)
            compressed = zstd.compress(payload)
            offset = binary.tell()
            binary.write(compressed)
            pose = derived_pose(stamp_ns, message.header.frame_id,
                                odom, amcl, dynamic, static)
            if bag_ns < map_bag_ns:
                pose['pose_status'] = 'map_not_yet_recorded'
                pose['map_from_sensor'] = None
                pose['pose_var'] = None
            if pose['pose_status'] != 'ok':
                missing[pose['pose_status']] += 1
            transform = pose['map_from_sensor']
            location = (*transform[0], *transform[1]) if transform else (None,) * 7
            variance = pose['pose_var'] if pose['pose_var'] else (None,) * 3
            row = (
                record_index, kind, bag_ns, stamp_ns, offset, len(compressed),
                len(payload), message.header.frame_id, message.width, message.height,
                message.point_step, message.row_step, int(message.is_bigendian),
                int(message.is_dense), layout_ids[key],
                *(field_offset(message, name) for name in
                  ('x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp')),
                pose['pose_status'], pose['pose_source'], *location, *variance,
                pose['odom_gap_ns'], pose['map_tf_gap_ns'], pose['sensor_tf_gap_ns'],
                pose['amcl_cov_age_ns'],
            )
            index.write('\t'.join('' if value is None else str(value) for value in row) + '\n')
            matches[stamp_ns][kind].append(record_index)
            counts[kind] += 1
            counts[kind + '_points'] += message.width * message.height
            counts[kind + '_uncompressed_bytes'] += len(payload)
            counts[kind + '_compressed_bytes'] += len(compressed)
    paired = 0
    with (temporary / 'pairs.tsv').open('w') as pair_out:
        pair_out.write('header_ns\traw_record_index\tfiltered_record_index\n')
        for stamp_ns, pair in sorted(matches.items()):
            for raw_index, filtered_index in zip(pair['raw'], pair['filtered']):
                pair_out.write(f'{stamp_ns}\t{raw_index}\t{filtered_index}\n')
                paired += 1
    counts['paired_header_stamps'] = paired
    counts['unmatched_raw'] = counts['raw'] - paired
    counts['unmatched_filtered'] = counts['filtered'] - paired
    return layouts, dict(counts), dict(missing)


def source_hashes(bag_dir):
    files = sorted(bag_dir.glob('*.mcap'))
    if not files:
        raise FileNotFoundError('no MCAP files: ' + str(bag_dir))
    sources = []
    for path in files + [bag_dir / 'metadata.yaml']:
        if path.exists():
            sources.append({'name': path.name, 'bytes': path.stat().st_size,
                            'sha256': sha256(path)})
    processing = []
    for path in (Path(__file__), REPOSITORY / 'src/hsl_lidar_filter/src/lidar_filter.cpp',
                 REPOSITORY / 'src/hsl_real/hsl_real/lidar_filter_core.py',
                 REPOSITORY / 'config/lidar_filter.yaml'):
        if path.exists():
            processing.append({'path': str(path.relative_to(REPOSITORY)),
                               'sha256': sha256(path)})
    return sources, processing


def extract(session, args):
    bag_dir = args.root / session / 'bag'
    if not bag_dir.is_dir():
        raise FileNotFoundError(bag_dir)
    destination = args.output / Path(session).name
    print(f'[{session}] hashing inputs', flush=True)
    sources, processing = source_hashes(bag_dir)
    signature = hashlib.sha256(json.dumps({'sources': sources, 'processing': processing,
                                           'cloud_mode': args.cloud_mode,
                                           'limit_clouds': args.limit_clouds},
                                          sort_keys=True).encode()).hexdigest()
    if destination.exists():
        manifest_path = destination / 'manifest.json'
        if manifest_path.exists() and json.loads(manifest_path.read_text()).get('signature') == signature:
            print(f'[{session}] existing matching cache: {destination}', flush=True)
            return
        raise FileExistsError(f'stale/different cache at {destination}; choose a new --output')
    temporary = args.output / (Path(session).name + f'.tmp-{os.getpid()}')
    if temporary.exists():
        raise FileExistsError(temporary)
    temporary.mkdir(parents=True)
    started = time.monotonic()
    try:
        print(f'[{session}] extracting poses, TF and map', flush=True)
        odom, amcl, dynamic, static, map_info, first_counts = first_pass(bag_dir, temporary)
        print(f'[{session}] extracting full-rate clouds', flush=True)
        cloud_topics = list(TOPICS) if args.cloud_mode == 'both' else [
            topic for topic, kind in TOPICS.items() if kind == args.cloud_mode]
        layouts, cloud_counts, missing = cloud_pass(
            bag_dir, temporary, odom, amcl, dynamic, static, map_info['bag_ns'],
            cloud_topics, args.limit_clouds)
        artifact_names = ('clouds.zstbin', 'frames.tsv', 'map.bin', 'map.tsv',
                          'map.json', 'poses.jsonl', 'tf.jsonl', 'amcl_pose.jsonl',
                          'pairs.tsv')
        artifacts = [{'name': name, 'bytes': (temporary / name).stat().st_size,
                      'sha256': sha256(temporary / name)} for name in artifact_names]
        manifest = {
            'schema': 'real-detector-cache-v1', 'signature': signature,
            'session': session, 'input_root': str(args.root), 'bag_dir': str(bag_dir),
            'sources': sources, 'processing_sources': processing,
            'cloud_mode': args.cloud_mode, 'truncated': args.limit_clouds is not None,
            'cloud_limit': args.limit_clouds,
            'artifacts': artifacts,
            'cloud_encoding': 'each PointCloud2.data payload is one independent zstd frame in clouds.zstbin; offsets and exact CDR-deserialized layout in frames.tsv',
            'frame_order': 'rosbag2_py.SequentialReader.read_next() order, no skipping except explicit --limit-clouds',
            'point_time': 'original FLOAT64 timestamp field bytes are retained exactly; no conversion or deskew',
            'layouts': layouts, 'map': map_info,
            'pose_policy': {
                'selected': 'map->odom TF from recorded AMCL composed with recorded /odom odom->base_footprint and sensor TF; at cloud header stamp',
                'interpolation': 'strict two-sided translation linear; quaternion shortest-arc slerp; exact stamp exact sample; no extrapolation',
                'odom_max_bracket_ns': ODOM_MAX_GAP_NS,
                'map_to_odom_max_bracket_ns': MAP_TF_MAX_GAP_NS,
                'sensor_tf_max_bracket_ns': SENSOR_TF_MAX_GAP_NS,
                'missing': 'pose_status records missing/outside/oversized gap; blank map_from_sensor; never latest-pose fallback',
                'map_availability': 'clouds preceding the recorded /map by bag timestamp have no usable map pose',
                'covariance': 'max of odom diagonal, nearest recorded /amcl_pose diagonal within 0.5 s, floors 0.05 m x/y and 5 deg yaw; approximate metadata, not localization truth',
                'amcl_cov_max_age_ns': AMCL_COV_MAX_AGE_NS,
                'other_pose_topics': 'saved verbatim as separate provenance in poses.jsonl; old FAST-LIO and /navigation/self never substitute for selected pose',
                'tf_edges': 'saved individually in tf.jsonl with parent/child and topic; no localization edge mixing',
            },
            'counts': {'first_pass_topics': dict(first_counts), 'clouds': cloud_counts,
                       'missing_pose': missing, 'odom_samples': len(odom),
                       'amcl_cov_samples': len(amcl),
                       'dynamic_edges': {f'{a}->{b}': len(rows) for (a, b), rows in dynamic.items()},
                       'static_edges': [f'{a}->{b}' for a, b in static]},
            'elapsed_wall_s': time.monotonic() - started,
        }
        (temporary / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        temporary.rename(destination)
        print(json.dumps({'cache': str(destination), 'clouds': cloud_counts,
                          'missing_pose': missing, 'elapsed_wall_s': manifest['elapsed_wall_s']}),
              flush=True)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT,
                        help='read-only root of the real bag sessions')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--session', action='append', choices=SESSIONS,
                        help='repeat to select sessions; default all listed sessions')
    parser.add_argument('--cloud-mode', choices=('both', 'filtered', 'raw'), default='both')
    parser.add_argument('--limit-clouds', type=int,
                        help='development smoke test only; manifest will be marked truncated')
    args = parser.parse_args()
    if args.limit_clouds is not None and args.limit_clouds <= 0:
        parser.error('--limit-clouds must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    for session in args.session or SESSIONS:
        extract(session, args)


if __name__ == '__main__':
    main()
