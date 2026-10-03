#!/usr/bin/env python3
"""Cache raw-only real Livox bags in their *unanchored odom* frame.

The five short recordings have neither /map nor map->odom nor a recorded
filtered cloud. This is an explicitly separate preprocessing variant: keep
raw PointCloud2 bytes and reconstruct the production filter's output with its
field-preserving Python oracle. Never label these coordinates as map poses.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time

import real_detector_cache as base


REPOSITORY = Path(__file__).resolve().parents[1]
LOCAL_SESSIONS = (
    'recordings_last8/20261003T095154.890891Z-bag',
    'recordings_last8/20261003T095433.714618Z-bag',
    'recordings_last8/20261003T095724.528830Z-bag',
    'recordings_last8/20261003T095806.834495Z-bag',
    'recordings_last8/20261003T095829.438480Z-bag',
)
FILTER_INPUTS = (
    'src/hsl_lidar_filter/src/lidar_filter.cpp',
    'src/hsl_real/hsl_real/lidar_filter.py',
    'src/hsl_real/hsl_real/lidar_filter_core.py',
    'config/lidar_filter.yaml',
)


def input_hashes(bag_dir: Path):
    source = []
    files = sorted(bag_dir.glob('*.mcap'))
    if not files:
        raise FileNotFoundError(f'no MCAP in {bag_dir}')
    for path in files + [bag_dir / 'metadata.yaml']:
        if not path.exists():
            raise FileNotFoundError(path)
        source.append({'name': path.name, 'bytes': path.stat().st_size,
                       'sha256': base.sha256(path)})
    processing = []
    for path in (Path(__file__), REPOSITORY / 'benchmarks/real_detector_cache.py',
                 *(REPOSITORY / name for name in FILTER_INPUTS)):
        processing.append({'path': str(path.relative_to(REPOSITORY)),
                           'sha256': base.sha256(path)})
    return source, processing


def filter_parameters():
    import yaml
    path = REPOSITORY / 'config/lidar_filter.yaml'
    config = yaml.safe_load(path.read_text())
    return config['real_lidar_filter']['ros__parameters']


def first_pass(bag_dir: Path, temporary: Path):
    odom = []
    dynamic = defaultdict(list)
    static = {}
    counts = Counter()
    with (temporary / 'poses.jsonl').open('w') as poses_out, \
         (temporary / 'tf.jsonl').open('w') as tf_out:
        for topic, message, bag_ns in base.ros_messages(bag_dir, {'/odom', '/tf', '/tf_static'}):
            counts[topic] += 1
            if topic == '/odom':
                item = base.pose_record(topic, message, bag_ns)
                poses_out.write(json.dumps(item, separators=(',', ':')) + '\n')
                if item['frame_id'] == 'odom' and item['child_frame_id'] == 'base_footprint':
                    covariance = tuple(item['pose_covariance'][i] for i in (0, 7, 35))
                    odom.append((item['header_ns'], item['translation'],
                                 item['quaternion'], covariance))
            else:
                for transform in message.transforms:
                    item = base.tf_record(topic, transform, bag_ns)
                    tf_out.write(json.dumps(item, separators=(',', ':')) + '\n')
                    edge = item['parent'], item['child']
                    value = item['translation'], item['quaternion']
                    if topic == '/tf_static':
                        if edge in static and static[edge] != value:
                            raise ValueError(f'conflicting static TF {edge}')
                        static[edge] = value
                    else:
                        dynamic[edge].append((item['header_ns'], *value, None))
    if not odom:
        raise ValueError('no odom->base_footprint samples')
    if ('map', 'odom') in dynamic or ('map', 'odom') in static:
        raise ValueError('map->odom is present; use the map-anchored extractor')
    odom.sort(key=lambda item: item[0])
    for samples in dynamic.values():
        samples.sort(key=lambda item: item[0])
    return odom, dynamic, static, dict(counts)


def local_pose(stamp_ns, cloud_frame, odom, dynamic, static):
    own, status, odom_gap = base.interpolate(odom, stamp_ns, base.ODOM_MAX_GAP_NS)
    if status != 'ok':
        return None, None, 'odom_' + status, odom_gap, None
    sensor, status, sensor_gap = base.sensor_tf(static, dynamic, cloud_frame, stamp_ns)
    if status != 'ok':
        return None, None, status, odom_gap, sensor_gap
    frame_sensor = base.compose((own[0], own[1]), sensor)
    floor = (0.05**2, 0.05**2, math.radians(5)**2)
    covariance = tuple(max(own[2][i] if own[2] else 0., floor[i]) for i in range(3))
    return frame_sensor, covariance, 'ok', odom_gap, sensor_gap


def cloud_pass(bag_dir, temporary, odom, dynamic, static, parameters):
    sys.path.insert(0, str(REPOSITORY / 'src/hsl_real'))
    from hsl_real.lidar_filter import filter_cloud

    reader, topics = base.open_reader(bag_dir, {'/livox/lidar', '/sensing/lidar/points_filtered', '/map'})
    if '/livox/lidar' not in topics or '/sensing/lidar/points_filtered' in topics or '/map' in topics:
        raise ValueError('expected raw-only bag with /livox/lidar and no filtered cloud or /map')
    del reader

    zstd = base.Zstd()
    counts = Counter()
    missing = Counter()
    layouts = []
    layout_ids = {}
    with (temporary / 'clouds.zstbin').open('wb') as binary, \
         (temporary / 'frames.tsv').open('w') as index, \
         (temporary / 'pairs.tsv').open('w') as pairs, \
         (temporary / 'filter_stats.jsonl').open('w') as stats_out:
        index.write('\t'.join(base.FIELDS) + '\n')
        pairs.write('header_ns\traw_record_index\tfiltered_record_index\n')
        for topic, raw, bag_ns in base.ros_messages(bag_dir, {'/livox/lidar'}):
            filtered, filter_stats = filter_cloud(raw, parameters)
            if filtered.header != raw.header or filtered.point_step != raw.point_step \
                    or filtered.fields != raw.fields:
                raise ValueError('offline filter changed header or layout')
            stamp_ns = base.ns(raw.header.stamp)
            stats_out.write(json.dumps({'bag_ns': bag_ns, 'header_ns': stamp_ns,
                                        **filter_stats}, separators=(',', ':')) + '\n')
            pair_index = counts['raw'] + counts['filtered']
            for kind, cloud in (('raw', raw), ('filtered', filtered)):
                record_index = counts['raw'] + counts['filtered']
                layout = base.field_layout(cloud)
                key = json.dumps(layout, sort_keys=True)
                if key not in layout_ids:
                    layout_ids[key] = len(layouts)
                    layouts.append(layout)
                payload = bytes(cloud.data)
                compressed = zstd.compress(payload)
                offset = binary.tell()
                binary.write(compressed)
                pose, variance, status, odom_gap, sensor_gap = local_pose(
                    stamp_ns, cloud.header.frame_id, odom, dynamic, static)
                if status != 'ok':
                    missing[status] += 1
                location = (*pose[0], *pose[1]) if pose else (None,) * 7
                var = variance if variance else (None,) * 3
                row = (
                    record_index, kind, bag_ns, stamp_ns, offset, len(compressed),
                    len(payload), cloud.header.frame_id, cloud.width, cloud.height,
                    cloud.point_step, cloud.row_step, int(cloud.is_bigendian),
                    int(cloud.is_dense), layout_ids[key],
                    *(base.field_offset(cloud, field) for field in
                      ('x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp')),
                    status, 'recorded_odom_local', *location, *var,
                    odom_gap, None, sensor_gap, None,
                )
                index.write('\t'.join('' if value is None else str(value) for value in row) + '\n')
                counts[kind] += 1
                counts[kind + '_points'] += cloud.width * cloud.height
                counts[kind + '_uncompressed_bytes'] += len(payload)
                counts[kind + '_compressed_bytes'] += len(compressed)
            pairs.write(f'{stamp_ns}\t{pair_index}\t{pair_index + 1}\n')
    counts['paired_header_stamps'] = counts['raw']
    counts['unmatched_raw'] = 0
    counts['unmatched_filtered'] = 0
    return layouts, dict(counts), dict(missing)


def extract(session, args):
    bag_dir = args.root / session / 'bag'
    if not bag_dir.is_dir():
        raise FileNotFoundError(bag_dir)
    destination = args.output / Path(session).name
    print(f'[{session}] hashing source and filter', flush=True)
    sources, processing = input_hashes(bag_dir)
    parameters = filter_parameters()
    signature = hashlib.sha256(json.dumps({'sources': sources, 'processing': processing,
                                           'filter_parameters': parameters,
                                           'pose_policy': 'strict_recorded_odom_local_v1'},
                                          sort_keys=True).encode()).hexdigest()
    if destination.exists():
        path = destination / 'manifest.json'
        if path.exists() and json.loads(path.read_text()).get('signature') == signature:
            print(f'[{session}] existing matching cache: {destination}', flush=True)
            return
        raise FileExistsError(f'different cache exists at {destination}')
    temporary = args.output / (Path(session).name + f'.tmp-{os.getpid()}')
    if temporary.exists():
        raise FileExistsError(temporary)
    temporary.mkdir(parents=True)
    started = time.monotonic()
    try:
        odom, dynamic, static, first_counts = first_pass(bag_dir, temporary)
        layouts, counts, missing = cloud_pass(bag_dir, temporary, odom, dynamic,
                                               static, parameters)
        names = ('clouds.zstbin', 'frames.tsv', 'pairs.tsv', 'filter_stats.jsonl',
                 'poses.jsonl', 'tf.jsonl')
        artifacts = [{'name': name, 'bytes': (temporary / name).stat().st_size,
                      'sha256': base.sha256(temporary / name)} for name in names]
        manifest = {
            'schema': 'real-detector-cache-local-v1', 'signature': signature,
            'session': session, 'input_root': str(args.root), 'bag_dir': str(bag_dir),
            'sources': sources, 'processing_sources': processing,
            'artifacts': artifacts,
            'world_frame': 'odom', 'map_available': False,
            'pose_quality': 'local_wheel_odom_unanchored; drift is not corrected',
            'pose_policy': {
                'selected': 'recorded /odom odom->base_footprint composed with base->sensor TF at cloud header timestamp',
                'interpolation': 'strict two-sided translation linear; quaternion shortest-arc slerp; no extrapolation or latest-pose fallback',
                'odom_max_bracket_ns': base.ODOM_MAX_GAP_NS,
                'sensor_tf_max_bracket_ns': base.SENSOR_TF_MAX_GAP_NS,
                'legacy_columns': 'frames.tsv map_sensor_* columns mean odom_from_sensor in this separate schema',
                'covariance': 'recorded /odom diagonal with 0.05 m and 5 degree floor; does not bound accumulated drift',
            },
            'filter': {
                'variant': 'production-field-preserving-python-oracle',
                'parameters': parameters,
                'original_topic': '/livox/lidar',
                'derived_topic': '/sensing/lidar/points_filtered',
                'derived_bag_ns': 'same as original raw message; no actual publication recorded',
                'output_matches_recorded_real_cloud_check': 'see REAL_DETECTOR_CACHE.md',
            },
            'truncated': False, 'cloud_mode': 'both',
            'frame_order': 'for each recorded raw cloud, raw row then derived filtered row; no skipping',
            'point_time': 'original FLOAT64 timestamp bytes retained on kept point records; no deskew',
            'cloud_encoding': 'each PointCloud2.data payload is one independent zstd frame',
            'layouts': layouts, 'map': None,
            'counts': {'first_pass_topics': first_counts, 'clouds': counts,
                       'missing_pose': missing, 'odom_samples': len(odom),
                       'dynamic_edges': {f'{a}->{b}': len(rows) for (a, b), rows in dynamic.items()},
                       'static_edges': [f'{a}->{b}' for a, b in static]},
            'elapsed_wall_s': time.monotonic() - started,
        }
        (temporary / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        temporary.rename(destination)
        print(json.dumps({'cache': str(destination), 'clouds': counts,
                          'missing_pose': missing, 'elapsed_wall_s': manifest['elapsed_wall_s']}),
              flush=True)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=base.DEFAULT_ROOT,
                        help='read-only root of the real bag sessions')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--session', action='append', choices=LOCAL_SESSIONS,
                        help='repeat to select; default is all five local-only recordings')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for session in args.session or LOCAL_SESSIONS:
        extract(session, args)


if __name__ == '__main__':
    main()
