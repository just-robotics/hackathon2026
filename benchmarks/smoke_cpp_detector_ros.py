#!/usr/bin/env python3
"""ROS interface smoke from original real-bag clouds and timestamped derived pose.

This is a bag replay, not synthetic LiDAR or a localization accuracy test. It
publishes exact cached PointCloud2 payloads, the recorded static sensor TF,
and timestamped derived base pose. Map-backed caches replay their original map;
the explicit local cache schema remains an unanchored, mapless odom ablation.
"""
import argparse
import csv
import hashlib
import json
import math
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from real_detector_cache import REPOSITORY, compose, inverse
from real_detector_local_cache import filter_parameters
from replay_cached_baseline import Zstd, cloud_from_cache, occupancy_from_cache, sha256


NS = 1_000_000_000


def validate_cache(cache, manifest, source_bag=None):
    """Check bytes and preprocessing provenance before publishing any data."""
    schema = manifest['schema']
    if schema not in ('real-detector-cache-v1', 'real-detector-cache-local-v1'):
        raise ValueError('unknown real detector cache schema')
    mapless = schema == 'real-detector-cache-local-v1'
    if mapless and (manifest.get('world_frame') != 'odom' or
                    manifest.get('map_available') is not False or manifest.get('map') is not None):
        raise ValueError('local cache must explicitly identify unanchored odom without map')
    if manifest.get('truncated'):
        raise ValueError('a complete real cache is required for this replay')
    processing = manifest['processing_sources']
    for item in processing:
        source = REPOSITORY / item['path']
        if not source.is_file() or sha256(source) != item['sha256']:
            raise ValueError(f"preprocessing source no longer matches cache: {item['path']}")
    required_filter_sources = {
        'src/hsl_lidar_filter/src/lidar_filter.cpp',
        'src/hsl_real/hsl_real/lidar_filter_core.py', 'config/lidar_filter.yaml'}
    if not required_filter_sources <= {item['path'] for item in processing}:
        raise ValueError('cache lacks production filter source/configuration hashes')
    signature_input = {'sources': manifest['sources'], 'processing': processing}
    if mapless:
        parameters = filter_parameters()
        if parameters != manifest['filter']['parameters']:
            raise ValueError('local cache filter parameters differ from current production profile')
        signature_input.update(filter_parameters=parameters,
                               pose_policy='strict_recorded_odom_local_v1')
    else:
        signature_input.update(cloud_mode=manifest['cloud_mode'],
                               limit_clouds=manifest['cloud_limit'])
    signature = hashlib.sha256(json.dumps(signature_input, sort_keys=True).encode()).hexdigest()
    if signature != manifest['signature']:
        raise ValueError('cache source/filter signature does not match its manifest')
    artifacts = {item['name']: item for item in manifest['artifacts']}
    required = ('frames.tsv', 'clouds.zstbin', 'tf.jsonl')
    required += ('filter_stats.jsonl',) if mapless else ('map.bin', 'map.json')
    for name in required:
        item = artifacts[name]
        path = cache / name
        if path.stat().st_size != item['bytes'] or sha256(path) != item['sha256']:
            raise ValueError(f'{name} no longer matches cache manifest')
    # Old source bags may have been deleted by the user. Their recorded hashes
    # remain part of the checked signature; verify original files when mounted.
    # A cache may have been extracted under a different container mount. An
    # explicitly selected live bag must be verified at its current path.
    bag_dir = source_bag.resolve() if source_bag is not None else Path(manifest['bag_dir'])
    if source_bag is not None and not bag_dir.is_dir():
        raise ValueError(f'original source bag is unavailable: {bag_dir}')
    checked_sources = 0
    if bag_dir.is_dir():
        for item in manifest['sources']:
            source = bag_dir / item['name']
            if source.stat().st_size != item['bytes'] or sha256(source) != item['sha256']:
                raise ValueError(f"original bag source differs from cache: {item['name']}")
            checked_sources += 1
    return dict(cache_signature=signature, cache_manifest_sha256=sha256(cache / 'manifest.json'),
                cache_artifacts_verified=True, preprocessing_sources_verified=True,
                filter_signature_verified=True, original_source_files_verified=checked_sources,
                original_source_available=bag_dir.is_dir(),
                original_source_path=str(bag_dir),
                recorded_source_path=manifest['bag_dir']), mapless


def message_stamp_ns(message):
    return int(message.header.stamp.sec) * NS + int(message.header.stamp.nanosec)


def stamp(message, nanoseconds):
    message.sec = nanoseconds // NS
    message.nanosec = nanoseconds % NS


def static_sensor_pose(cache, frame):
    static = {}
    for line in (cache / 'tf.jsonl').open():
        item = json.loads(line)
        if item['topic'] == '/tf_static':
            static[(item['parent'], item['child'])] = (
                tuple(item['translation']), tuple(item['quaternion']))
    if frame == 'base_footprint':
        return ((0., 0., 0.), (0., 0., 0., 1.))
    if ('base_footprint', frame) in static:
        return static[('base_footprint', frame)]
    return compose(static[('base_footprint', 'base_link')], static[('base_link', frame)])


def base_pose(row, base_from_sensor, world_frame='map'):
    map_from_sensor = (
        tuple(float(row[f'map_sensor_{axis}']) for axis in 'xyz'),
        tuple(float(row[f'map_sensor_q{axis}']) for axis in 'xyzw'))
    position, quaternion = compose(map_from_sensor, inverse(base_from_sensor))
    message = Odometry()
    stamp(message.header.stamp, int(row['header_ns']))
    message.header.frame_id = world_frame
    message.child_frame_id = 'base_footprint'
    message.pose.pose.position.x, message.pose.pose.position.y, \
        message.pose.pose.position.z = position
    message.pose.pose.orientation.x, message.pose.pose.orientation.y, \
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = quaternion
    message.pose.covariance[0] = float(row['pose_var_x'])
    message.pose.covariance[7] = float(row['pose_var_y'])
    message.pose.covariance[35] = float(row['pose_var_yaw'])
    return message


class Replay(Node):
    def __init__(self, mapless=False):
        super().__init__('real_bag_cpp_detector_smoke')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = None if mapless else self.create_publisher(OccupancyGrid, '/map', latched)
        self.tf_pub = self.create_publisher(TFMessage, '/tf_static', latched)
        self.dynamic_tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.pose_pub = self.create_publisher(Odometry, '/cached/self_pose', 100)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/sensing/lidar/points_filtered', qos_profile_sensor_data)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odometry = []
        self.visibility = []
        self.diagnostics = []
        self.markers = []
        self.create_subscription(Odometry, '/opponent/odom', self.odometry.append, 100)
        self.create_subscription(Bool, '/navigation/opponent_visible',
                                 self.visibility.append, 100)
        self.create_subscription(String, '/navigation/detector_diagnostics',
                                 lambda m: self.diagnostics.append(json.loads(m.data)), 100)
        self.create_subscription(MarkerArray, '/opponent/markers', self.markers.append, 100)

    def clock(self, nanoseconds):
        message = Clock()
        stamp(message.clock, nanoseconds)
        self.clock_pub.publish(message)


def validate_outputs(node, sent_rows, world_frame, *, complete):
    input_stamps = {int(row['header_ns']) for row in sent_rows}
    output_stamps = {message_stamp_ns(message) for message in node.odometry}
    assert output_stamps <= input_stamps, 'Odometry was restamped'
    assert len(output_stamps) == len(node.odometry), 'duplicate measured Odometry timestamp'
    for message in node.odometry:
        assert message.header.frame_id == world_frame
        assert message.child_frame_id == 'opponent_tracking_frame'
        assert message.pose.pose.orientation.w == 1.0
        assert message.pose.covariance[35] >= 1e4
        assert all(math.isfinite(value) for value in (
            message.pose.pose.position.x, message.pose.pose.position.y,
            message.twist.twist.linear.x, message.twist.twist.linear.y))
        assert message.twist.covariance[0] > 0 and message.twist.covariance[7] > 0
        if message.twist.covariance[0] >= 1e4:
            assert message.twist.twist.linear.x == message.twist.twist.linear.y == 0.
    # Each processed-result diagnostic carries candidate_count, unlike timer
    # status messages. cloud_stamp_s refers to the actual processed input.
    processed = [item for item in node.diagnostics if 'candidate_count' in item]
    diagnostic_stamps = {round(item['cloud_stamp_s'] * NS): item for item in processed}
    gated_stamps = set()
    for item in processed:
        if item.get('status') == 'map_alignment_inconsistent':
            assert item.get('visible') is False and item.get('measurement') is False
            gated_stamps.add(round(item['cloud_stamp_s'] * NS))
    # Diagnostics use double seconds: allow rounding of a nanosecond stamp.
    is_gated = lambda stamp_ns: any(abs(stamp_ns - gate) <= 1000 for gate in gated_stamps)
    assert not any(is_gated(stamp_ns) for stamp_ns in output_stamps), 'Odometry on gated scan'
    marker_batches = {}
    for batch in node.markers:
        for marker in batch.markers:
            assert marker.header.frame_id == world_frame
            stamp_ns = message_stamp_ns(marker)
            # Timeout deletion can use the replay clock instead of cloud stamp.
            if marker.action == Marker.ADD:
                assert stamp_ns in input_stamps, 'marker ADD was restamped'
                if marker.ns == 'selected_measurement':
                    assert stamp_ns in output_stamps, 'selected marker lacks measured Odometry'
                assert not is_gated(stamp_ns), 'marker ADD on gated scan'
            marker_batches.setdefault(stamp_ns, []).append(marker)
    active = set()
    for stamp_ns in sorted(input_stamps | marker_batches.keys()):
        for marker in marker_batches.get(stamp_ns, ()):
            key = (marker.ns, marker.id)
            if marker.action == Marker.ADD:
                active.add(key)
            elif marker.action == Marker.DELETE:
                active.discard(key)
            elif marker.action == Marker.DELETEALL:
                active.clear()
        if complete and is_gated(stamp_ns):
            assert not active, 'old markers were not deleted on gated scan'
    if complete:
        assert node.diagnostics, 'no detector diagnostics received'
        last = node.diagnostics[-1]
        assert last['processed_clouds'] == len(sent_rows), 'not every replay scan was processed'
        assert last['replaced_clouds'] == last['discarded_clouds'] == 0, 'replay scans dropped/replaced'
        assert len(processed) == len(sent_rows), 'missing per-scan result diagnostics'
        assert len(diagnostic_stamps) == len(sent_rows), 'duplicate processed-result diagnostics'
        assert sum(item['measurement'] for item in processed) == len(node.odometry)
        assert sum(message.data for message in node.visibility) == len(node.odometry)
        assert len(node.visibility) >= len(sent_rows), 'missing visibility messages'
        if node.odometry:
            assert node.markers, 'no detector markers received for measured target'
            selected = {message_stamp_ns(marker) for batch in node.markers
                        for marker in batch.markers
                        if marker.action == Marker.ADD and marker.ns == 'selected_measurement'}
            assert selected == output_stamps, 'missing selected-measurement markers'
    return dict(output_stamps_match_real_clouds=True, gated_scans=len(gated_stamps),
                marker_batches=len(node.markers), verification_completed=complete)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('cache', type=Path)
    parser.add_argument('--source-bag', type=Path,
                        help='Verify the original MCAP at its current path when the cache moved')
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--from-seconds', type=float, default=161.)
    parser.add_argument('--count', type=int, default=85, help='0 replays all remaining scans')
    parser.add_argument('--expect', choices=('present', 'absent', 'any'), default='present')
    parser.add_argument('--rate', type=float, default=1., help='Playback rate relative to bag timestamps')
    parser.add_argument('--log', type=Path, default=Path('/tmp/opponent_cpp_smoke.log'))
    parser.add_argument('--rviz-config', type=Path,
                        help='Open RViz for this one real-bag replay')
    parser.add_argument('--rviz-log', type=Path, default=Path('/tmp/opponent_cpp_rviz.log'))
    parser.add_argument('--hold-seconds', type=float, default=0.,
                        help='Keep final RViz scene; -1 waits for window close or Ctrl+C')
    args = parser.parse_args()
    if args.count < 0:
        parser.error('--count must be >= 0')
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('--rate must be finite and > 0')
    if not math.isfinite(args.from_seconds) or args.from_seconds < 0:
        parser.error('--from-seconds must be finite and >= 0')
    if not math.isfinite(args.hold_seconds) or (args.hold_seconds < 0 and args.hold_seconds != -1):
        parser.error('--hold-seconds must be >= 0 or -1')
    cache = args.cache.resolve()
    manifest = json.loads((cache / 'manifest.json').read_text())
    provenance, mapless = validate_cache(cache, manifest, args.source_bag)
    world_frame = 'odom' if mapless else 'map'
    with (cache / 'frames.tsv').open() as stream:
        all_rows = [row for row in csv.DictReader(stream, delimiter='\t')
                    if row['topic'] == 'filtered' and row['pose_status'] == 'ok']
    if not all_rows:
        raise ValueError('no filtered real scans with valid timestamped pose')
    start_ns = int(all_rows[0]['header_ns']) + round(args.from_seconds * NS)
    rows = [row for row in all_rows if int(row['header_ns']) >= start_ns]
    if args.count:
        rows = rows[:args.count]
    if not rows or (args.count and len(rows) < args.count):
        raise ValueError('not enough cached real scans in selected window')
    frames = {row['frame_id'] for row in rows}
    if len(frames) != 1:
        raise ValueError(f'multiple sensor frames: {frames}')
    frame = frames.pop()
    base_from_sensor = static_sensor_pose(cache, frame)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Replay(mapless)
    rviz = None
    rviz_log = None
    temporary_rviz_config = None
    sent_rows = []
    stopped_early = None
    command = [str(args.executable), '--ros-args', '--params-file', str(args.profile),
               '-p', 'use_sim_time:=true', '-p', 'pose_topic:=/cached/self_pose']
    if mapless:
        command += ['-p', 'world_frame:=odom', '-p', 'allow_mapless:=true']
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            try:
                if args.rviz_config:
                    config_path = args.rviz_config.resolve()
                    if mapless:
                        import yaml
                        config = yaml.safe_load(config_path.read_text())
                        config['Visualization Manager']['Global Options']['Fixed Frame'] = world_frame
                        temporary_rviz_config = tempfile.TemporaryDirectory(prefix='real-detector-rviz-')
                        config_path = Path(temporary_rviz_config.name) / 'local_odom.rviz'
                        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
                    args.rviz_log.parent.mkdir(parents=True, exist_ok=True)
                    rviz_log = args.rviz_log.open('w')
                    rviz = subprocess.Popen(
                        ['rviz2', '-d', str(config_path), '--ros-args', '-p', 'use_sim_time:=true'],
                        stdout=rviz_log, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic() + 8.
                required_publishers = [node.tf_pub, node.pose_pub, node.cloud_pub]
                if node.map_pub is not None:
                    required_publishers.append(node.map_pub)
                while time.monotonic() < deadline and process.poll() is None:
                    rclpy.spin_once(node, timeout_sec=.05)
                    if rviz is not None and rviz.poll() is not None:
                        stopped_early = 'rviz_closed'
                        break
                    if all(pub.get_subscription_count() for pub in required_publishers):
                        break
                else:
                    raise RuntimeError('C++ detector did not subscribe; see ' + str(args.log))

                if stopped_early is None:
                    first_ns = int(rows[0]['header_ns'])
                    node.clock(first_ns)
                    if node.map_pub is not None:
                        node.map_pub.publish(occupancy_from_cache(cache))
                    sensor_tf = TransformStamped()
                    stamp(sensor_tf.header.stamp, first_ns)
                    sensor_tf.header.frame_id = 'base_footprint'
                    sensor_tf.child_frame_id = frame
                    sensor_tf.transform.translation.x, sensor_tf.transform.translation.y, \
                        sensor_tf.transform.translation.z = base_from_sensor[0]
                    sensor_tf.transform.rotation.x, sensor_tf.transform.rotation.y, \
                        sensor_tf.transform.rotation.z, sensor_tf.transform.rotation.w = base_from_sensor[1]
                    node.tf_pub.publish(TFMessage(transforms=[sensor_tf]))
                    for _ in range(10):
                        rclpy.spin_once(node, timeout_sec=.03)

                    decoder = Zstd()
                    with (cache / 'clouds.zstbin').open('rb') as cloud_file:
                        for index, row in enumerate(rows):
                            if rviz is not None and rviz.poll() is not None:
                                stopped_early = 'rviz_closed'
                                break
                            if process.poll() is not None:
                                raise RuntimeError('C++ detector exited; see ' + str(args.log))
                            header_ns = int(row['header_ns'])
                            node.clock(header_ns + 20_000_000)
                            pose = base_pose(row, base_from_sensor, world_frame)
                            node.pose_pub.publish(pose)
                            world_to_base = TransformStamped()
                            world_to_base.header = pose.header
                            world_to_base.child_frame_id = 'base_footprint'
                            world_to_base.transform.translation.x = pose.pose.pose.position.x
                            world_to_base.transform.translation.y = pose.pose.pose.position.y
                            world_to_base.transform.translation.z = pose.pose.pose.position.z
                            world_to_base.transform.rotation = pose.pose.pose.orientation
                            node.dynamic_tf_pub.publish(TFMessage(transforms=[world_to_base]))
                            cloud_file.seek(int(row['offset']))
                            encoded = cloud_file.read(int(row['compressed_size']))
                            payload = decoder.decompress(encoded, int(row['uncompressed_size']))
                            layout = manifest['layouts'][int(row['layout_id'])]
                            node.cloud_pub.publish(cloud_from_cache(row, layout, payload))
                            sent_rows.append(row)
                            next_ns = (int(rows[index + 1]['header_ns']) if index + 1 < len(rows)
                                       else header_ns + 100_000_000)
                            until = time.monotonic() + max(0., (next_ns - header_ns) * 1e-9 / args.rate)
                            while time.monotonic() < until:
                                if rviz is not None and rviz.poll() is not None:
                                    stopped_early = 'rviz_closed'
                                    break
                                rclpy.spin_once(node, timeout_sec=min(.01, max(0., until - time.monotonic())))
                            if stopped_early is not None:
                                break
            except (KeyboardInterrupt, ExternalShutdownException):
                stopped_early = 'interrupted'
            if rclpy.ok():
                try:
                    for _ in range(40):
                        rclpy.spin_once(node, timeout_sec=.01)
                except (KeyboardInterrupt, ExternalShutdownException):
                    stopped_early = 'interrupted'

            complete = stopped_early is None and len(sent_rows) == len(rows)
            checks = validate_outputs(node, sent_rows, world_frame, complete=complete)
            report = dict(source=str(cache), world_frame=world_frame, mapless_local_odom=mapless,
                          selected_frames=len(rows), frames=len(sent_rows), rate=args.rate,
                          partial=not complete, stop_reason=stopped_early,
                          measured_odometry=len(node.odometry), visible_true=sum(m.data for m in node.visibility),
                          diagnostic_statuses=sorted({d.get('status') for d in node.diagnostics}),
                          last_diagnostics=node.diagnostics[-1] if node.diagnostics else {},
                          run_log=str(args.log), **provenance, **checks)
            print(json.dumps(report, indent=2), flush=True)
            if complete and args.expect == 'present' and not node.odometry:
                raise AssertionError('No measured odometry published for labeled visible robot window')
            if complete and args.expect == 'absent' and (node.odometry or any(m.data for m in node.visibility)):
                raise AssertionError('Measured odometry or visible=true in robot-free window')
            if complete and rviz is not None:
                deadline = (None if args.hold_seconds == -1 else time.monotonic() + args.hold_seconds)
                try:
                    while rviz.poll() is None and (deadline is None or time.monotonic() < deadline):
                        rclpy.spin_once(node, timeout_sec=.05)
                except (KeyboardInterrupt, ExternalShutdownException):
                    pass
        finally:
            if rviz is not None and rviz.poll() is None:
                rviz.send_signal(signal.SIGINT)
                try:
                    rviz.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    rviz.kill()
                    rviz.wait()
            if rviz_log is not None:
                rviz_log.close()
            if temporary_rviz_config is not None:
                temporary_rviz_config.cleanup()
            process.send_signal(signal.SIGINT) if process.poll() is None else None
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
