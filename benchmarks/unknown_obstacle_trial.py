#!/usr/bin/env python3
"""Gazebo fixture observer: spawn an unmapped box on an actual published route."""
import argparse
import json
import time
from math import hypot, ceil, isfinite
from pathlib import Path as File

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data
from gazebo_msgs.srv import SpawnEntity
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from hsl_planning.core import Pose2, VoxelWorld, astar
from hsl_planning.node import read_xyz


def box_distance(p, centre, half=0.3):
    return hypot(max(abs(p[0] - centre[0]) - half, 0.0),
                 max(abs(p[1] - centre[1]) - half, 0.0))


def samples(route, step=0.025):
    for a, b in zip(route, route[1:]):
        n = max(1, ceil(hypot(b[0] - a[0], b[1] - a[1]) / step))
        for i in range(n):
            yield (a[0] + (b[0] - a[0]) * i / n,
                   a[1] + (b[1] - a[1]) * i / n)
    if route:
        yield route[-1]


def static_world(grid, arena_bounds):
    world = VoxelWorld(0.15, 0.23)
    walls, free = [], set()
    for i, v in enumerate(grid.data):
        row, col = divmod(i, grid.info.width)
        x = grid.info.origin.position.x + (col + 0.5) * grid.info.resolution
        y = grid.info.origin.position.y + (row + 0.5) * grid.info.resolution
        if v >= 50:
            walls.append((x, y, 0.3))
        elif v == 0:
            free.add(world.cell(x, y))
    bounds = (grid.info.origin.position.x, grid.info.origin.position.y,
              grid.info.origin.position.x + grid.info.width * grid.info.resolution,
              grid.info.origin.position.y + grid.info.height * grid.info.resolution)
    bounds = tuple(arena_bounds)
    world.update(walls, [], None, free, bounds)
    return world, walls, free, bounds


def choose_fixture(grid, route, own, peer, arena_bounds):
    world, walls, free, bounds = static_world(grid, arena_bounds)
    # Candidate positions come from the real global route, never maze coordinates.
    travel = 0.0
    previous = route[0]
    for point in samples(route, 0.15):
        travel += hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
        if not 1.4 <= travel <= 2.6 or hypot(point[0]-own[0], point[1]-own[1]) < 1.2:
            continue
        if peer and hypot(point[0]-peer[0], point[1]-peer[1]) < 1.0:
            continue
        if world.obstacle_clearance(*point) < 0.5 or not world.inside_map(*point, 0.5):
            continue
        boundary = [(point[0]+i*0.03, point[1]+j*0.03, 0.3)
                    for i in range(-10, 11) for j in range(-10, 11)
                    if abs(i)==10 or abs(j)==10]
        alternate = VoxelWorld(0.15, 0.23)
        alternate.update(walls + boundary, [], None, free, bounds)
        detour = astar(alternate, Pose2(*own), Pose2(*route[-1]), max_cells=20000)
        if detour and all(box_distance(p, point) >= 0.23 for p in samples([(q.x,q.y) for q in detour])):
            return point, [[p.x, p.y] for p in detour]
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--arena-bounds', type=float, nargs=4, required=True)
    parser.add_argument('--height', type=float, default=0.8,
                        help='fixture height; XY size remains 0.6 x 0.6 m')
    parser.add_argument('--select-only', action='store_true')
    parser.add_argument('--record-scans', action='store_true',
                        help='save height-filtered map-frame clouds of both robots for offline replay')
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--origin-x', type=float, default=-0.34)
    parser.add_argument('--origin-y', type=float, default=0.4)
    parser.add_argument('--wall-seconds', type=float, default=1200)
    args = parser.parse_args()
    if not isfinite(args.height) or args.height < 0.15:
        parser.error("fixture height must be finite and at least 0.15 m")
    rclpy.init()
    node = Node('unknown_obstacle_trial', parameter_overrides=[Parameter('use_sim_time', value=True)])
    data, records = {}, []
    qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    for key, topic, kind, quality in (
        ('grid','/navigation/known_grid',OccupancyGrid,qos),
        ('own','/navigation/self',Odometry,10),
        ('peer','/opponent/navigation/self',Odometry,10),
        ('path','/navigation/global_path',Path,10),
        ('scan','/navigation/scan',PointCloud2,qos_profile_sensor_data),
        ('peer_scan','/opponent/navigation/scan',PointCloud2,qos_profile_sensor_data),
        ('track','/navigation/opponent',Odometry,10),
        ('status','/navigation/planner_status',String,10),
        ('outcome','/match/outcome',String,qos),
    ):
        node.create_subscription(kind, topic, lambda msg, key=key: data.__setitem__(key,msg), quality)
    client = node.create_client(SpawnEntity, '/spawn_entity')
    report = {'run_id':args.run_id, 'fixture_size_m':[0.6,0.6,args.height],
              'scope':'debug duel; capture must also pass fixture line-of-sight check',
              'fixture_spawned':False, 'samples':records}
    future, centre, last, finished = None, None, -1.0, None
    clouds, last_cloud = [], {}
    if args.record_scans:
        report['cloud_samples'] = clouds
    deadline = time.monotonic() + args.wall_seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = node.get_clock().now().nanoseconds * 1e-9
            if 'outcome' in data:
                outcome = json.loads(data['outcome'].data)
                if outcome.get('run_id') == args.run_id:
                    report['outcome'] = outcome
                    finished = finished or time.monotonic()
                    if time.monotonic() - finished >= 2:
                        break
            if centre is None and all(key in data for key in ('grid','own','peer','path')) and client.service_is_ready():
                route = [(p.pose.position.x,p.pose.position.y) for p in data['path'].poses]
                own = (data['own'].pose.pose.position.x,data['own'].pose.pose.position.y)
                peer = (data['peer'].pose.pose.position.x,data['peer'].pose.pose.position.y)
                chosen = choose_fixture(data['grid'], route, own, peer, args.arena_bounds) if len(route)>2 else None
                if chosen:
                    centre, detour = chosen
                    if args.select_only:
                        report.update(fixture_map_xy=list(centre), original_route=route, offline_alternative=detour, selection_only=True)
                        print(json.dumps(report), flush=True)
                        break
                    request = SpawnEntity.Request()
                    request.name = 'unmapped_test_box'
                    request.reference_frame = 'world'
                    request.initial_pose.position.x = centre[0]+args.origin_x
                    request.initial_pose.position.y = centre[1]+args.origin_y
                    request.initial_pose.position.z = args.height/2
                    request.initial_pose.orientation.w = 1.0
                    request.xml = f'''<sdf version="1.6"><model name="unmapped_test_box"><static>true</static><link name="box"><collision name="collision"><geometry><box><size>0.6 0.6 {args.height}</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.6 0.6 {args.height}</size></box></geometry></visual></link></model></sdf>'''
                    report.update(fixture_map_xy=list(centre), spawn_requested_sim_s=now,
                                  original_route=route, offline_alternative=detour,
                                  original_map_sha=__import__('hashlib').sha256(bytes((v+1)%256 for v in data['grid'].data)).hexdigest())
                    future = client.call_async(request)
                    print(json.dumps({'fixture_map_xy':centre,'spawn_requested_sim_s':now}), flush=True)
            if future is not None and future.done() and 'spawn_result' not in report:
                response = future.result()
                report['spawn_result'] = response.status_message
                report['fixture_spawned'] = response.success
                report['spawn_completed_sim_s'] = now
            if centre and now-last >= 0.1 and 'own' in data:
                last = now
                own = data['own'].pose.pose.position
                peer = data['peer'].pose.pose.position
                route = [(p.pose.position.x,p.pose.position.y) for p in data['path'].poses]
                scan = read_xyz(data['scan'], 16000) if 'scan' in data else []
                record = {'sim_s':now, 'own':[own.x,own.y], 'peer':[peer.x,peer.y],
                          'body_fixture_clearance_m':box_distance((own.x,own.y),centre)-0.178,
                          'peer_body_fixture_clearance_m':box_distance((peer.x,peer.y),centre)-0.178,
                          'global_path':route, 'status':data['status'].data if 'status' in data else None,
                          'fixture_scan_hits':sum(0.08 <= z <= 0.60 and box_distance((x,y),centre)<0.05 for x,y,z in scan),
                          'global_route_avoids_fixture':bool(route) and all(box_distance(p,centre)>=0.23 for p in samples(route))}
                if 'track' in data:
                    track = data['track'].pose.pose.position
                    record['estimated_opponent'] = [track.x,track.y]
                records.append(record)
                if args.record_scans:
                    for key, observer, target in (('scan', own, peer), ('peer_scan', peer, own)):
                        cloud = data.get(key)
                        if cloud is None:
                            continue
                        stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
                        if stamp - last_cloud.get(key, float('-inf')) < 0.5:
                            continue
                        last_cloud[key] = stamp
                        clouds.append(dict(observer=key, stamp_sim_s=stamp, received_sim_s=now,
                                           own_xy=[observer.x,observer.y],
                                           peer_truth_xy=[target.x,target.y],
                                           points=[p for p in read_xyz(cloud, 1000000)
                                                   if 0.08 <= p[2] <= 0.60]))

    finally:
        if records:
            physical_records = [r for r in records if r['sim_s'] >= report.get('spawn_completed_sim_s',float('inf'))]
            report['min_body_fixture_clearance_m'] = min((r['body_fixture_clearance_m'] for r in physical_records), default=None)
            report['min_peer_body_fixture_clearance_m'] = min((r['peer_body_fixture_clearance_m'] for r in physical_records), default=None)
            report['scan_detected_fixture'] = any(r['fixture_scan_hits']>=3 for r in records)
            report['safe_replanned_route_seen'] = any(r['global_route_avoids_fixture'] and r['sim_s']>report.get('spawn_completed_sim_s',float('inf')) for r in records)
            if report.get('outcome',{}).get('event') == 'guardian_capture':
                report['capture_fixture_line_of_sight_clear'] = all(box_distance(p,centre)>0 for p in samples([records[-1]['own'],records[-1]['peer']], 0.005))
        if args.record_scans and 'grid' in data:
            g = data['grid']
            report['replay_grid'] = dict(resolution=g.info.resolution,
                width=g.info.width, height=g.info.height,
                origin_x=g.info.origin.position.x, origin_y=g.info.origin.position.y,
                data=list(g.data))
            report['cloud_scope'] = 'read-only evaluation; peer truth is an offline label, never a navigation input'
        if not centre:
            report['error'] = 'No suitable free route point with a feasible alternative before outcome/deadline'
        report['final_map_sha'] = (__import__('hashlib').sha256(
            bytes((v+1)%256 for v in data['grid'].data)).hexdigest() if 'grid' in data else None)
        report['static_map_unchanged'] = (report.get('original_map_sha') == report['final_map_sha']
                                          if 'original_map_sha' in report else None)
        File(args.output).parent.mkdir(parents=True, exist_ok=True)
        File(args.output).write_text(json.dumps(report,indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    if not report['fixture_spawned'] and not (args.select_only and centre):
        raise SystemExit(1)

if __name__=='__main__':
    main()
