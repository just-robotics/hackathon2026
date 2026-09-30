#!/usr/bin/env python3
"""Print a live planner map/path diagnosis without publishing control topics."""

import argparse
import json
import time
from math import hypot

import rclpy
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from hsl_planning.core import (Pose2, VoxelWorld, astar, local_guidance,
                               reachable_target, recovery_step, safe_segment)
from hsl_planning.node import odom_pose, read_xyz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--on-status", default="",
                        help="wait for this planner status before taking a snapshot")
    parser.add_argument("--wait-s", type=float, default=300)
    args = parser.parse_args()
    prefix = "/" + args.namespace.strip("/") if args.namespace else ""
    rclpy.init()
    node = Node("planner_probe")
    data = {}
    subscriptions = []
    for key, topic, kind, qos in (
        ("own", "self", Odometry, 10), ("opponent", "opponent", Odometry, 10),
        ("intent", "intent", PlanningIntent, 10),
        ("map_points", "map_points", PointCloud2, qos_profile_sensor_data),
        ("scan", "scan", PointCloud2, qos_profile_sensor_data),
        ("status", "planner_status", String, 10),
        ("mppi_diagnostics", "mppi_diagnostics", String, 10),
        ("global_path", "global_path", Path, 10),
        ("local_path", "local_path", Path, 10),
        ("known_grid", "known_grid", OccupancyGrid,
         QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)),
    ):
        subscriptions.append(node.create_subscription(
            kind, prefix + "/navigation/" + topic,
            lambda msg, name=key: data.__setitem__(name, msg), qos))
    deadline = time.monotonic() + args.wait_s
    required = {"own", "intent", "map_points", "scan", "known_grid", "status"}
    while time.monotonic() < deadline and (not required.issubset(data) or
           args.on_status and data["status"].data != args.on_status):
        rclpy.spin_once(node, timeout_sec=0.5)
    if (not required.issubset(data) or
            args.on_status and data["status"].data != args.on_status):
        print(json.dumps({"missing": sorted(required - data.keys()),
                          "last_status": data["status"].data
                          if "status" in data else None}))
        raise SystemExit(1)
    own = odom_pose(data["own"])
    enemy = odom_pose(data["opponent"]) if "opponent" in data else None
    intent = data["intent"]
    grid = data["known_grid"]
    world = VoxelWorld(0.15, 0.23)
    static = read_xyz(data["map_points"])
    scan = read_xyz(data["scan"], 5000)
    if enemy:
        static = [p for p in static if hypot(p[0] - enemy.x, p[1] - enemy.y) > 0.45]
        scan = [p for p in scan if hypot(p[0] - enemy.x, p[1] - enemy.y) > 0.45]
    free = set()
    occupied_points = []
    for index, value in enumerate(grid.data):
        row, col = divmod(index, grid.info.width)
        x = grid.info.origin.position.x + (col + 0.5) * grid.info.resolution
        y = grid.info.origin.position.y + (row + 0.5) * grid.info.resolution
        if value >= 50:
            occupied_points.append((x, y, 0.3))
        elif value == 0:
            free.add(world.cell(x, y))
    bounds = (grid.info.origin.position.x, grid.info.origin.position.y,
              grid.info.origin.position.x + grid.info.width * grid.info.resolution,
              grid.info.origin.position.y + grid.info.height * grid.info.resolution)
    world.update(static + occupied_points, scan, own, free, bounds)
    requested = (Pose2(intent.target.position.x, intent.target.position.y)
                 if intent.has_target else None)
    target = reachable_target(world, own, requested,
                              explore=intent.behavior == 3)
    source = world.cell(own.x, own.y)
    near = {}
    for dy in range(2, -3, -1):
        near[str(dy)] = "".join("#" if (source[0] + dx, source[1] + dy) in world.occupied
                                else "." if (source[0] + dx, source[1] + dy) in world.free
                                else "?" for dx in range(-2, 3))
    route = astar(world, own, target, enemy, intent.opponent_clearance,
                  intent.opponent_cost_weight) if target else []
    frontier = world.frontier(own, target, min_travel=0.6)
    frontier_route = (astar(world, own, frontier, enemy,
                            intent.opponent_clearance,
                            intent.opponent_cost_weight)
                      if frontier else [])
    alternatives = []
    for cell in world.free:
        if cell in world.occupied:
            continue
        point = world.point(cell)
        if hypot(point.x - own.x, point.y - own.y) < 0.6:
            continue
        if not any((cell[0] + dx, cell[1] + dy) not in world.free
                   for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))):
            continue
        alternatives.append((hypot(point.x - target.x, point.y - target.y)
                             if target else 0.0, point))
    alternatives.sort(key=lambda item: item[0])
    first_reachable_frontier = None
    for _, point in alternatives[:30]:
        candidate_route = astar(world, own, point, enemy,
                                intent.opponent_clearance,
                                intent.opponent_cost_weight)
        if candidate_route:
            first_reachable_frontier = [point.x, point.y, len(candidate_route)]
            break
    neighbors = []
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                   (1, 1), (1, -1), (-1, 1), (-1, -1)):
        point = world.point((source[0] + dx, source[1] + dy))
        neighbors.append({"cell": [source[0] + dx, source[1] + dy],
                          "blocked": world.blocked(point.x, point.y),
                          "known_free": world.cell(point.x, point.y) in world.free,
                          "segment_safe": safe_segment(world, own, point, enemy,
                                                       intent.opponent_clearance),
                          "opponent_distance_m": hypot(point.x - enemy.x,
                                                       point.y - enemy.y)
                          if enemy else None})
    local = local_guidance(world, own, route, enemy,
                           intent.opponent_clearance) if route else []
    print(json.dumps({"own": [own.x, own.y, own.yaw],
                      "opponent": [enemy.x, enemy.y] if enemy else None,
                      "behavior": intent.behavior,
                      "planner_status": data["status"].data,
                      "actual_mppi_diagnostics": (json.loads(data["mppi_diagnostics"].data)
                          if "mppi_diagnostics" in data else None),
                      "actual_global_head": [[p.pose.position.x, p.pose.position.y]
                          for p in data["global_path"].poses[:10]]
                          if "global_path" in data else None,
                      "actual_local_path": [[p.pose.position.x, p.pose.position.y]
                          for p in data["local_path"].poses]
                          if "local_path" in data else None,
                      "source_cell": source,
                      "source_blocked": source in world.occupied,
                      "source_clearance_m": world.obstacle_clearance(own.x, own.y),
                      "next_south_clearance_m": world.obstacle_clearance(own.x, own.y - 0.15),
                      "target": [target.x, target.y] if target else None,
                      "requested_target": [requested.x, requested.y]
                      if requested else None,
                      "target_blocked": world.blocked(target.x, target.y) if target else None,
                      "route_cells": len(route),
                      "frontier": [frontier.x, frontier.y] if frontier else None,
                      "frontier_route_cells": len(frontier_route),
                      "frontiers_examined": min(30, len(alternatives)),
                      "first_reachable_frontier": first_reachable_frontier,
                      "recovery_step": recovery_step(
                          world, own, enemy, intent.opponent_clearance),
                      "neighbors": neighbors,
                      "route_head": [[p.x, p.y] for p in route[:7]],
                      "local_points": len(local),
                      "local_end": [local[-1].x, local[-1].y] if local else None,
                      "local_occupancy": near,
                      "occupied_cells": len(world.occupied), "free_cells": len(world.free)},
                     indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
