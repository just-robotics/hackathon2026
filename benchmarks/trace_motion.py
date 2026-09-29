#!/usr/bin/env python3
"""Sample one robot's command pipeline to explain low moving fraction."""

import argparse
import json
import time
from collections import Counter
from math import atan2, cos, hypot, pi, sin

import rclpy
from geometry_msgs.msg import Twist
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile,
                       qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--wall-seconds", type=float, default=60)
    parser.add_argument("--spawn-x", type=float, default=-0.34)
    parser.add_argument("--spawn-y", type=float, default=0.4)
    args = parser.parse_args()
    prefix = "/" + args.namespace.strip("/") if args.namespace else ""
    rclpy.init()
    node = Node("trace_motion")
    latest = {}
    match = {"started": False, "finished": False}
    samples = Counter()
    sums = Counter()
    path_lengths = []
    path_errors = []
    published_spans = []
    published_lengths = []
    published_tangent_errors = []
    published_chord_errors = []
    global_lengths = []
    clipped_errors = []
    gate_clipped_tangent_errors = []
    opponent_position_errors = []
    subscriptions = []

    def remember(name):
        def callback(message):
            latest[name] = (message, time.monotonic(),
                            node.get_clock().now().nanoseconds * 1e-9)
        return callback

    def on_path(message):
        previous = latest.get("path")
        latest["path"] = (message, time.monotonic(),
                           node.get_clock().now().nanoseconds * 1e-9)
        if not latest.get("allowed", False):
            return
        samples["path_updates"] += 1
        if len(message.poses) >= 2:
            positions = [pose.pose.position for pose in message.poses]
            first, last = positions[0], positions[-1]
            span = hypot(last.x - first.x, last.y - first.y)
            published_spans.append(span)
            published_lengths.append(sum(
                hypot(b.x - a.x, b.y - a.y)
                for a, b in zip(positions, positions[1:])))
            pose = latest.get("pose")
            if pose and span >= 0.02:
                p = pose[0].pose.pose.position
                q = pose[0].pose.pose.orientation
                own_yaw = atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z))
                nearest = min(range(len(positions)),
                              key=lambda i: hypot(positions[i].x - p.x,
                                                  positions[i].y - p.y))
                before = positions[max(0, nearest - 2)]
                after = positions[min(len(positions) - 1, nearest + 3)]
                tangent = atan2(after.y - before.y, after.x - before.x)
                chord = atan2(last.y - first.y, last.x - first.x)
                published_tangent_errors.append(
                    abs((tangent - own_yaw + pi) % (2 * pi) - pi))
                published_chord_errors.append(
                    abs((chord - own_yaw + pi) % (2 * pi) - pi))
                forward = ((last.x - p.x) * cos(own_yaw) +
                           (last.y - p.y) * sin(own_yaw))
                if forward < -0.05:
                    samples["path_ends_behind"] += 1
        if len(message.poses) >= 3:
            first = message.poses[0].pose.position
            last = message.poses[-1].pose.position
            span = hypot(last.x - first.x, last.y - first.y)
            if span > 0.02 and max(abs((p.pose.position.x - first.x) *
                                       (last.y - first.y) -
                                       (p.pose.position.y - first.y) *
                                       (last.x - first.x)) / span
                                   for p in message.poses) > 0.035:
                samples["curved_path_updates"] += 1
        if (not previous or len(previous[0].poses) != len(message.poses) or
                any((old.pose.position.x != new.pose.position.x or
                     old.pose.position.y != new.pose.position.y)
                    for old, new in zip(previous[0].poses, message.poses))):
            samples["path_geometry_changes"] += 1
        if not previous or len(message.poses) < 2 or len(previous[0].poses) < 2:
            return
        first = message.poses[0].pose.position
        last = message.poses[-1].pose.position
        old_first = previous[0].poses[0].pose.position
        old_last = previous[0].poses[-1].pose.position
        if (hypot(last.x - first.x, last.y - first.y) >= 0.02 or
                hypot(old_last.x - old_first.x, old_last.y - old_first.y) >= 0.02):
            return
        q = message.poses[-1].pose.orientation
        yaw = atan2(2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z))
        q = previous[0].poses[-1].pose.orientation
        old_yaw = atan2(2 * (q.w * q.z + q.x * q.y),
                        1 - 2 * (q.y * q.y + q.z * q.z))
        if abs((yaw - old_yaw + pi) % (2 * pi) - pi) > 1.0:
            samples["rotation_target_flips"] += 1

    def on_global_path(message):
        if not latest.get("allowed", False):
            return
        pose = latest.get("pose")
        if not pose or not message.poses:
            return
        global_lengths.append(sum(
            hypot(b.pose.position.x - a.pose.position.x,
                  b.pose.position.y - a.pose.position.y)
            for a, b in zip(message.poses, message.poses[1:])))
        position = pose[0].pose.pose.position
        candidate = next((item.pose.position for item in message.poses
                          if hypot(item.pose.position.x - position.x,
                                   item.pose.position.y - position.y) >= 0.3), None)
        if candidate is None:
            return
        heading = atan2(candidate.y - position.y, candidate.x - position.x)
        old = latest.get("global_heading")
        if old and hypot(position.x - old[1], position.y - old[2]) < 0.2:
            error = abs((heading - old[0] + pi) % (2 * pi) - pi)
            if error > 1.0:
                samples["global_heading_flips"] += 1
        latest["global_heading"] = (heading, position.x, position.y)

    for name, kind, topic in (
        ("intent", PlanningIntent, "navigation/intent"),
        ("status", String, "navigation/planner_status"),
        ("mpc", Twist, "navigation/mpc_cmd_vel"),
        ("v_ref", Float64, "v_ref"),
        ("v_curve", Float64, "v_curve"),
        ("pose", Odometry, "navigation/self"),
    ):
        subscriptions.append(node.create_subscription(
            kind, prefix + "/" + topic, remember(name), 10))
    subscriptions.append(node.create_subscription(
        PointCloud2, prefix + "/navigation/scan", remember("scan"),
        qos_profile_sensor_data))
    def on_allowed(message):
        latest["allowed"] = message.data
        if message.data:
            match["started"] = True
        elif match["started"]:
            match["finished"] = True

    subscriptions.append(node.create_subscription(
        Bool, prefix + "/match/allowed", on_allowed,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    subscriptions.append(node.create_subscription(
        String, "/match/outcome",
        lambda message: match.__setitem__("finished", True)
        if match["started"] else None, 10))
    subscriptions.append(node.create_subscription(
        Path, prefix + "/navigation/local_path", on_path, 10))
    subscriptions.append(node.create_subscription(
        Path, prefix + "/navigation/global_path", on_global_path, 10))

    truth_topic = ("/opponent/localization/pose" if not prefix
                   else "/localization/pose")
    subscriptions.append(node.create_subscription(
        Odometry, truth_topic, remember("opponent_truth"), 10))

    def on_opponent_estimate(message):
        latest["opponent_estimate"] = (message, time.monotonic())
        truth = latest.get("opponent_truth")
        if not match["started"] or not truth:
            return
        truth_message = truth[0]
        estimate_stamp = (message.header.stamp.sec * 1_000_000_000 +
                          message.header.stamp.nanosec)
        truth_stamp = (truth_message.header.stamp.sec * 1_000_000_000 +
                       truth_message.header.stamp.nanosec)
        if abs(estimate_stamp - truth_stamp) > 150_000_000:
            return
        estimated = message.pose.pose.position
        actual = truth_message.pose.pose.position
        opponent_position_errors.append(hypot(
            estimated.x - (actual.x - args.spawn_x),
            estimated.y - (actual.y - args.spawn_y)))

    subscriptions.append(node.create_subscription(
        Odometry, prefix + "/navigation/opponent", on_opponent_estimate, 10))

    def on_command(message):
        if not latest.get("allowed", False):
            return
        now = time.monotonic()
        samples["total"] += 1
        sums["cmd_linear"] += message.linear.x
        sums["cmd_angular_abs"] += abs(message.angular.z)
        if message.linear.x > 0.3:
            samples["cmd_over_0_3"] += 1
        if abs(message.linear.x) > 0.02:
            samples["moving"] += 1
            return
        intent = latest.get("intent")
        path = latest.get("path")
        mpc = latest.get("mpc")
        status = latest.get("status")
        if path and len(path[0].poses) >= 2:
            first = path[0].poses[0].pose.position
            last = path[0].poses[-1].pose.position
            length = hypot(last.x - first.x, last.y - first.y)
            path_lengths.append(length)
            pose = latest.get("pose")
            if pose and length >= 0.02:
                q = pose[0].pose.pose.orientation
                own_yaw = atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z))
                error = abs((atan2(last.y - first.y, last.x - first.x) -
                             own_yaw + pi) % (2 * pi) - pi)
                path_errors.append(error)
                if message.linear.x <= 0.02 and mpc and mpc[0].linear.x > 0.02:
                    clipped_errors.append(error)
        if not intent or now - intent[1] > 1.0 or intent[0].behavior in (0, 1):
            reason = "intent_wait_stop_stale"
        elif not path or now - path[1] > 1.0 or not path[0].poses:
            reason = "empty_or_stale_path"
        else:
            first = path[0].poses[0].pose.position
            last = path[0].poses[-1].pose.position
            if hypot(last.x - first.x, last.y - first.y) < 0.02:
                reason = "rotate_path"
                pose = latest.get("pose")
                if pose:
                    q = pose[0].pose.pose.orientation
                    own_yaw = atan2(2 * (q.w * q.z + q.x * q.y),
                                    1 - 2 * (q.y * q.y + q.z * q.z))
                    q = path[0].poses[-1].pose.orientation
                    target_yaw = atan2(2 * (q.w * q.z + q.x * q.y),
                                       1 - 2 * (q.y * q.y + q.z * q.z))
                    error = abs((target_yaw - own_yaw + pi) % (2 * pi) - pi)
                    if error < 0.1:
                        samples["rotate_already_aligned"] += 1
                    elif error < 0.5:
                        samples["rotate_error_under_0_5"] += 1
                    else:
                        samples["rotate_error_over_0_5"] += 1
            elif not mpc or now - mpc[1] > 0.5 or mpc[0].linear.x <= 0.02:
                reason = "mpc_zero_or_stale"
            else:
                sim_now = node.get_clock().now().nanoseconds * 1e-9
                if not pose or sim_now - pose[2] > 1.2:
                    reason = "gate_stale_pose"
                elif not latest.get("scan") or sim_now - latest["scan"][2] > 1.8:
                    reason = "gate_stale_scan"
                elif sim_now - path[2] > 1.0:
                    reason = "gate_stale_path"
                elif sim_now - intent[2] > 1.0:
                    reason = "gate_stale_intent"
                else:
                    reason = "gate_clipped"
                    if pose and len(path[0].poses) >= 2:
                        p = pose[0].pose.pose.position
                        q = pose[0].pose.pose.orientation
                        own_yaw = atan2(2 * (q.w * q.z + q.x * q.y),
                                        1 - 2 * (q.y * q.y + q.z * q.z))
                        points = [item.pose.position for item in path[0].poses]
                        nearest = min(range(len(points)), key=lambda i:
                                      hypot(points[i].x - p.x,
                                            points[i].y - p.y))
                        before = points[max(0, nearest - 2)]
                        after = points[min(len(points) - 1, nearest + 3)]
                        if hypot(after.x - before.x,
                                 after.y - before.y) >= 0.02:
                            tangent = atan2(after.y - before.y,
                                            after.x - before.x)
                            error = abs((tangent - own_yaw + pi) % (2 * pi) - pi)
                            gate_clipped_tangent_errors.append(error)
                            samples["gate_tangent_over_0_75"] += error > 0.75
                            samples["gate_tangent_over_1_0"] += error > 1.0
                            samples["gate_tangent_over_1_2"] += error > 1.2
        samples[reason] += 1
        if status:
            samples["zero_status_" + status[0].data] += 1

    subscriptions.append(node.create_subscription(
        Twist, prefix + "/cmd_vel", on_command, 10))
    deadline = time.monotonic() + args.wall_seconds
    while time.monotonic() < deadline and not match["finished"]:
        rclpy.spin_once(node, timeout_sec=0.2)
    total = max(1, samples["total"])
    def percentile(values, fraction):
        if not values:
            return None
        ordered = sorted(values)
        return round(ordered[min(len(ordered) - 1,
                                 int(fraction * (len(ordered) - 1)))], 3)

    report = {"namespace": prefix or "/", "wall_seconds": args.wall_seconds,
              "counts": dict(samples),
              "fractions": {key: round(value / total, 3)
                            for key, value in samples.items() if key != "total"},
              "mean_cmd_linear": round(sums["cmd_linear"] / total, 3),
              "mean_cmd_angular_abs": round(sums["cmd_angular_abs"] / total, 3),
              "path_length_median_m": percentile(path_lengths, 0.5),
              "path_length_p90_m": percentile(path_lengths, 0.9),
              "path_heading_error_median_rad": percentile(path_errors, 0.5),
              "path_heading_error_p90_rad": percentile(path_errors, 0.9),
              "published_path_span_median_m": percentile(published_spans, 0.5),
              "published_path_length_median_m": percentile(published_lengths, 0.5),
              "published_path_length_p90_m": percentile(published_lengths, 0.9),
              "published_path_length_max_m": (round(max(published_lengths), 3)
                                              if published_lengths else None),
              "global_path_length_median_m": percentile(global_lengths, 0.5),
              "path_ends_behind_fraction": round(
                  samples["path_ends_behind"] /
                  max(1, samples["path_updates"]), 3),
              "published_tangent_error_median_rad": percentile(
                  published_tangent_errors, 0.5),
              "published_chord_error_median_rad": percentile(
                  published_chord_errors, 0.5),
              "gate_clipped_heading_error_median_rad": percentile(clipped_errors, 0.5),
              "gate_clipped_tangent_error_median_rad": percentile(
                  gate_clipped_tangent_errors, 0.5),
              "gate_clipped_tangent_error_p90_rad": percentile(
                  gate_clipped_tangent_errors, 0.9),
              "opponent_position_error_count": len(opponent_position_errors),
              "opponent_position_error_median_m": percentile(
                  opponent_position_errors, 0.5),
              "opponent_position_error_p90_m": percentile(
                  opponent_position_errors, 0.9),
              "opponent_position_error_max_m": (round(max(opponent_position_errors), 3)
                                                  if opponent_position_errors else None),
              "path_geometry_change_fraction": round(
                  samples["path_geometry_changes"] /
                  max(1, samples["path_updates"]), 3),
              "curved_path_fraction": round(
                  samples["curved_path_updates"] /
                  max(1, samples["path_updates"]), 3),
              "last_v_ref": latest["v_ref"][0].data if "v_ref" in latest else None,
              "last_v_curve": latest["v_curve"][0].data if "v_curve" in latest else None}
    print(json.dumps(report, indent=2, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
