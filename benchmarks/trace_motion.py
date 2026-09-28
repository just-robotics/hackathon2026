#!/usr/bin/env python3
"""Sample one robot's command pipeline to explain low moving fraction."""

import argparse
import json
import time
from collections import Counter
from math import atan2, hypot, pi

import rclpy
from geometry_msgs.msg import Twist
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64, String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--wall-seconds", type=float, default=60)
    args = parser.parse_args()
    prefix = "/" + args.namespace.strip("/") if args.namespace else ""
    rclpy.init()
    node = Node("trace_motion")
    latest = {}
    samples = Counter()
    sums = Counter()
    subscriptions = []

    def remember(name):
        return lambda message: latest.__setitem__(name, (message, time.monotonic()))

    def on_path(message):
        previous = latest.get("path")
        latest["path"] = (message, time.monotonic())
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
        pose = latest.get("pose")
        if not pose or not message.poses:
            return
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
        Path, prefix + "/navigation/local_path", on_path, 10))
    subscriptions.append(node.create_subscription(
        Path, prefix + "/navigation/global_path", on_global_path, 10))

    def on_command(message):
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
                reason = "gate_clipped"
        samples[reason] += 1
        if status:
            samples["zero_status_" + status[0].data] += 1

    subscriptions.append(node.create_subscription(
        Twist, prefix + "/cmd_vel", on_command, 10))
    deadline = time.monotonic() + args.wall_seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    total = max(1, samples["total"])
    report = {"namespace": prefix or "/", "wall_seconds": args.wall_seconds,
              "counts": dict(samples),
              "fractions": {key: round(value / total, 3)
                            for key, value in samples.items() if key != "total"},
              "mean_cmd_linear": round(sums["cmd_linear"] / total, 3),
              "mean_cmd_angular_abs": round(sums["cmd_angular_abs"] / total, 3),
              "last_v_ref": latest["v_ref"][0].data if "v_ref" in latest else None,
              "last_v_curve": latest["v_curve"][0].data if "v_curve" in latest else None}
    print(json.dumps(report, indent=2, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
