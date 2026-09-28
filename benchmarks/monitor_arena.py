#!/usr/bin/env python3
"""Observe whether either duel robot leaves the configured arena in map frame."""

import argparse
import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


class ArenaMonitor(Node):
    def __init__(self, bounds):
        super().__init__("arena_monitor")
        self.bounds = bounds
        self.samples = {"first": 0, "second": 0}
        self.outside = {"first": 0, "second": 0}
        self.min_margin = {"first": float("inf"), "second": float("inf")}
        self.extents = {"first": [float("inf"), float("inf"),
                                   float("-inf"), float("-inf")],
                        "second": [float("inf"), float("inf"),
                                    float("-inf"), float("-inf")]}
        self.outcome = None
        self.allowed = [False, False]
        self.started = False
        self.stopped = False
        self.create_subscription(Odometry, "/navigation/self",
                                 lambda msg: self.on_pose("first", msg), 10)
        self.create_subscription(Odometry, "/opponent/navigation/self",
                                 lambda msg: self.on_pose("second", msg), 10)
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/match/outcome",
                                 self.on_outcome, qos)
        self.create_subscription(Bool, "/match/allowed",
                                 lambda msg: self.on_allowed(0, msg), qos)
        self.create_subscription(Bool, "/opponent/match/allowed",
                                 lambda msg: self.on_allowed(1, msg), qos)

    def on_pose(self, name, msg):
        if msg.header.frame_id != "map":
            return
        p = msg.pose.pose.position
        left, bottom, right, top = self.bounds
        margin = min(p.x - left, p.y - bottom, right - p.x, top - p.y)
        self.samples[name] += 1
        self.outside[name] += margin < 0
        self.min_margin[name] = min(self.min_margin[name], margin)
        extent = self.extents[name]
        extent[0], extent[1] = min(extent[0], p.x), min(extent[1], p.y)
        extent[2], extent[3] = max(extent[2], p.x), max(extent[3], p.y)

    def on_outcome(self, msg):
        self.outcome = json.loads(msg.data)

    def on_allowed(self, index, msg):
        self.allowed[index] = msg.data
        self.started |= all(self.allowed)
        self.stopped |= self.started and not any(self.allowed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bounds", nargs=4, type=float,
                        metavar=("MIN_X", "MIN_Y", "MAX_X", "MAX_Y"))
    parser.add_argument("--wall-timeout", type=float, default=180)
    args = parser.parse_args()
    rclpy.init()
    node = ArenaMonitor(args.bounds)
    deadline = time.monotonic() + args.wall_timeout
    try:
        while time.monotonic() < deadline and node.outcome is None and not node.stopped:
            rclpy.spin_once(node, timeout_sec=0.2)
        result = {"bounds": args.bounds, "outcome": node.outcome,
                  "samples": node.samples, "outside_samples": node.outside,
                  "min_margin_m": node.min_margin, "extents": node.extents}
        print(json.dumps(result, indent=2, sort_keys=True), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
