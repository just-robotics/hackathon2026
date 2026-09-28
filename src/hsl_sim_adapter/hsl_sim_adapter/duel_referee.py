"""Gazebo-only referee: end both autonomous runs at the first match result."""

import json
from datetime import datetime, timezone
from pathlib import Path
from time import monotonic

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .match_metrics import pose3, seconds
from .metrics import duel_outcome
from .visibility import StaticGrid


class DuelReferee(Node):
    def __init__(self):
        super().__init__("duel_referee")
        for name, value in (
                ("first_role", "explorer"),
                ("second_role", "guardian"),
                ("max_active_s", 360.0),
                ("spawn_x", -0.34), ("spawn_y", 0.4),
                ("first_start", [-0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, 0.5]),
                ("second_start", [2.59, 1.85, 3.09, 1.85, 3.09, 2.35, 2.59, 2.35]),
                ("run_id", "manual"), ("seed", 0), ("code_revision", "unknown"),
                ("result_dir", "/autoware/match-results")):
            self.declare_parameter(name, value)
        self.first_role = self.get_parameter("first_role").value
        self.second_role = self.get_parameter("second_role").value
        if {self.first_role, self.second_role} != {"explorer", "guardian"}:
            raise ValueError("duel requires one explorer and one guardian")
        self.max_active_s = float(self.get_parameter("max_active_s").value)
        self.spawn = (self.get_parameter("spawn_x").value,
                      self.get_parameter("spawn_y").value)
        self.first_start = self.get_parameter("first_start").value
        self.second_start = self.get_parameter("second_start").value
        self.run_id = self.get_parameter("run_id").value
        self.seed = self.get_parameter("seed").value
        self.code_revision = self.get_parameter("code_revision").value
        self.result_dir = Path(self.get_parameter("result_dir").value)
        self.allowed = [False, False]
        self.poses = [None, None]
        self.grid = None
        self.started_at = None
        self.wall_started_at = None
        self.finished = False
        self.stop_sent = set()
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/match/allowed",
                                 lambda msg: self.on_allowed(0, msg), qos)
        self.create_subscription(Bool, "/opponent/match/allowed",
                                 lambda msg: self.on_allowed(1, msg), qos)
        self.create_subscription(Odometry, "/localization/pose",
                                 lambda msg: self.on_pose(0, msg), 10)
        self.create_subscription(Odometry, "/opponent/localization/pose",
                                 lambda msg: self.on_pose(1, msg), 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.outcome_pub = self.create_publisher(String, "/match/outcome", qos)
        self.stop_clients = [self.create_client(SetBool, name) for name in
                             ("/match/allow_motion", "/opponent/match/allow_motion")]
        self.create_timer(0.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_allowed(self, index, msg):
        self.allowed[index] = msg.data

    def on_pose(self, index, msg):
        self.poses[index] = msg

    def on_map(self, msg):
        if msg.info.resolution > 0 and msg.info.width > 0 and msg.info.height > 0:
            self.grid = StaticGrid(msg.info.resolution, msg.info.width,
                                   msg.info.height, msg.info.origin.position.x,
                                   msg.info.origin.position.y, msg.data)

    def tick(self):
        if self.finished:
            self.stop_both()
            return
        if not all(self.allowed):
            return
        now = self.now()
        if self.started_at is None:
            self.started_at = now
            self.wall_started_at = monotonic()
            self.get_logger().info(f"Duel {self.run_id} started")
        positions = [pose3(msg, self.spawn) if msg is not None
                     and 0 <= now - seconds(msg.header.stamp) <= 0.5 else None
                     for msg in self.poses]
        event = (duel_outcome(self.first_role, *positions, self.first_start,
                              self.second_start, self.grid)
                 if all(positions) else None)
        if event is None and now - self.started_at >= self.max_active_s:
            event = "timeout"
        if event is not None:
            self.finish(event, now)

    def finish(self, event, now):
        self.finished = True
        roles = [self.first_role, self.second_role]
        report = {
            "run_id": self.run_id, "seed": self.seed,
            "code_revision": self.code_revision,
            "roles": roles, "event": event,
            "winner": "guardian" if event == "guardian_capture" else "explorer",
            "guardian_captured": event == "guardian_capture",
            "explorer_reached_goal": event == "explorer_goal",
            "duration_s": round(now - self.started_at, 2),
            "wall_duration_s": round(monotonic() - self.wall_started_at, 2),
        }
        body = json.dumps(report, sort_keys=True)
        self.outcome_pub.publish(String(data=body))
        try:
            self.result_dir.mkdir(parents=True, exist_ok=True)
            name = datetime.now(timezone.utc).strftime("outcome-%Y%m%dT%H%M%S%fZ.json")
            pretty = json.dumps(report, indent=2, sort_keys=True) + "\n"
            (self.result_dir / name).write_text(pretty)
            (self.result_dir / "latest_outcome.json").write_text(pretty)
        except OSError as error:
            self.get_logger().error(f"Could not save duel result: {error}")
        self.get_logger().info(f"Duel {self.run_id}: {event} at {report['duration_s']} s")
        self.stop_both()

    def stop_both(self):
        for index, client in enumerate(self.stop_clients):
            if index not in self.stop_sent and client.service_is_ready():
                self.stop_sent.add(index)
                client.call_async(SetBool.Request(data=False))


def main():
    rclpy.init()
    node = DuelReferee()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
