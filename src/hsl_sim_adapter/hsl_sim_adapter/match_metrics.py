"""Gazebo-only referee and run metrics, isolated from the navigation policy."""

import json
from datetime import datetime, timezone
from math import hypot
from pathlib import Path
from time import monotonic

import rclpy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, String

from .metrics import RunMetrics, capture_possible
from .visibility import StaticGrid


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose3(odom, spawn):
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    from math import atan2
    yaw = atan2(2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z))
    return p.x - spawn[0], p.y - spawn[1], yaw


class MatchMetrics(Node):
    def __init__(self):
        super().__init__("match_metrics")
        self.declare_parameter("role", "explorer")
        self.declare_parameter("own_spawn_x", -0.34)
        self.declare_parameter("own_spawn_y", 0.4)
        self.declare_parameter("result_dir", "/autoware/match-results")
        self.declare_parameter("max_reference_speed", 1.0)
        self.declare_parameter("own_truth_topic", "/localization/pose")
        self.declare_parameter("opponent_truth_topic", "/opponent/localization/pose")
        self.declare_parameter("other_model", "opponent")
        self.declare_parameter("run_id", "manual")
        self.declare_parameter("seed", 0)
        self.declare_parameter("code_revision", "unknown")
        self.role = self.get_parameter("role").value
        self.other_model = self.get_parameter("other_model").value
        self.spawn = (self.get_parameter("own_spawn_x").value,
                      self.get_parameter("own_spawn_y").value)
        self.result_dir = Path(self.get_parameter("result_dir").value)
        self.metrics = RunMetrics(self.get_parameter("max_reference_speed").value)
        self.own = None
        self.opponent = None
        self.grid = None
        self.visible = False
        self.planner_ok = False
        self.planner_status = None
        self.behavior = None
        self.command = None
        self.pending_stop = None
        self.window_source = "allow_motion"
        self.completed = False
        state_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "match/allowed", self.on_allowed, state_qos)
        self.create_subscription(String, "/match/outcome", self.on_outcome, state_qos)
        self.create_subscription(Odometry, self.get_parameter("own_truth_topic").value,
                                 self.on_own, 10)
        self.create_subscription(Odometry, self.get_parameter("opponent_truth_topic").value,
                                 self.on_opponent, 10)
        self.create_subscription(ContactsState, "body_contacts", self.on_contacts, 10)
        self.create_subscription(Bool, "navigation/opponent_visible", self.on_visible, 10)
        self.create_subscription(String, "navigation/planner_status", self.on_planner, 10)
        self.create_subscription(PlanningIntent, "navigation/intent", self.on_intent, 10)
        self.create_subscription(PointCloud2, "navigation/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self.on_command, 10)
        for name, topic in (
                ("decision_compute", "navigation/decision_cycle_ms"),
                ("planner_compute", "navigation/planner_cycle_ms"),
                ("control_compute", "navigation/control_cycle_ms")):
            self.create_subscription(Float32, topic,
                                     lambda msg, key=name: self.metrics.record_timing(
                                         key, msg.data, self.now()), 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, state_qos)
        self.pub = self.create_publisher(String, "match/metrics", state_qos)
        self.create_timer(0.1, self.sample)
        self.create_timer(1.0, self.publish)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_allowed(self, msg):
        if self.completed:
            return
        now = self.now()
        if msg.data and not self.metrics.active:
            self.metrics.start(now, monotonic())
            self.pending_stop = None
            self.window_source = "allow_motion"
            self.sample()
            self.get_logger().info("Match metrics started")
        elif not msg.data and self.metrics.active:
            self.pending_stop = (now, monotonic())

    def on_outcome(self, msg):
        report = json.loads(msg.data)
        if report.get("run_id") != self.get_parameter("run_id").value:
            return
        if self.completed:
            return
        start, end = report.get("started_at_sim_s"), report.get("finished_at_sim_s")
        if start is None or end is None or self.metrics.started_at is None:
            return
        self.sample()
        self.metrics.finish_window(start, end, report.get("wall_duration_s"))
        if report.get("event") == "explorer_goal" and self.role == "explorer":
            self.metrics.goal_at = end
        if report.get("event") == "guardian_capture" and self.role == "guardian":
            self.metrics.capture_at = end
        self.window_source = "referee"
        self.completed = True
        self.pending_stop = None
        self.publish()
        self.save()

    def on_own(self, msg):
        self.own = msg

    def on_opponent(self, msg):
        self.opponent = msg

    def on_visible(self, msg):
        self.visible = msg.data

    def on_planner(self, msg):
        self.planner_status = msg.data
        self.planner_ok = msg.data == "OK"
        self.metrics.observe_stream("planner", monotonic(), self.now())

    def on_intent(self, msg):
        self.metrics.observe_stream("decision", monotonic(), self.now())
        self.behavior = msg.behavior
        if (self.metrics.active and self.role == "explorer"
                and msg.reason == "guardian start area reached"
                and self.metrics.goal_at is None):
            self.metrics.goal_at = self.now()

    def on_scan(self, _msg):
        self.metrics.observe_stream("scan", monotonic(), self.now())

    def on_command(self, msg):
        self.metrics.observe_stream("command", monotonic(), self.now())
        self.command = (msg.linear.x, msg.angular.z, self.now())

    def on_map(self, msg):
        if msg.info.resolution > 0 and msg.info.width > 0 and msg.info.height > 0:
            self.grid = StaticGrid(msg.info.resolution, msg.info.width,
                                   msg.info.height, msg.info.origin.position.x,
                                   msg.info.origin.position.y, msg.data)

    def on_contacts(self, msg):
        kind = None
        for state in msg.states:
            names = (state.collision1_name, state.collision2_name)
            if any("ground_plane" in name for name in names):
                continue
            kind = "robot" if any(name.startswith(self.other_model + "::")
                                  for name in names) else "wall"
            break
        position = pose3(self.own, self.spawn)[:2] if self.own is not None else None
        self.metrics.contact(self.now(), kind, position)

    def sample(self):
        now = self.now()
        if self.pending_stop and monotonic() - self.pending_stop[1] >= 1.0:
            self.metrics.stop(*self.pending_stop)
            self.pending_stop = None
            self.publish()
            self.save()
        if not self.metrics.active or self.own is None:
            return
        if not 0 <= now - seconds(self.own.header.stamp) <= 0.5:
            return
        x, y, yaw = pose3(self.own, self.spawn)
        twist = self.own.twist.twist
        speed = hypot(twist.linear.x, twist.linear.y)
        command = (self.command[:2] if self.command is not None and
                   0 <= now - self.command[2] <= 0.3 else None)
        self.metrics.sample(now, x, y, speed, twist.angular.z,
                            self.visible, self.planner_ok, self.behavior, command,
                            self.planner_status)
        if (self.role == "guardian" and self.metrics.capture_at is None
                and self.opponent is not None
                and 0 <= now - seconds(self.opponent.header.stamp) <= 0.5
                and capture_possible((x, y, yaw), pose3(self.opponent, self.spawn),
                                     self.grid)):
            self.metrics.capture_at = now
            self.get_logger().info("Capture conditions met")

    def report(self):
        return {
            "role": self.role,
            "run_id": self.get_parameter("run_id").value,
            "seed": self.get_parameter("seed").value,
            "code_revision": self.get_parameter("code_revision").value,
            "window_source": self.window_source,
            **self.metrics.snapshot(self.now(), monotonic()),
        }

    def publish(self):
        self.pub.publish(String(data=json.dumps(self.report(), sort_keys=True)))

    def save(self):
        report = self.report()
        try:
            self.result_dir.mkdir(parents=True, exist_ok=True)
            name = datetime.now(timezone.utc).strftime("duel-%Y%m%dT%H%M%S%fZ.json")
            body = json.dumps(report, indent=2, sort_keys=True) + "\n"
            (self.result_dir / name).write_text(body)
            (self.result_dir / "latest.json").write_text(body)
            self.get_logger().info(f"Match metrics saved to {self.result_dir / name}")
        except OSError as error:
            self.get_logger().error(f"Could not save match metrics: {error}")


def main():
    rclpy.init()
    node = MatchMetrics()
    try:
        rclpy.spin(node)
    finally:
        if node.metrics.active:
            node.metrics.stop(node.now(), monotonic())
            node.save()
        node.destroy_node()
        rclpy.shutdown()
