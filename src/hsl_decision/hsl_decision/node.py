"""ROS-facing decision manager."""

from math import atan2, cos, sin
from time import perf_counter

import rclpy
from hsl_interfaces.msg import PlanningIntent
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool

from .core import DecisionPolicy, Observation, Pose2


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose2(odom):
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return Pose2(p.x, p.y, yaw)


class DecisionManager(Node):
    def __init__(self):
        super().__init__("decision_manager")
        defaults = {
            "role": "explorer", "planning_frame": "map",
            "own_start": [-0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, 0.5],
            "opponent_start": [2.59, 1.85, 3.09, 1.85, 3.09, 2.35, 2.59, 2.35],
            "pose_timeout": 1.2, "scan_timeout": 1.8,
            "opponent_timeout": 1.0, "switch_margin": 0.15,
            "min_dwell": 0.5, "evade_distance": 1.8,
            "capture_distance": 0.8, "danger_weight": 2.0,
            "goal_weight": 1.0, "evade_target_distance": 1.5,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        values = {key: self.get_parameter(key).value for key in defaults}
        self.frame = values.pop("planning_frame")
        self.policy = DecisionPolicy(**values)
        self.allowed = False
        self.own = None
        self.opponent = None
        self.scan_stamp = 0.0
        self.map_stamp = 0.0
        self.create_subscription(Odometry, "navigation/self", self.on_own, 10)
        self.create_subscription(Odometry, "navigation/opponent", self.on_opponent, 10)
        self.create_subscription(PointCloud2, "navigation/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "navigation/map_points", self.on_map,
                                 qos_profile_sensor_data)
        self.intent_pub = self.create_publisher(PlanningIntent, "navigation/intent", 10)
        self.state_pub = self.create_publisher(String, "navigation/behavior", 10)
        self.cycle_pub = self.create_publisher(Float32, "navigation/decision_cycle_ms", 10)
        state_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.allowed_pub = self.create_publisher(Bool, "match/allowed", state_qos)
        self.allowed_pub.publish(Bool(data=False))
        self.create_service(SetBool, "match/allow_motion", self.on_allow)
        self.create_timer(0.2, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_own(self, msg):
        if msg.header.frame_id != self.frame:
            self.get_logger().warn("self odometry must be in planning frame")
            return
        self.own = (pose2(msg), stamp_seconds(msg.header.stamp))

    def on_opponent(self, msg):
        if msg.header.frame_id == self.frame:
            pose = pose2(msg)
            twist = msg.twist.twist
            vx = cos(pose.yaw) * twist.linear.x - sin(pose.yaw) * twist.linear.y
            vy = sin(pose.yaw) * twist.linear.x + cos(pose.yaw) * twist.linear.y
            self.opponent = (pose, stamp_seconds(msg.header.stamp), (vx, vy))

    def on_scan(self, msg):
        self.scan_stamp = stamp_seconds(msg.header.stamp)

    def on_map(self, msg):
        self.map_stamp = stamp_seconds(msg.header.stamp)

    def on_allow(self, request, response):
        self.allowed = request.data
        self.allowed_pub.publish(Bool(data=self.allowed))
        response.success = True
        response.message = "motion enabled" if self.allowed else "motion disabled"
        return response

    def tick(self):
        started = perf_counter()
        try:
            self._tick()
        finally:
            self.cycle_pub.publish(Float32(data=(perf_counter() - started) * 1000))

    def _tick(self):
        obs = Observation(
            now=self.now(), own=self.own[0] if self.own else None,
            own_stamp=self.own[1] if self.own else 0.0,
            opponent=self.opponent[0] if self.opponent else None,
            opponent_stamp=self.opponent[1] if self.opponent else 0.0,
            opponent_velocity=self.opponent[2] if self.opponent else (0.0, 0.0),
            scan_stamp=self.scan_stamp, map_stamp=self.map_stamp,
            allowed=self.allowed,
        )
        decision = self.policy.step(obs)
        msg = PlanningIntent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.behavior = decision.behavior
        if decision.target:
            msg.has_target = True
            msg.target.position.x = decision.target.x
            msg.target.position.y = decision.target.y
            msg.target.orientation.z = sin(decision.target.yaw / 2)
            msg.target.orientation.w = cos(decision.target.yaw / 2)
        msg.target_tolerance = float(decision.tolerance)
        msg.max_speed = float(decision.max_speed)
        msg.opponent_clearance = float(decision.opponent_clearance)
        msg.opponent_cost_weight = float(decision.opponent_cost_weight)
        msg.reason = decision.reason
        self.intent_pub.publish(msg)
        self.state_pub.publish(String(data=decision.reason))


def main():
    rclpy.init()
    node = DecisionManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
