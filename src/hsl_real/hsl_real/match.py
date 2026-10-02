"""One real robot: movement permission, stage timer and goal stop, no truth referee."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from hsl_interfaces.msg import PlanningIntent


class RealMatch(Node):
    def __init__(self):
        super().__init__('real_match')
        self.declare_parameter('active_seconds', 360.)
        self.declare_parameter('require_localization', False)
        self.localized = False
        self.localized_at = None
        self.create_subscription(Bool, '/localization/ready', self.on_localized, 10)
        self.allowed = False
        self.last_tick = self.get_clock().now()
        self.elapsed = 0.0
        self.finished = False
        self.stop_future = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Bool, '/match/active', qos)
        self.finished_pub = self.create_publisher(Bool, '/real/match_finished', qos)
        self.create_subscription(Bool, '/match/allowed', self.on_allowed, qos)
        self.create_subscription(PlanningIntent, '/navigation/intent', self.on_intent, 10)
        self.client = self.create_client(SetBool, '/match/allow_motion')
        self.create_timer(.1, self.tick)

    def on_localized(self, msg):
        self.localized = msg.data
        self.localized_at = self.get_clock().now()

    def on_allowed(self, msg):
        self.allowed = msg.data

    def on_intent(self, msg):
        if self.allowed and msg.reason == 'guardian start center reached':
            self.finished = True

    def tick(self):
        now = self.get_clock().now()
        ready = not self.get_parameter('require_localization').value or (
            self.localized and self.localized_at is not None and
            (now-self.localized_at).nanoseconds < 1_000_000_000)
        if self.allowed and ready and not self.finished:
            self.elapsed += max(0., (now-self.last_tick).nanoseconds*1e-9)
        if self.stop_future is not None and self.stop_future.done():
            self.stop_future = None
        self.last_tick = now
        if self.elapsed >= self.get_parameter('active_seconds').value:
            self.finished = True
        self.pub.publish(Bool(data=self.allowed and ready and not self.finished))
        self.finished_pub.publish(Bool(data=self.finished))
        if (self.finished or not ready) and self.allowed and self.client.service_is_ready() and self.stop_future is None:
            self.stop_future = self.client.call_async(SetBool.Request(data=False))


def main():
    rclpy.init()
    node = RealMatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): node.pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
