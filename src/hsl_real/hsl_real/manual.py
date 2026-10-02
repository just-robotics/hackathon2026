"""Single final velocity publisher for keyboard recording; receipt watchdog."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class ManualGate(Node):
    def __init__(self):
        super().__init__('real_manual_gate')
        self.timeout=float(self.declare_parameter('timeout_s',.6).value)
        self.linear=float(self.declare_parameter('max_speed',.5).value)
        self.angular=float(self.declare_parameter('max_angular_speed',1.).value)
        self.allowed=True
        self.command=Twist()
        self.received=-math.inf
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        self.create_subscription(Twist,'/real/keyboard_cmd_vel',self.receive,10)
        self.create_service(SetBool,'/real/manual_allow_motion',self.allow)
        self.create_timer(.05,self.tick)

    def receive(self,msg):
        if not all(math.isfinite(v) for v in (msg.linear.x,msg.angular.z)):return
        self.command=Twist()
        self.command.linear.x=max(-self.linear,min(self.linear,msg.linear.x))
        self.command.angular.z=max(-self.angular,min(self.angular,msg.angular.z))
        self.received=time.monotonic()

    def tick(self):
        fresh=time.monotonic()-self.received<=self.timeout
        self.publisher.publish(self.command if self.allowed and fresh else Twist())

    def allow(self,req,res):
        self.allowed=req.data
        self.received=-math.inf  # resuming never replays a previous held key
        self.publisher.publish(Twist())
        res.success=True;res.message='Manual motion enabled' if req.data else 'Manual motion stopped'
        return res


def main():
    rclpy.init();node=ManualGate()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
