"""Run only in a driver-free, network-isolated real image; checks terminal ROS stage behavior."""

def main():
    import time,json
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool
    from nav_msgs.msg import Odometry
    from hsl_real.match import RealMatch
    rclpy.init();stage=RealMatch();stage.set_parameters([Parameter('goal_center',value=[.5,3.5])]);probe=rclpy.create_node('finish_probe');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    allowed=probe.create_publisher(Bool,'/match/allowed',q);pose=probe.create_publisher(Odometry,'/navigation/self',10);state={'active':None,'finished':None,'revocations':0}
    probe.create_subscription(Bool,'/match/active',lambda m:state.update(active=m.data),q);probe.create_subscription(Bool,'/real/match_finished',lambda m:state.update(finished=m.data),q)
    def permission(req,res):
     if not req.data:state['revocations']+=1
     allowed.publish(Bool(data=req.data));res.success=True;return res
    probe.create_service(SetBool,'/match/allow_motion',permission);e=SingleThreadedExecutor();e.add_node(probe);e.add_node(stage)
    def spin(seconds):
     end=time.monotonic()+seconds
     while time.monotonic()<end:e.spin_once(timeout_sec=.01)
    def position(x,y):
     msg=Odometry();msg.header.frame_id='map';msg.header.stamp=probe.get_clock().now().to_msg();msg.pose.pose.position.x=x;msg.pose.pose.position.y=y;pose.publish(msg);spin(.2)
    spin(.5);allowed.publish(Bool(data=True));spin(.3);position(.5,3.3);assert state['active'] is True and state['finished'] is False
    # A single pose in the 8cm circle must close the stage, even if the next
    # pose jitters outside and no terminal decision intent is ever published.
    position(.5,3.45);position(.5,3.3);assert state['finished'] is True and state['active'] is False and state['revocations']>0
    # Operator cannot accidentally resume a completed stage.
    allowed.publish(Bool(data=True));spin(.4);assert state['active'] is False
    print(json.dumps(state));e.shutdown();stage.destroy_node();probe.destroy_node();rclpy.shutdown()


if __name__ == "__main__":
    main()
