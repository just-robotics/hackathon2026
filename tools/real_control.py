"""Run inside the real container: explicitly enable or pause autonomous motion."""
import argparse
import json
import time


def main():
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import Bool
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from std_srvs.srv import SetBool
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['enable', 'pause'])
    parser.add_argument('--timeout', type=float, default=None)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('real_motion_control')
    state = {'native':False,'pose':None,'scan':None,'finished':None}
    qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(Bool,'/navigation/native_ready',
        lambda m:state.update(native=m.data),qos),
        node.create_subscription(Odometry,'/navigation/self',lambda m:state.update(pose=m.header.stamp),10),
        node.create_subscription(PointCloud2,'/navigation/scan',lambda m:state.update(scan=m.header.stamp),qos_profile_sensor_data)]
    subscriptions.append(node.create_subscription(Bool,'/real/match_finished',
        lambda m:state.update(finished=m.data),qos))
    client = node.create_client(SetBool,'/match/allow_motion')
    timeout = args.timeout if args.timeout is not None else (30 if args.action == 'enable' else 5)
    deadline = time.monotonic()+timeout
    def fresh(stamp, max_age):
        if stamp is None:return False
        age = node.get_clock().now().nanoseconds*1e-9-stamp.sec-stamp.nanosec*1e-9
        return 0 <= age <= max_age
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if not client.service_is_ready():continue
            if args.action=='enable' and state['finished'] is True:
                raise RuntimeError('Stage finished; place robot at configured start and use start_real for a new stage')
            if args.action=='enable' and not (
                    state['finished'] is False and state['native'] and fresh(state['pose'],1.2) and fresh(state['scan'],1.8)):
                continue
            if args.action=='enable':
                publishers = node.get_publishers_info_by_topic('/cmd_vel')
                if len(publishers)!=1 or publishers[0].node_name!='hsl_motion_gate':
                    raise RuntimeError('Expected exactly one /cmd_vel publisher: hsl_motion_gate')
            future = client.call_async(SetBool.Request(data=args.action=='enable'))
            rclpy.spin_until_future_complete(node,future,timeout_sec=5)
            if not future.done() or not future.result().success:
                raise RuntimeError('Motion permission service failed')
            print(json.dumps({'action':args.action,'success':True,'message':future.result().message}))
            return
        raise RuntimeError('Not ready: check driver logs, /odom, /livox/lidar, TF and native_ready; permission was not granted')
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
