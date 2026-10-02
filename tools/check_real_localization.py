"""Offline ROS smoke: raw Livox bag + deliberately biased LIO-derived odometry.

Run in an isolated real image WITHOUT drivers/devices. Reference and map use
one recording, so errors are consistency measurements, not physical accuracy.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
import rosbag2_py


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag');p.add_argument('registered');p.add_argument('quality');p.add_argument('output')
    p.add_argument('--config',default='/config/real.yaml');p.add_argument('--rate',type=float,default=2.)
    p.add_argument('--exercise-permission',action='store_true',help='Enable autonomous commands ONLY in the isolated driver-free test')
    p.add_argument('--max-clouds',type=int,default=0,help='Short integration check; 0 replays the entire bag')
    p.add_argument('--session-dir',default='',help='Optional isolated diagnostics output under /records')
    args=p.parse_args()
    quality=json.loads(Path(args.quality).read_text())
    rot=np.asarray(quality['raw_to_map_rotation']);shift=np.asarray(quality['raw_xy_shift'])
    trajectory=np.loadtxt(Path(args.registered)/'trajectory.csv',delimiter=',',skiprows=1)
    reference_time=trajectory[:,0]
    params=yaml.safe_load((Path(args.registered)/'parameters.yaml').read_text())['/**']['ros__parameters']
    lidar=trajectory[:,1:4]+Rotation.from_quat(trajectory[:,4:8]).apply(params['mapping']['extrinsic_T'])
    positions=(lidar-shift)@rot.T
    rotations=Rotation.from_matrix(rot)*Rotation.from_quat(trajectory[:,4:8])
    slerp=Slerp(reference_time,rotations)
    reader=rosbag2_py.SequentialReader()
    bag=next(Path(args.bag).glob('*.mcap'))
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    rclpy.init();node=rclpy.create_node('localization_replay_check')
    cloud_pub=node.create_publisher(PointCloud2,'/livox/lidar',qos_profile_sensor_data)
    odom_pub=node.create_publisher(Odometry,'/odom',10);tf=TransformBroadcaster(node)
    state={'ready':False,'active':False,'pose':None,'self':None,'allowed':None,'cmd':None}
    subscriptions=[node.create_subscription(Bool,'/match/allowed',lambda m:state.update(allowed=m.data),10),
        node.create_subscription(Twist,'/cmd_vel',lambda m:state.update(cmd=m),10),node.create_subscription(Bool,'/localization/ready',lambda m:state.update(ready=m.data),10),
        node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:state.update(pose=m),10),
        node.create_subscription(Odometry,'/navigation/self',lambda m:state.update(self=m),10),
        node.create_subscription(Bool,'/match/active',lambda m:state.update(active=m.data),10)]
    log_path=Path(args.output).with_suffix('.log');log_path.parent.mkdir(parents=True,exist_ok=True)
    log=log_path.open('w')
    process=subprocess.Popen(['ros2','launch','hsl_real','robot.launch.py',
        'drivers_enabled:=false','config_file:='+args.config,'session_dir:='+args.session_dir],stdout=log,stderr=subprocess.STDOUT)
    rows=[];published=0;ready_samples=0;enable_future=None;active_seen=False
    client=node.create_client(SetBool,'/match/allow_motion')
    try:
        startup=time.monotonic()+8.
        while time.monotonic()<startup:rclpy.spin_once(node,timeout_sec=.05)
        begin=time.monotonic();first=None
        while reader.has_next() and (not args.max_clouds or published<args.max_clouds):
            if process.poll() is not None:raise RuntimeError('Real launch exited; inspect replay log')
            topic,data,_=reader.read_next()
            if topic!='/livox/lidar':continue
            cloud=deserialize_message(data,PointCloud2)
            stamp=cloud.header.stamp.sec+cloud.header.stamp.nanosec*1e-9
            if first is None:first=stamp
            deadline=begin+(stamp-first)/args.rate
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
            t=np.clip(stamp,reference_time[0],reference_time[-1])
            x,y=[np.interp(t,reference_time,positions[:,k]) for k in range(2)]
            yaw=float(slerp(t).as_euler('xyz')[2])
            fraction=(t-reference_time[0])/(reference_time[-1]-reference_time[0])
            # Add up to 30cm / -20cm drift while leaving the initial pose intact.
            ox,oy=x+.30*fraction,y-.20*fraction
            header=node.get_clock().now().to_msg()
            odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_footprint';odom.header.stamp=header
            odom.pose.pose.position.x=ox;odom.pose.pose.position.y=oy
            odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
            tr=TransformStamped();tr.header=odom.header;tr.child_frame_id=odom.child_frame_id
            tr.transform.translation.x=ox;tr.transform.translation.y=oy;tr.transform.rotation=odom.pose.pose.orientation
            tf.sendTransform(tr);odom_pub.publish(odom)
            cloud.header.stamp=header;cloud_pub.publish(cloud);published+=1
            until=time.monotonic()+.005
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.001)
            if state['ready']:ready_samples+=1
            active_seen = active_seen or state['active']
            if args.exercise_permission and state['ready'] and published>100 and enable_future is None and client.service_is_ready():
                enable_future=client.call_async(SetBool.Request(data=True))
            own=state['self']
            if own is not None:
                # Compare against the reference at this output's replay time;
                # account for asynchronous transport using wall-clock stamps.
                output_wall=own.header.stamp.sec+own.header.stamp.nanosec*1e-9
                current_wall=header.sec+header.nanosec*1e-9
                tt=np.clip(t-(current_wall-output_wall)*args.rate,reference_time[0],reference_time[-1])
                rx,ry=[np.interp(tt,reference_time,positions[:,k]) for k in range(2)]
                err=math.hypot(own.pose.pose.position.x-rx,own.pose.pose.position.y-ry)
                rows.append([float(t-first),err,bool(state['ready'])])
        ready_before_loss=state['ready']
        until=time.monotonic()+2.
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        error=np.array([r[1] for r in rows if r[2]])
        report=dict(raw_clouds_published=published,ready_fraction=ready_samples/max(1,published),
            ready_before_sensor_loss=ready_before_loss,ready_after_sensor_loss=state['ready'],
            navigation_samples=len(rows),position_median_m=float(np.median(error)) if len(error) else None,
            position_p95_m=float(np.percentile(error,95)) if len(error) else None,
            injected_final_odom_error_m=math.hypot(.3,.2)*float(fraction),
            cmd_vel_publishers=[i.node_name for i in node.get_publishers_info_by_topic('/cmd_vel')],
            movement_active_after_sensor_loss=state['active'],
            permission_exercised=args.exercise_permission,active_seen=active_seen,
            permission_allowed_after_sensor_loss=state['allowed'],
            final_cmd=[state['cmd'].linear.x,state['cmd'].angular.z] if state['cmd'] is not None else None,
            reference='LIO from the same bag as map, not independent ground truth',samples=rows)
        Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
        if not ready_before_loss or state['ready'] or not len(error):raise RuntimeError('Localization readiness/replay failed')
        if args.exercise_permission and (state['allowed'] is not False or report['final_cmd']!=[0.,0.]):raise RuntimeError('Permission or final velocity not cleared on sensor loss')
        if args.exercise_permission and not active_seen:raise RuntimeError('Motion permission did not become active')
        if report['cmd_vel_publishers']!=['hsl_motion_gate'] or state['active']:raise RuntimeError('Motion isolation failed')
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=15)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

    if args.session_dir:
        metadata=Path(args.session_dir)/'bag/metadata.yaml'
        if not metadata.is_file():raise RuntimeError('Diagnostic bag was not finalized')
        report['diagnostic_bag_finalized']=True
        Path(args.output).write_text(json.dumps(report,indent=2))
        print('Diagnostic bag finalized: '+str(metadata))


if __name__=='__main__':main()
