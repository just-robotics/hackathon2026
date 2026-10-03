#!/usr/bin/env python3
"""Exercise native detector ROS inputs, stationary/moving track, and expiry."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tests'),str(ROOT/'src/hsl_sim_adapter'),str(ROOT/'src/hsl_perception')]
from test_shape_detector import robot_cloud
from hsl_perception.profiles import REAL_PARAMETERS
from hsl_sim_adapter.cloud import make_cloud


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    import rclpy
    from nav_msgs.msg import OccupancyGrid,Odometry
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header,Bool,String
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from tf2_ros import StaticTransformBroadcaster
    rclpy.init();node=rclpy.create_node('detector_port_fixture')
    grid_pub=node.create_publisher(OccupancyGrid,'navigation/known_grid',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    own_pub=node.create_publisher(Odometry,'navigation/self',10)
    scan_pub=node.create_publisher(PointCloud2,'navigation/scan',qos_profile_sensor_data)
    tracks=[];visible=[];diagnostics=[]
    subs=[node.create_subscription(Odometry,'navigation/opponent',tracks.append,10),node.create_subscription(Bool,'navigation/opponent_visible',lambda m:visible.append(m.data),10),node.create_subscription(String,'navigation/detector_diagnostics',lambda m:diagnostics.append(json.loads(m.data)),10)]
    broadcaster=StaticTransformBroadcaster(node)
    tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='livox';tf.transform.translation.y=1.;tf.transform.translation.z=.08;tf.transform.rotation.w=1.
    broadcaster.sendTransform(tf)
    grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05;grid.info.width=grid.info.height=100;grid.info.origin.position.x=grid.info.origin.position.y=-1.;grid.info.origin.orientation.w=1.;grid.data=[0]*10000;grid_pub.publish(grid)
    args=['ros2','run','hsl_perception_cpp','opponent_detector','--ros-args','-p','sensor_frame:=livox']
    for key,value in REAL_PARAMETERS.items():args+=['-p',key+':='+str(value).lower()]
    child=subprocess.Popen(args,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    try:
        until=time.monotonic()+3
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        # A current own pose must not discard a valid slightly older scan:
        # exact scan-time sensor TF is available; own pose is not its transform.
        stamp=node.get_clock().now().to_msg();own=Odometry();own.header=Header(stamp=stamp,frame_id='map');own.child_frame_id='base_footprint';own.pose.pose.orientation.w=1.;own_pub.publish(own)
        older=node.get_clock().now().nanoseconds-400_000_000
        scan_stamp=type(stamp)(sec=int(older//1_000_000_000),nanosec=int(older%1_000_000_000))
        expected=scan_stamp.sec+scan_stamp.nanosec*1e-9
        scan_pub.publish(make_cloud(Header(stamp=scan_stamp,frame_id='map'),robot_cloud((1.,1.))))
        until=time.monotonic()+.08
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
        assert any(abs(row['stamp_s']-expected)<1e-6 for row in diagnostics),'scan rejected solely due to newer own pose'
        for i in range(35):
            stamp=node.get_clock().now().to_msg();own=Odometry();own.header=Header(stamp=stamp,frame_id='map');own.child_frame_id='base_footprint';own.pose.pose.orientation.w=1.;own_pub.publish(own)
            scan_pub.publish(make_cloud(own.header,robot_cloud((1.+max(0,i-10)*.01,1.))))
            until=time.monotonic()+.10
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
        fresh_visible=any(visible)
        before=len(tracks);until=time.monotonic()+.7
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        report={'tracks':before,'diagnostics':len(diagnostics),'became_visible':fresh_visible,'expired_visibility':bool(visible and not visible[-1]),'no_predicted_publications':len(tracks)==before,'last_position':([tracks[-1].pose.pose.position.x,tracks[-1].pose.pose.position.y] if tracks else None)}
        assert before>=20 and len(diagnostics)>=25 and fresh_visible and report['expired_visibility'] and report['no_predicted_publications'],report
        assert all(t.header.frame_id=='map' and t.child_frame_id=='tracked_opponent/base_footprint' for t in tracks)
        until=time.monotonic()+.65
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        count=len(diagnostics)
        scan_pub.publish(make_cloud(Header(stamp=node.get_clock().now().to_msg(),frame_id='map'),robot_cloud((1.24,1.))))
        until=time.monotonic()+.15
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
        report['stale_own_blocks_processing']=len(diagnostics)==count
        assert report['stale_own_blocks_processing'],report
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    finally:
        child.terminate()
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
