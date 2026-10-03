"""Isolated ROS check: AMCL corrects FAST-LIO odometry; stale correction stops output.
No drivers, controller or cmd_vel. Run inside the real image with /work mounted.
"""
import json
import math
import subprocess
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def main():
    rclpy.init()
    node=Node('map_registration_check')
    children=[]
    logs=[]
    try:
        for executable in [('nav2_amcl','amcl'),('hsl_real','map_kinematic_state'),('hsl_real','localization_monitor')]:
            log=open('/tmp/'+executable[1]+'-check.log','w');logs.append(log)
            args=['ros2','run',*executable,'--ros-args','--params-file','/work/config/localization.yaml']
            if executable[1]=='amcl':
                args+=['-p','odom_frame_id:=lio_odom','-p','initial_pose.x:=0.62',
                       '-p','initial_pose.y:=0.57','-p','initial_pose.yaw:=0.10',
                       '-r','scan:=/localization/scan']
            if executable[1]=='localization_monitor':
                args+=['-p','mode:=fastlio','-p','initial_x:=0.62','-p','initial_y:=0.57','-p','initial_yaw:=0.10']
            children.append(subprocess.Popen(args,stdout=log,stderr=log))
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub=node.create_publisher(OccupancyGrid,'/map',qos)
        raw_pub=node.create_publisher(PointCloud2,'/livox/lidar',10)
        imu_pub=node.create_publisher(Imu,'/livox/imu',10)
        wheel_pub=node.create_publisher(Odometry,'/odom',10)
        ready=[]
        node.create_subscription(Bool,'/localization/ready',lambda m:ready.append(m.data),10)
        scan_pub=node.create_publisher(LaserScan,'/localization/scan',10)
        pose_pub=node.create_publisher(Odometry,'/localization/lio_odometry',10)
        tf_pub=TransformBroadcaster(node)
        states=[]
        amcl=[]
        node.create_subscription(PoseWithCovarianceStamped,"/amcl_pose",lambda m:amcl.append(m),10)
        node.create_subscription(Odometry,'/localization/kinematic_state',lambda m:states.append(
            (time.monotonic(),m.pose.pose.position.x,m.pose.pose.position.y,
             m.header.frame_id,m.child_frame_id)),10)
        data=Path('/work/config/maps/maze_bag_v1.pgm').read_bytes().split(b'\n',3)
        w,h=map(int,data[1].split());pixels=np.flipud(np.frombuffer(data[3],np.uint8).reshape(h,w))
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=w;grid.info.height=h
        grid.info.resolution=.05;grid.info.origin.position.x=-.35;grid.info.origin.position.y=-.5
        grid.info.origin.orientation.w=1.;grid.data=np.where(pixels<90,100,0).astype(np.int8).reshape(-1).tolist()
        ranges=[]
        for angle in np.linspace(-math.pi,math.pi,720,endpoint=False):
            distance=8.
            for r in np.arange(.25,8.,.01):
                x,y=.5+r*math.cos(angle),.5+r*math.sin(angle)
                ix,iy=int(math.floor((x+.35)/.05)),int(math.floor((y+.5)/.05))
                if not (0<=ix<w and 0<=iy<h):break
                if pixels[iy,ix]<90:distance=float(r);break
            ranges.append(distance)
        time.sleep(2.)
        for transition in ('configure','activate'):
            subprocess.run(['ros2','lifecycle','set','/amcl',transition],check=True,timeout=12)
        for step in range(200):
            stamp=node.get_clock().now().to_msg()
            grid.header.stamp=stamp
            if step==0:map_pub.publish(grid)
            odom=Odometry();odom.header.frame_id='lio_odom';odom.child_frame_id='base_footprint'
            odom.header.stamp=stamp;odom.pose.pose.position.x=odom.pose.pose.position.y=.5
            odom.pose.pose.orientation.w=1.;odom.pose.covariance[0]=odom.pose.covariance[7]=1e-4
            tf=TransformStamped();tf.header=odom.header;tf.child_frame_id=odom.child_frame_id
            tf.transform.translation.x=tf.transform.translation.y=.5;tf.transform.rotation.w=1.
            tf.child_frame_id='odom'
            wheel_tf=TransformStamped();wheel_tf.header.stamp=stamp;wheel_tf.header.frame_id='odom'
            wheel_tf.child_frame_id='base_footprint';wheel_tf.transform.rotation.w=1.
            tf_pub.sendTransform([tf,wheel_tf]);pose_pub.publish(odom)
            wheel=Odometry();wheel.header.stamp=stamp;wheel.header.frame_id='odom';wheel.pose.pose.orientation.w=1.
            wheel_pub.publish(wheel)
            raw=PointCloud2();raw.header.stamp=stamp;raw.header.frame_id='livox';raw_pub.publish(raw)
            imu=Imu();imu.header.stamp=stamp;imu_pub.publish(imu)
            if step<150:
                scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_footprint'
                scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/720
                scan.angle_max=scan.angle_min+719*scan.angle_increment
                scan.range_min=.25;scan.range_max=8.;scan.ranges=ranges;scan_pub.publish(scan)
                if step%5==0:
                    subprocess.Popen(['ros2','service','call','/request_nomotion_update','std_srvs/srv/Empty','{}'],
                                     stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            rclpy.spin_once(node,timeout_sec=.1)
            time.sleep(.04)
        end=time.monotonic()
        print('amcl poses:',len(amcl),flush=True)
        if amcl:
            print('last amcl:',amcl[-1],flush=True)
        assert len(states)>20, f'No map-corrected state: {len(states)}'
        last=states[-1]
        error=math.hypot(last[1]-.5,last[2]-.5)
        stale_gap=end-last[0]
        result=dict(states=len(states),last_xy=last[1:3],error_m=error,
                    stale_gap_s=stale_gap,initial_error_m=math.hypot(.12,.07))
        print(json.dumps(result))
        assert all(s[3:] == ('map','base_footprint') for s in states)
        assert error<.08,result
        assert stale_gap>1.,result
        assert any(ready) and not ready[-1], f'Incorrect readiness: {ready}'
    finally:
        for child in children:
            child.terminate()
        for child in children:
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        for log in logs:
            log.close()
            print(Path(log.name).read_text()[-8000:])
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
