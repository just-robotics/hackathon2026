#!/usr/bin/env python3
"""Stationary sensor replay through real ROS nodes and native MPPI, no drivers.

The recorded start pose never moves. Intermediate control availability is
measured, not navigation success. No final cmd_vel publisher is launched.
Run in an isolated network-none container, not on hardware.
"""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/hsl_perception'),str(ROOT/'src/hsl_real')]
from audit_real_bags import messages
from hsl_perception.profiles import REAL_PARAMETERS
from hsl_real.lidar_filter_core import DEFAULT_PARAMETERS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--baseline',action='store_true')
    parser.add_argument('--seconds',type=float,default=12.)
    parser.add_argument('--costmap-source',choices=['static','memory'],default='static',help='isolate the static versus observed-memory input to MPPI')
    args=parser.parse_args()
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid,Odometry
    from nav2_msgs.msg import Costmap
    import numpy as np
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String,Bool
    from tf2_msgs.msg import TFMessage
    from hsl_interfaces.msg import PlanningIntent
    files=sorted((args.session/'bag').glob('*.mcap'))
    grid=None;statics=[];clouds=[];pose=None
    for file in files:
        for topic,msg,_ in messages(file,['/map','/tf_static','/navigation/self','/livox/lidar']):
            if topic=='/map':grid=msg
            elif topic=='/tf_static':statics.extend(msg.transforms)
            elif topic=='/navigation/self' and pose is None:pose=msg.pose.pose
            elif topic=='/livox/lidar' and len(clouds)<20:clouds.append(msg)
    if grid is None or pose is None or not clouds:raise RuntimeError('missing start observations')
    # Re-anchor the start on wall time, with exactly the recorded static mounts.
    for parent,child,p in [('map','odom',pose),('odom','base_footprint',None)]:
        transform=TransformStamped();transform.header.frame_id=parent;transform.child_frame_id=child
        if p is not None:
            transform.transform.translation.x=p.position.x;transform.transform.translation.y=p.position.y
            transform.transform.rotation=p.orientation
        else:transform.transform.rotation.w=1.
        statics.append(transform)
    rclpy.init();node=rclpy.create_node('real_start_replay')
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tfpub=node.create_publisher(TFMessage,'/tf_static',latched)
    map_pub=node.create_publisher(OccupancyGrid,'/map',latched)
    odom_pub=node.create_publisher(Odometry,'/odom',10)
    lidar_pub=node.create_publisher(PointCloud2,'/livox/lidar',qos_profile_sensor_data)
    intent_pub=node.create_publisher(PlanningIntent,'/navigation/intent',10)
    active_pub=node.create_publisher(Bool,'/match/active',latched)
    diagnostics=[];detector=[];filter_rows=[];ready=[];near_costmap=[]
    def costmap_sample(msg):
        metadata=msg.metadata; r=metadata.resolution; o=metadata.origin.position
        a=np.asarray(msg.data).reshape(metadata.size_y,metadata.size_x)
        y,x=np.where(a==254)
        distances=np.hypot(o.x+(x+.5)*r-pose.position.x,o.y+(y+.5)*r-pose.position.y)
        near_costmap.append(int(np.sum(distances<.27)))
    subscriptions=[node.create_subscription(String,'/navigation/mppi_diagnostics',lambda m:diagnostics.append(json.loads(m.data)),10),
        node.create_subscription(String,'/navigation/detector_diagnostics',lambda m:detector.append(json.loads(m.data)),10),
        node.create_subscription(String,'/sensing/lidar/filter_diagnostics',lambda m:filter_rows.append(json.loads(m.data)),10),
        node.create_subscription(Bool,'/navigation/native_ready',lambda m:ready.append(m.data),10)]
    subscriptions.append(node.create_subscription(Costmap,'/native_mppi/costmap_raw',costmap_sample,10))
    environment=os.environ.copy()
    environment['PYTHONPATH']=':'.join(str(ROOT/'src'/p) for p in ('hsl_real','hsl_planning','hsl_perception','hsl_sim_adapter'))+':'+environment.get('PYTHONPATH','')
    commands=[[str(Path(get_package_prefix('hsl_lidar_filter'))/'lib/hsl_lidar_filter/real_lidar_filter')],
        ['python3','-c','from hsl_real.observations import main; main()','--ros-args','-p','lidar_topic:=/sensing/lidar/points_filtered'],
        [str(Path(get_package_prefix('hsl_perception_cpp'))/'lib/hsl_perception_cpp/opponent_detector')],
        ['python3','-c','from hsl_planning.node import main; main()','--ros-args','-p','role:=explorer'],
        [str(Path(get_package_prefix('hsl_nav2_control'))/'lib/hsl_nav2_control/native_mppi'),'--ros-args','--params-file',str(ROOT/'src/hsl_nav2_control/config/native_mppi.yaml'),'-p','use_sim_time:=false',
         '-p','costmap.obstacle_layer.cloud.marking:=true']]
    commands[-1] += ['-p','costmap.static_layer.map_topic:='+('/map' if args.costmap_source=='static' else '/navigation/obstacle_grid')]
    detector_params=dict(REAL_PARAMETERS,sensor_frame='livox')
    lidar_params=dict(DEFAULT_PARAMETERS)
    if args.baseline:
        detector_params={'sensor_frame':'livox_frame'}
        lidar_params.update(self_occlusion_max_range=0.,filter_low_confidence=False)
    param_file=args.output.with_suffix('.params.yaml');param_file.parent.mkdir(parents=True,exist_ok=True)
    param_file.write_text(yaml.safe_dump({'opponent_detector':{'ros__parameters':detector_params},
        'real_lidar_filter':{'ros__parameters':lidar_params}}))
    for index in (0,2):commands[index]+=['--ros-args','--params-file',str(param_file)]
    log=args.output.with_suffix('.ros.log').open('w');processes=[]
    try:
        processes=[subprocess.Popen(command,env=environment,stdout=log,stderr=subprocess.STDOUT) for command in commands]
        start=time.monotonic();last=-1;last_static=-1;index=0
        while time.monotonic()-start<args.seconds:
            rclpy.spin_once(node,timeout_sec=.02)
            if any(process.poll() is not None for process in processes):raise RuntimeError('ROS child exited; inspect log')
            now=time.monotonic()
            if now-last<.05:continue
            last=now;stamp=node.get_clock().now().to_msg()
            if now-last_static>=1.:
                last_static=now
                for transform in statics:transform.header.stamp=stamp
                tfpub.publish(TFMessage(transforms=statics));grid.header.stamp=stamp;map_pub.publish(grid)
            odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_footprint';odom.header.stamp=stamp;odom.pose.pose.orientation.w=1.;odom_pub.publish(odom)
            cloud=deepcopy(clouds[index%len(clouds)]);cloud.header.stamp=stamp;lidar_pub.publish(cloud);index+=1
            intent=PlanningIntent();intent.header.frame_id='map';intent.header.stamp=stamp
            intent.behavior=PlanningIntent.GOAL;intent.has_target=True;intent.target.position.x=.5;intent.target.position.y=3.5;intent.target.orientation.w=1.
            intent.max_speed=.5;intent.target_tolerance=.08;intent.reason='stationary start replay, no drivers'
            intent_pub.publish(intent);active_pub.publish(Bool(data=True))
        results={result:sum(d['result']==result for d in diagnostics) for result in sorted(set(d['result'] for d in diagnostics))}
        moving=[d for d in diagnostics if abs(d.get('first_speed_mps',0))>.05]
        report=dict(scope='stationary sensor replay, no drivers or final cmd_vel; not a navigation outcome',
            baseline=args.baseline,costmap_source=args.costmap_source,near_lethal_costmap_cells=near_costmap,detector_diagnostics=len(detector),filter_diagnostics=len(filter_rows),
            native_ready_samples=sum(ready),native_results=results,moving_commands=len(moving),
            max_command_speed_mps=max((abs(d.get('first_speed_mps',0)) for d in diagnostics),default=0),
            diagnostics=diagnostics,detector_reports=detector)
        args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ('diagnostics','detector_reports')}))
    finally:
        for process in processes:process.terminate()
        for process in processes:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
