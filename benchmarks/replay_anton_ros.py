#!/usr/bin/env python3
"""Native ROS crop/DBSCAN plus the unchanged Anton detector on cached real scans."""
import argparse,array,csv,json,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from jr_perception.robot_detector import RobotDetector
from replay_cached_baseline import Zstd,cloud_from_cache,occupancy_from_cache,sensor_pose
from real_detector_cache import sensor_tf,inverse,compose
from smoke_cpp_detector_ros import validate_cache

class Forward:
    def __init__(self,publisher,callback):self.publisher=publisher;self.callback=callback
    def publish(self,message):self.callback(message);self.publisher.publish(message)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('cache',type=Path)
    p.add_argument('--profile',type=Path,required=True);p.add_argument('--crop-profile',type=Path,required=True)
    p.add_argument('--dbscan-profile',type=Path,required=True);p.add_argument('--source-bag',type=Path)
    p.add_argument('--rate',type=float,default=1);p.add_argument('--from-seconds',type=float,default=0)
    p.add_argument('--count',type=int,default=0);p.add_argument('--expect',default='any')
    p.add_argument('--log',type=Path,required=True);p.add_argument('--rviz-config',type=Path)
    p.add_argument('--rviz-log',type=Path);p.add_argument('--hold-seconds',type=float,default=0)
    a=p.parse_args();manifest=json.loads((a.cache/'manifest.json').read_text())
    provenance,local=validate_cache(a.cache,manifest,a.source_bag)
    world='odom' if local else 'map'
    rclpy.init(args=['--ros-args','--params-file',str(a.profile),'-p','use_sim_time:=true',
                     '-p','pose_topic:=/anton_replay/self_pose','-p',f'world_frame:={world}'])
    detector=RobotDetector()
    if local:detector.map_topic='' # Explicit mapless local-odom ablation, never production launch.
    bridge=Node('anton_replay');executor=SingleThreadedExecutor();executor.add_node(detector);executor.add_node(bridge)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    static_pub=bridge.create_publisher(TFMessage,'/tf_static',latched)
    tf_pub=bridge.create_publisher(TFMessage,'/tf',10)
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid
    cloud_pub=bridge.create_publisher(PointCloud2,'/sensing/lidar/points_filtered',qos_profile_sensor_data)
    pose_pub=bridge.create_publisher(Odometry,'/anton_replay/self_pose',qos_profile_sensor_data)
    map_pub=bridge.create_publisher(OccupancyGrid,'/map',latched)
    clock_pub=bridge.create_publisher(Clock,'/clock',10)
    static,dynamic={},{};statics=[]
    for line in (a.cache/'tf.jsonl').open():
        v=json.loads(line);edge=(v['parent'],v['child']);value=(tuple(v['translation']),tuple(v['quaternion']))
        if v['topic']=='/tf_static':
            static[edge]=value;t=TransformStamped();t.header.frame_id=edge[0];t.child_frame_id=edge[1]
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=value[0]
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=value[1]
            statics.append(t)
        else:dynamic.setdefault(edge,[]).append((v['header_ns'],*value,None))
    for samples in dynamic.values():samples.sort()
    if not local:map_pub.publish(occupancy_from_cache(a.cache))
    static_pub.publish(TFMessage(transforms=statics))
    rows=[r for r in csv.DictReader((a.cache/'frames.tsv').open(),delimiter='\t') if r['topic']=='filtered' and r['pose_status']=='ok']
    if a.count:rows=rows[:a.count]
    first=int(rows[0]['header_ns']);clock=Clock();clock.clock.sec=first//10**9;clock.clock.nanosec=first%10**9;clock_pub.publish(clock)
    children=[];rviz_file=None
    received={};observations={};track_ids={}
    def odom(message):
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        track=detector.tracker.selected;track_ids.setdefault(track,len(track_ids)+1)
        observations[stamp]={'xy':[message.pose.pose.position.x,message.pose.pose.position.y],
                             'measured':abs(track.last_update-stamp/10**9)<1e-5,'track_id':track_ids[track]}
    detector.odometry_publisher=Forward(detector.odometry_publisher,odom)
    detector.foreground_publisher=Forward(detector.foreground_publisher,lambda m:received.__setitem__(m.header.stamp.sec*10**9+m.header.stamp.nanosec,m.width))
    try:
        children.append(subprocess.Popen(['ros2','run','robot_body_filter','robot_body_filter_node','--ros-args',
          '--params-file',str(a.crop_profile),'-p','use_sim_time:=true','-p','keep_input_frame:=true',
          '-p','static_boxes.body.min_x:=-0.25','-p','static_boxes.body.min_y:=-0.25',
          '-p','static_boxes.body.max_x:=0.25','-p','static_boxes.body.max_y:=0.25',
          '-r','~/input/pointcloud:=/sensing/lidar/points_filtered','-r','~/output/pointcloud:=/sensing/lidar/cropped/pointcloud']))
        children.append(subprocess.Popen(['ros2','run','dbscan_filter','dbscan_filter_node','--ros-args',
          '--params-file',str(a.dbscan_profile),'-p','use_sim_time:=true',
          '-r','~/input/pointcloud:=/sensing/lidar/cropped/pointcloud','-r','~/output/pointcloud:=/sensing/lidar/dbscan/pointcloud']))
        ready=time.monotonic()+15
        while time.monotonic()<ready:
            executor.spin_once(timeout_sec=.05)
            if cloud_pub.get_subscription_count() and bridge.count_publishers('/sensing/lidar/dbscan/pointcloud'):
                if all(c.poll() is None for c in children):break
        else:raise RuntimeError('native filters did not start')
        # Let transient TF reach the filter's startup buffer before the first input.
        end=time.monotonic()+.5
        while time.monotonic()<end:executor.spin_once(timeout_sec=.02)
        if a.rviz_config:
            rviz_file=(a.rviz_log or a.log.with_suffix('.rviz.log')).open('w')
            children.append(subprocess.Popen(['rviz2','-d',str(a.rviz_config),'-f',world,'--ros-args','-p','use_sim_time:=true'],stdout=rviz_file,stderr=subprocess.STDOUT))
        zstd=Zstd();start=time.monotonic();paced_start=None
        with (a.cache/'clouds.zstbin').open('rb') as binary:
            for row in rows:
                stamp=int(row['header_ns']);rel=(stamp-first)/10**9
                if rel>=a.from_seconds:
                    if paced_start is None:paced_start=time.monotonic()-(rel-a.from_seconds)/a.rate
                    due=paced_start+(rel-a.from_seconds)/a.rate
                    while time.monotonic()<due:executor.spin_once(timeout_sec=min(.02,due-time.monotonic()))
                binary.seek(int(row['offset']));payload=zstd.decompress(binary.read(int(row['compressed_size'])),int(row['uncompressed_size']))
                cloud=cloud_from_cache(row,manifest['layouts'][int(row['layout_id'])],payload)
                local_sensor,status,_=sensor_tf(static,dynamic,row['frame_id'],stamp)
                if status!='ok':raise RuntimeError('missing sensor TF')
                world_sensor=(tuple(float(row['map_sensor_'+k]) for k in 'xyz'),tuple(float(row['map_sensor_q'+k]) for k in 'xyzw'))
                position,quaternion=compose(world_sensor,inverse(local_sensor))
                pose=Odometry();pose.header.stamp=cloud.header.stamp;pose.header.frame_id=world;pose.child_frame_id='base_footprint'
                pose.pose.pose.position.x,pose.pose.pose.position.y,pose.pose.pose.position.z=position
                pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w=quaternion
                t=TransformStamped();t.header=pose.header;t.child_frame_id='base_footprint'
                t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=position
                t.transform.rotation=pose.pose.pose.orientation
                clock.clock=cloud.header.stamp;clock_pub.publish(clock);tf_pub.publish(TFMessage(transforms=[t]));pose_pub.publish(pose)
                for _ in range(3):executor.spin_once(timeout_sec=.005)
                cloud_pub.publish(cloud)
                timeout=time.monotonic()+5
                while stamp not in received and time.monotonic()<timeout:
                    executor.spin_once(timeout_sec=.02)
                if stamp not in received:raise RuntimeError(f'no detector acknowledgement for {stamp}')
        report={'provenance':provenance,'mapless_local_odom_ablation':local,'sent':len(rows),'processed':len(received),
                'odometry':len(observations),'measured':sum(v['measured'] for v in observations.values()),
                'prediction':sum(not v['measured'] for v in observations.values()),'elapsed_s':time.monotonic()-start,
                'observations':{str(k):v for k,v in observations.items()}}
        a.log.parent.mkdir(parents=True,exist_ok=True);a.log.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('observations','provenance')}),flush=True)
        until=time.monotonic()+a.hold_seconds
        while a.hold_seconds<0 or time.monotonic()<until:
            executor.spin_once(timeout_sec=.05)
            if a.rviz_config and children[-1].poll() is not None:break
    finally:
        for child in children:
            if child.poll() is None:child.terminate()
        for child in children:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        if rviz_file:rviz_file.close()
        executor.shutdown();detector.destroy_node();bridge.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
