#!/usr/bin/env python3
"""Read-only replay of unchanged detector callbacks on raw LiDAR and recorded map poses.

No commands are published. Selection coverage is not semantic accuracy.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from jr_perception.robot_detector import RobotDetector, stamp_seconds
from ament_index_python.packages import get_package_share_directory


def reader(path, topics):
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in topics}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    while r.has_next():
        topic,data,stamp=r.read_next()
        yield topic,deserialize_message(data,types[topic]),stamp


class Collect:
    def __init__(self, callback):self.callback=callback
    def publish(self, message):self.callback(message)


def replay(path, pose_source="recorded", cloud_topic="/livox/lidar"):
    topics={'/map','/tf_static','/tf',cloud_topic, '/odom' if pose_source=='relay' else '/navigation/self'}
    grid=None;statics=[]
    for topic,m,_ in reader(path,{'/map','/tf_static'}):
        if topic=='/map':grid=m
        else:statics.extend(m.transforms)
    if grid is None:
        return dict(bag=str(path),skipped='No recorded /map or map pose; no localization invented')
    n=RobotDetector();n.on_map(grid)
    for tf in statics:n.tf_buffer.set_transform_static(tf,'recorded_bag')
    relay=None
    if pose_source=='relay':
        from jr_perception.pose_relay import PoseRelay
        relay=PoseRelay()
    rows=[];own=[];counts=dict(raw=0,poses=0,processed=0,selected=0,measured_selected=0,marker_arrays=0,object_marker_frames=0)
    pending={}
    original_step=n.tracker.step
    def observed_step(moment, detections):
        selected=original_step(moment,detections)
        pending['candidates']=[dict(xy=d.center.tolist(),strong=d.strong,sigma=d.sigma) for d in detections]
        confirmed=[track for track in n.tracker.tracks if track.confirmed]
        has_reacquire_timeout=hasattr(n.tracker,'robot_last_seen')
        timeout_expired=has_reacquire_timeout and (n.tracker.robot_last_seen is None or
            moment-n.tracker.robot_last_seen>=n.tracker.config.reacquire_timeout)
        reliable=[track for track in confirmed if (not has_reacquire_timeout or
            (track.strong_streak>=n.tracker.config.reacquire_strong_hits and
             moment-track.last_update<=n.tracker.config.tentative_coast))]
        pending['tracker_state']=dict(confirmed=len(confirmed),moving=int(sum(track.moved for track in confirmed)),
            reliable=len(reliable),reacquire_timeout_expired=bool(timeout_expired),
            selected_before_map_gate=selected is not None,
            robot_position=None if n.tracker.robot_position is None else n.tracker.robot_position.tolist(),
            eligible_stationary=int(sum(n.tracker.robot_position is None or timeout_expired or
                np.linalg.norm(track.birth-n.tracker.robot_position)<=n.tracker.config.reacquire_radius
                for track in reliable)))
        if selected is not None:
            pending.update(measurement_age_s=moment-selected.last_update,
                strong_hits=selected.strong_hits,hits=selected.hits,moved=selected.moved,
                birth_xy=selected.birth.tolist(),last_measurement_xy=selected.last_position.tolist())
        return selected
    n.tracker.step=observed_step
    def odometry(m):
        p=m.pose.pose.position;stamp=stamp_seconds(m.header.stamp)
        selected=n.tracker.selected;measured=selected is not None and abs(selected.last_update-stamp)<1e-5
        counts['selected']+=1;counts['measured_selected']+=int(measured)
        pending.update(x=p.x,y=p.y,measured=measured)
    def foreground(m):
        counts['processed']+=1
        pending.update(stamp_s=stamp_seconds(m.header.stamp),foreground_points=m.width*m.height)
    def markers(m):
        clusters=[x for x in m.markers if x.ns=='clusters']
        counts['marker_arrays']+=1;counts['object_marker_frames']+=bool(clusters)
        rows.append(dict(pending,cluster_markers=len(clusters),
            strong_markers=sum(x.color.g>.7 and x.color.r<.2 for x in clusters)))
        pending.clear()
    def health(m):
        data=json.loads(m.data)
        pending.update(map_rejected_candidates=data.get('map_rejected_candidates',0),
                       map_rejected_track=data.get('map_rejected_track',False),
                       visibility_rejected_candidates=data.get('visibility_rejected_candidates',0),
                       visibility_rejected_track=data.get('visibility_rejected_track',False))
    n.odometry_publisher=Collect(odometry);n.foreground_publisher=Collect(foreground);n.marker_publisher=Collect(markers)
    n.health_publisher=Collect(health)
    def pose(m):
        counts['poses']+=1;n.on_pose(m);p=m.pose.pose.position
        own.append([stamp_seconds(m.header.stamp),p.x,p.y])
    if relay is not None: relay.publisher=Collect(pose)
    started=time.monotonic()
    try:
        for topic,m,_ in reader(path,topics):
            if topic=='/tf':
                if relay is not None: relay.on_tf(m)
                for tf in m.transforms:n.tf_buffer.set_transform(tf,'recorded_bag')
            elif topic=='/tf_static':
                for tf in m.transforms:n.tf_buffer.set_transform_static(tf,'recorded_bag')
            elif topic=='/navigation/self':
                pose(m)
            elif topic=='/odom' and relay is not None: relay.on_odom(m)
            elif topic==cloud_topic:counts['raw']+=1;n.on_cloud(m)
        stamps=[x['stamp_s'] for x in rows]
        coordinates=np.array([[x['x'],x['y']] for x in rows if 'x' in x])
        summary=dict(bag=str(path),counts=counts,wall_seconds=time.monotonic()-started,
            duration_s=max(stamps)-min(stamps) if stamps else 0,
            selection_share=counts['selected']/max(1,counts['processed']),
            measured_selection_share=counts['measured_selected']/max(1,counts['processed']),
            source=f'{cloud_topic}; {pose_source} map pose/recorded TF; production detector callbacks',
            semantic_accuracy='not measured: no object labels',map=dict(resolution=grid.info.resolution,
                origin=[grid.info.origin.position.x,grid.info.origin.position.y],width=grid.info.width,height=grid.info.height,data=list(grid.data)))
        if len(coordinates):
            median=np.median(coordinates,axis=0)
            summary.update(selected_median_xy=median.tolist(),selected_spread_p90_m=float(np.quantile(np.linalg.norm(coordinates-median,axis=1),.9)))
        return dict(summary=summary,frames=rows,own=own)
    finally:
        n.destroy_node()
        if relay is not None: relay.destroy_node()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bags',type=Path,nargs='+');p.add_argument('--output',required=True,type=Path)
    p.add_argument('--profile',type=Path,help='Detector ROS parameters; defaults to installed real.yaml')
    p.add_argument('--pose-source',choices=('recorded','relay'),default='recorded')
    p.add_argument('--cloud-topic',default='/livox/lidar')
    a=p.parse_args()
    profile=str(a.profile or Path(get_package_share_directory('jr_perception'))/'config/real.yaml')
    rclpy.init(args=['--ros-args','--params-file',profile,'-p','background_topic:=/map','-p','world_frame:=map','-p','pose_topic:=navigation/self','-p','base_frame:=base_footprint','-p','log_period:=10000.0','-p',f'cloud_topic:={a.cloud_topic}'])
    try:
        reports=[]
        for path in a.bags:
            report=replay(path,a.pose_source,a.cloud_topic);reports.append(report)
            compact=dict(report.get('summary',report));compact.pop('map',None);print(json.dumps(compact),flush=True)
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(reports))
    finally:rclpy.shutdown()


if __name__=='__main__':main()
