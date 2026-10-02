#!/usr/bin/env python3
"""Read-only audit of real MCAP sessions. Run inside a ROS image, without DDS.

No inferred obstacle/robot label is ground truth. Raw LiDAR reconstruction uses
recorded TF at each scan stamp, never the current configuration or latest TF.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import sys
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/hsl_perception'))
from hsl_perception.core import Detector, StaticBackground
from hsl_perception.segmentation import RobotModel, cluster_xy, split_clusters
from hsl_perception.profiles import real_detector


def stamp(msg):
    return msg.sec + msg.nanosec*1e-9


def reader(path, topics):
    import rosbag2_py
    result = rosbag2_py.SequentialReader()
    result.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    result.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return result


def messages(path, topics):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = reader(path, topics)
    classes = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()
               if t.name in topics}
    while r.has_next():
        topic, data, time = r.read_next()
        yield topic, deserialize_message(data, classes[topic]), time*1e-9


def xyz(msg):
    fields={f.name:f for f in msg.fields}
    if any(name not in fields or fields[name].datatype!=7 for name in ('x','y','z')):
        raise ValueError('Expected FLOAT32 XYZ fields')
    arrays=[np.ndarray((msg.height,msg.width), dtype=('>' if msg.is_bigendian else '<')+'f4',
                       buffer=msg.data, offset=fields[name].offset,
                       strides=(msg.row_step,msg.point_step)).ravel() for name in ('x','y','z')]
    return np.column_stack(arrays).astype(float)[::max(1,msg.width*msg.height//6000)]


def transform(points, tf):
    q,t=tf.transform.rotation,tf.transform.translation
    v=np.array([q.x,q.y,q.z]); w=q.w
    a=2*np.cross(v,points)
    return points+w*a+np.cross(v,a)+[t.x,t.y,t.z]


def intervals(rows, predicate):
    result=[]; start=None
    for time,value in rows:
        active=predicate(value)
        if active and start is None: start=time
        if not active and start is not None:
            result.append([start,time]);start=None
    if start is not None: result.append([start,rows[-1][0]])
    return result


def audit(session, output, cloud_period, filter_lidar=False):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer
    files=sorted((session/'bag').glob('*.mcap'))
    if not files:return {'session':session.name,'error':'no MCAP'}
    # Most sessions have one file; streaming all split files preserves order.
    config=yaml.safe_load((session/'real_match.yaml').read_text()) if (session/'real_match.yaml').exists() else {}
    report={'session':session.name,'role':config.get('robot',{}).get('role'),
            'source':json.loads((session/'session.json').read_text()) if (session/'session.json').exists() else {},
            'metadata_present':(session/'bag/metadata.yaml').exists(),
            'labels':'unlabelled; detections/clusters are hypotheses, not measured ground truth'}
    topics=['/tf','/tf_static','/map','/navigation/self','/navigation/opponent',
            '/navigation/behavior','/navigation/planning_diagnostics','/navigation/mppi_diagnostics',
            '/navigation/global_status','/navigation/planner_status','/localization/status',
            '/localization/ready','/match/allowed','/match/active','/real/match_finished','/cmd_vel']
    tf=Buffer(cache_time=Duration(seconds=6000))
    rows={t:[] for t in topics}; counts=Counter(); own=[]; static=None
    for file in files:
        for topic,msg,time in messages(file,topics):
            counts[topic]+=1
            if topic in ('/tf','/tf_static'):
                for item in msg.transforms:
                    (tf.set_transform_static if topic=='/tf_static' else tf.set_transform)(item,'bag')
            elif topic=='/map':
                static=StaticBackground(msg.info.resolution,[msg.info.origin.position.x,msg.info.origin.position.y],
                                        msg.info.width,msg.info.height,msg.data)
                report['grid']={'resolution':msg.info.resolution,'origin':[msg.info.origin.position.x,msg.info.origin.position.y],
                                'width':msg.info.width,'height':msg.info.height,'data':list(msg.data)}
            elif topic=='/navigation/self':
                p=msg.pose.pose.position; q=msg.pose.pose.orientation
                own.append([stamp(msg.header.stamp),p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),msg.twist.twist.linear.x,msg.twist.twist.angular.z])
            elif topic=='/navigation/opponent':
                p=msg.pose.pose.position
                rows[topic].append([stamp(msg.header.stamp),[p.x,p.y]])
            elif topic=='/cmd_vel':rows[topic].append([time,[msg.linear.x,msg.angular.z]])
            else:
                value=msg.data
                if isinstance(value,str):
                    try:value=json.loads(value)
                    except json.JSONDecodeError:pass
                rows[topic].append([time,value])
    report.update(counts=dict(counts),own=own,recorded=rows)
    active=intervals(rows['/match/active'] or rows['/match/allowed'],bool)
    report['active_windows']=active
    report['recovery_windows']=intervals(rows['/navigation/global_status'],lambda s:'RECOVERY' in s)
    report['planner_status_counts']=dict(Counter(v for _,v in rows['/navigation/planner_status']))
    report['mppi_failure_counts']=dict(Counter(v.get('reason',v.get('result','unspecified')) for _,v in rows['/navigation/mppi_diagnostics'] if isinstance(v,dict) and v.get('result')!='ok'))
    if own:
        a=np.asarray(own); mask=np.array([any(start<=t<=end for start,end in active) for t in a[:,0]])
        report['active_own_samples']=int(mask.sum())
        if mask.sum()>1:
            b=a[mask]; dt=np.diff(b[:,0]); valid=(dt>0)&(dt<.3)
            report['active_measured_speed_mps']=float(np.sum(.5*(abs(b[1:,4])+abs(b[:-1,4]))[valid]*dt[valid])/np.sum(dt[valid])) if valid.any() else None
            report['speed_time_coverage_s']=float(dt[valid].sum())
    samples=[]; chunks=[]; tf_fail=0; last=-math.inf
    detector=Detector(RobotModel(max_height=.46))
    real=real_detector()
    strict=Detector(RobotModel(max_height=.46,max_gap_share=.12,line_ratio=.35),strong_arc_min_span_deg=90,allow_merged_strong=False,strong_min_inlier_fraction=.95)
    for file in files:
        for _,msg,time in messages(file,['/livox/lidar']):
            t=stamp(msg.header.stamp)
            if t-last<cloud_period:continue
            last=t
            if static is None:continue
            try:trans=tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
            except Exception:tf_fail+=1;continue
            if filter_lidar:
                from hsl_real.lidar_filter import filter_cloud
                from hsl_real.lidar_filter_core import DEFAULT_PARAMETERS
                msg, _ = filter_cloud(msg, DEFAULT_PARAMETERS)
            points=transform(xyz(msg),trans)
            sensor=np.array([trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z])
            points=points[np.isfinite(points).all(axis=1)&(np.linalg.norm(points[:,:2]-sensor[:2],axis=1)>=.25)]
            tracks={}
            for name,instance in [('source',detector),('simulation',strict),('real',real)]:
                track,diag=instance.step(points,sensor,static,t)
                tracks[name]={'position':track.mean[:2].tolist() if track is not None and abs(track.last_update-t)<1e-5 else None,'diagnostics':diag}
            fg=points[(points[:,2]>=.08)&(points[:,2]<=.6)&static.foreground(points)]
            labels,n=cluster_xy(fg[:,:2],.05)
            clusters=[]
            for c in split_clusters(fg,labels,n):
                if len(c)<8:continue
                clusters.append({'center':np.mean(c[:,:2],axis=0).tolist(),'count':len(c),
                                 'axis_span':np.ptp(c[:,:2],axis=0).tolist(),
                                 'height':np.percentile(c[:,2],[5,50,95,100]).tolist()})
            samples.append({'t':t,'sensor':sensor.tolist(),'tracks':tracks,'clusters':clusters,'chunk':len(chunks)})
            chunks.append(points.astype(np.float32))
    report['replay']={'lidar_filter_applied':filter_lidar,'period_s':cloud_period,'tf_missing_scans':tf_fail,'samples':samples,
                       'scope':'subsampled at recorded TF; not full-rate tracker validation'}
    output.mkdir(parents=True,exist_ok=True)
    if chunks:
        np.savez_compressed(output/(session.name+'.npz'),points=np.concatenate(chunks),offsets=np.cumsum([0]+[len(c) for c in chunks]))
    (output/(session.name+'.json')).write_text(json.dumps(report,indent=2)+'\n')
    return {k:report.get(k) for k in ('session','role','metadata_present','counts','active_windows','recovery_windows','planner_status_counts','mppi_failure_counts','active_measured_speed_mps','active_own_samples')} | {'cloud_samples':len(samples),'tf_missing_scans':tf_fail}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--period',type=float,default=.5);p.add_argument('--session',action='append')
    p.add_argument('--filter-lidar',action='store_true',help='apply current real self-return filter before reconstruction')
    args=p.parse_args()
    import rclpy
    rclpy.init()
    reports=[]
    for session in sorted(args.root.iterdir()):
        if not session.is_dir() or (args.session and session.name not in args.session):continue
        try:report=audit(session,args.output,args.period,args.filter_lidar)
        except Exception as e:report={'session':session.name,'error':str(e)}
        reports.append(report);print(json.dumps(report),flush=True)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(reports,indent=2)+'\n')
    rclpy.shutdown()

if __name__=='__main__':main()
