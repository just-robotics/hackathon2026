#!/usr/bin/env python3
"""Compare production C++ cloud bytes/headers/fields to the Python replay oracle."""
import argparse
from array import array
import json
from pathlib import Path
import struct
import subprocess
import sys
import time
import numpy as np
from audit_real_bags import messages, stamp
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/hsl_real'))
from hsl_real.lidar_filter import filter_cloud
from hsl_real.lidar_filter_core import DEFAULT_PARAMETERS


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--period',type=float,default=.2);a=p.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2,PointField
    rclpy.init();node=rclpy.create_node('lidar_parity_check');results=[];received=[]
    pub=node.create_publisher(PointCloud2,'/test/lidar',qos_profile_sensor_data)
    sub=node.create_subscription(PointCloud2,'/sensing/lidar/points_filtered',received.append,qos_profile_sensor_data)
    child=subprocess.Popen(['ros2','run','hsl_lidar_filter','real_lidar_filter','--ros-args','-p','lidar_topic:=/test/lidar'])
    report={'scope':'byte/header/field parity, not physical motion or filter timing','sessions':{},'synthetic':0}
    try:
        deadline=time.monotonic()+10
        while pub.get_subscription_count()==0:
            if time.monotonic()>deadline:raise RuntimeError('native filter not discovered')
            rclpy.spin_once(node,timeout_sec=.02)
        def check(msg):
            expected,_=filter_cloud(msg,DEFAULT_PARAMETERS);received.clear()
            pub.publish(msg);deadline=time.monotonic()+10;retry=time.monotonic()+.2
            while not received:
                if time.monotonic()>deadline:raise RuntimeError('no native output')
                if time.monotonic()>retry:
                    pub.publish(msg);retry=time.monotonic()+.2
                rclpy.spin_once(node,timeout_sec=.02)
            got=received[-1]
            for key in ('header','fields','width','height','point_step','row_step','is_dense','is_bigendian'):
                assert getattr(got,key)==getattr(expected,key),(key,msg.header.stamp)
            assert bytes(got.data)==bytes(expected.data),msg.header.stamp
        for endian in ('<','>'):
            records=[struct.pack(endian+'fffBd',x,y,z,tag,i) for i,(x,y,z,tag) in enumerate([
                (1.,0.,0.,0),(.27,.07,.1,0),(2.,0.,0.,64),(3.,0.,0.,16),
                (float('nan'),0.,0.,0),(.2,0.,.1,32)])]
            msg=PointCloud2(height=2,width=3,point_step=21,row_step=66,is_bigendian=endian=='>',
                data=array('B',b''.join(records[:3])+b'pad'+b''.join(records[3:])+b'pad'))
            msg.header.frame_id='livox'
            msg.fields=[PointField(name=n,offset=o,datatype=d,count=1) for n,o,d in [('x',0,7),('y',4,7),('z',8,7),('tag',12,2),('timestamp',13,8)]]
            check(msg);report['synthetic']+=1
        for session in sorted(a.root.iterdir()):
            count=0;last=-np.inf
            for file in sorted((session/'bag').glob('*.mcap')):
                for _,msg,_ in messages(file,['/livox/lidar']):
                    t=stamp(msg.header.stamp)
                    if t-last<a.period:continue
                    last=t;check(msg);count+=1
            report['sessions'][session.name]=count;print(session.name,count,flush=True)
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
    finally:
        child.terminate();child.wait(timeout=10);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
