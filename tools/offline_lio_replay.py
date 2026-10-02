"""Isolated offline FAST-LIO replay/collector. No hardware or command publishers."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry


def xyz(msg):
    f = {f.name: f for f in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': [endian+'f4']*3,
                      'offsets': [f[k].offset for k in ('x','y','z')], 'itemsize': msg.point_step})
    data = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step))
    points = np.column_stack([data[k].ravel() for k in ('x','y','z')])
    return points[np.all(np.isfinite(points), axis=1)]


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


def main():
    p = argparse.ArgumentParser()
    p.add_argument('mcap'); p.add_argument('output'); p.add_argument('parameters')
    p.add_argument('--rate', type=float, default=1.)
    p.add_argument('--duration', type=float, default=0.)
    p.add_argument('--executable', default='/autoware/install/fast_lio/lib/fast_lio/fastlio_mapping')
    args = p.parse_args()
    if args.rate <= 0: raise ValueError('rate must be positive')
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    scans = out/'scans'; scans.mkdir(exist_ok=True)
    if any(scans.iterdir()): raise ValueError('Use an empty output directory')
    log = (out/'fastlio.log').open('w')
    process = subprocess.Popen([args.executable,'--ros-args','--params-file',args.parameters],stdout=log,stderr=log)
    rclpy.init(); n = rclpy.create_node('offline_map_collector')
    poses = []; frames = []; times = []; missing = 0
    qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
    def odom(msg):
        q=msg.pose.pose.orientation; pos=msg.pose.pose.position
        poses.append([stamp(msg), pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w])
    def cloud(msg):
        nonlocal missing
        t=stamp(msg)
        candidates = [p for p in poses[-6:] if abs(p[0]-t) < .02]
        if not candidates: missing += 1; return
        pose=min(candidates,key=lambda p:abs(p[0]-t))
        points=xyz(msg)
        name=f'{len(frames):05d}.npz'
        np.savez_compressed(scans/name, points=points, pose=np.asarray(pose))
        frames.append(name); times.append(t)
    n.create_subscription(Odometry,'/Odometry',odom,qos)
    n.create_subscription(PointCloud2,'/cloud_registered',cloud,qos)
    pubs={t:n.create_publisher(k,t,qos) for t,k in [('/livox/lidar',PointCloud2),('/livox/imu',Imu)]}
    types={'/livox/lidar':PointCloud2,'/livox/imu':Imu}
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.mcap,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    counts={t:0 for t in types}
    try:
        # Establish DDS subscribers before publishing the first stationary IMU samples.
        deadline=time.monotonic()+20
        while any(p.get_subscription_count()==0 for p in pubs.values()):
            if process.poll() is not None:raise RuntimeError('FAST-LIO exited; inspect fastlio.log')
            if time.monotonic()>deadline:raise TimeoutError('FAST-LIO subscriptions unavailable')
            rclpy.spin_once(n,timeout_sec=.05)
        start=None; wall_start=time.monotonic(); last_progress=-10
        while reader.has_next():
            topic,data,ts=reader.read_next()
            if topic not in types:continue
            if start is None:start=ts;wall_start=time.monotonic()
            elapsed=(ts-start)*1e-9
            if args.duration and elapsed>args.duration:break
            target=wall_start+elapsed/args.rate
            while time.monotonic()<target:
                rclpy.spin_once(n,timeout_sec=min(.005,max(0.,target-time.monotonic())))
                if process.poll() is not None:raise RuntimeError('FAST-LIO exited during replay')
            pubs[topic].publish(deserialize_message(data,types[topic]));counts[topic]+=1
            rclpy.spin_once(n,timeout_sec=0.)
            if elapsed-last_progress>=10:
                print(f'replay {elapsed:.1f}s, registered scans={len(frames)}, poses={len(poses)}',flush=True)
                last_progress=elapsed
        # Last scan needs a later IMU sample; do not invent an extra sample at EOF.
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
    finally:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=10)
        except subprocess.TimeoutExpired: process.kill();process.wait()
        log.close(); n.destroy_node();rclpy.shutdown()
        np.savetxt(out/'trajectory.csv', np.asarray(poses).reshape(-1,8), delimiter=',',
                   header='timestamp,x,y,z,qx,qy,qz,qw',comments='')
        report={'input_messages':counts,'registered_scans':len(frames),'poses':len(poses),
                'unpaired_clouds':missing,'registered_start_s':times[0] if times else None,
                'registered_end_s':times[-1] if times else None,'fastlio_exit_code':process.returncode,
                'rate':args.rate,'loop_closure':False}
        (out/'replay.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report),flush=True)
    if process.returncode != 0:raise RuntimeError('FAST-LIO exited abnormally; outputs retained for diagnosis')
    if len(frames)<10:raise RuntimeError('Too few registered frames')

if __name__=='__main__':main()
