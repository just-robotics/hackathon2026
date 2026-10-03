#!/usr/bin/env python3
"""Recorded raw sensors through real bringup/AMCL, with drivers and motion off.

Wall-time transport replay; it tests observation freshness, not physical driving.
No ground-truth map correction is replayed. No filter execution-time benchmark.
"""
import argparse
from collections import Counter
import os
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import shutil
import tempfile
import time
import numpy as np
from audit_real_bags import messages,stamp


def main():
 # Match the production real Compose middleware; all replay peers inherit it.
 os.environ.setdefault('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
 os.environ.setdefault('ROS_DOMAIN_ID','177')
 # network-none containers have only loopback; use explicit unicast discovery.
 os.environ.setdefault('CYCLONEDDS_URI','<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>80</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('session',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=35.);p.add_argument('--planning-target',type=float,nargs=2,help='Isolated manual planning probe; permission remains false');a=p.parse_args()
 if a.planning_target and any(x.name!='lo' for x in Path('/sys/class/net').iterdir()):
  raise RuntimeError('planning probe requires network-none container')
 import rclpy
 from sensor_msgs.msg import PointCloud2
 from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
 from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
 from tf2_msgs.msg import TFMessage
 from geometry_msgs.msg import Twist
 from std_msgs.msg import String, Bool
 from hsl_interfaces.msg import PlanningIntent
 rclpy.init();n=rclpy.create_node('recorded_transport')
 pubs={t:n.create_publisher(c,t,qos_profile_sensor_data if c is PointCloud2 else 50) for t,c in [('/livox/lidar',PointCloud2),('/odom',Odometry),('/tf',TFMessage)]}
 received={t:[] for t in ['/navigation/self','/navigation/scan','/sensing/lidar/points_filtered']};commands=[];diagnostics={t:[] for t in ['/localization/status','/navigation/observation_diagnostics','/navigation/detector_diagnostics','/navigation/obstacle_filter_diagnostics','/navigation/planning_diagnostics','/navigation/mppi_diagnostics']}
 wall_alignment=[]; map_state={}; last_alignment=[-float('inf')]
 from hsl_perception.geometry import StaticBackground
 from hsl_perception.cloud import cloud_xyz
 def map_received(msg):
  if msg.header.frame_id=='map' and msg.info.width:
   map_state['background']=StaticBackground(msg.info.resolution,
    [msg.info.origin.position.x,msg.info.origin.position.y],msg.info.width,msg.info.height,msg.data,margin=.10)
 def scan_alignment(msg):
  now=time.monotonic()
  if now-last_alignment[0]<.5 or 'background' not in map_state or msg.header.frame_id!='map':return
  last_alignment[0]=now
  xyz=np.asarray(cloud_xyz(msg),dtype=float).reshape(-1,3)
  # Tall returns predominantly belong to walls; boxes/robot can still overlap.
  # This is a map-consistency diagnostic, not independent localization truth.
  wall=xyz[np.isfinite(xyz).all(axis=1)&(xyz[:,2]>=.50)&(xyz[:,2]<=.70)]
  background=map_state['background']
  index=np.floor((wall[:,:2]-background.origin)/background.resolution).astype(int)
  x,y=index.T
  inside=(x>=0)&(y>=0)&(x<background.width)&(y<background.height)
  matched=np.zeros(len(wall),dtype=bool)
  matched[inside]=background.mask[y[inside],x[inside]]
  wall_alignment.append(dict(stamp_s=stamp(msg.header.stamp),points=len(wall),
   match_fraction=float(np.mean(matched)) if len(wall) else None))
 subs=[n.create_subscription(OccupancyGrid,'/map',map_received,
  QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
  n.create_subscription(PointCloud2,'/navigation/scan',scan_alignment,qos_profile_sensor_data)]
 subs += [n.create_subscription(c,t,lambda m,t=t:received[t].append([time.monotonic(),stamp(m.header.stamp)]),qos_profile_sensor_data if c is PointCloud2 else 50) for t,c in [('/navigation/self',Odometry),('/navigation/scan',PointCloud2),('/sensing/lidar/points_filtered',PointCloud2)]]
 subs.append(n.create_subscription(Twist,'/cmd_vel',lambda m:commands.append([m.linear.x,m.angular.z]),10))
 paths=[]; mppi_commands=[]; planning_pub=None
 subs.append(n.create_subscription(RosPath,'/navigation/global_path',lambda m:paths.append(dict(
  stamp_s=stamp(m.header.stamp),points=len(m.poses),
  end=[m.poses[-1].pose.position.x,m.poses[-1].pose.position.y] if m.poses else None)),10))
 subs.append(n.create_subscription(Twist,'/navigation/mppi_cmd_vel',lambda m:mppi_commands.append([m.linear.x,m.angular.z]),10))
 if a.planning_target:
  planning_pub=n.create_publisher(PlanningIntent,'/navigation/intent',10)
  state_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  allowed=n.create_publisher(Bool,'/match/allowed',state_qos)
  active=n.create_publisher(Bool,'/match/active',state_qos)
  locked=n.create_publisher(Bool,'/probe/never_active',state_qos)
  def probe():
   allowed.publish(Bool(data=False));active.publish(Bool(data=True));locked.publish(Bool(data=False))
   intent=PlanningIntent();intent.header.frame_id='map';intent.header.stamp=n.get_clock().now().to_msg()
   intent.behavior=PlanningIntent.GOAL;intent.has_target=True
   intent.target.position.x=a.planning_target[0];intent.target.position.y=a.planning_target[1]
   intent.target.orientation.w=1.;intent.target_tolerance=.08;intent.max_speed=.5
   intent.reason='recorded sensor planning probe, final motion locked'
   planning_pub.publish(intent)
  n.create_timer(.1,probe)
 for topic in diagnostics:
  subs.append(n.create_subscription(String,topic,lambda m,t=topic:diagnostics[t].append(json.loads(m.data)),10))
 data=[]
 for file in sorted((a.session/'bag').glob('*.mcap')):
  for topic,msg,t in messages(file,list(pubs)):
   if topic=='/tf':
    msg.transforms=[x for x in msg.transforms if x.header.frame_id=='odom']
    if not msg.transforms:continue
   data.append((t,topic,msg))
 if not data:raise RuntimeError('missing sensors')
 a.output.parent.mkdir(parents=True,exist_ok=True);log=a.output.with_suffix('.ros.log').open('w')
 # Preserve the recorded role/start, while testing current filter/planning.
 temporary=tempfile.TemporaryDirectory(prefix='hsl-transport-')
 config=Path(temporary.name)/'config'
 shutil.copytree('/work/config',config)
 mission=a.session/'real_match.yaml'
 if not mission.is_file():raise RuntimeError('recorded mission missing')
 shutil.copyfile(mission,config/'real_match.yaml')
 launch_file='/work/src/hsl_real/launch/robot.launch.py'
 if a.planning_target:
  # Only the fixture replaces decision/referee. All sensor/planner/controller
  # nodes and their current parameters are the production bringup.
  wrapper=Path(temporary.name)/'probe.launch.py'
  wrapper.write_text('''import importlib.util
from launch.conditions import IfCondition
spec=importlib.util.spec_from_file_location('production_real','/work/src/hsl_real/launch/robot.launch.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
original=m.Node
def node(**kwargs):
 if kwargs.get('executable') in ('decision_manager','real_match'):
  kwargs['condition']=IfCondition('false')
 if kwargs.get('executable')=='motion_gate':
  kwargs['remappings']=[('/match/active','/probe/never_active')]
 return original(**kwargs)
m.Node=node
generate_launch_description=m.generate_launch_description
''')
  launch_file=str(wrapper)
 child=subprocess.Popen(['ros2','launch',launch_file,'config_file:='+str(config/'real.yaml'),'drivers_enabled:=false'],stdout=log,stderr=subprocess.STDOUT)
 try:
  # Let discovery happen; no recorded timestamp is altered except by one offset.
  until=time.monotonic()+5
  while time.monotonic()<until:
   if child.poll() is not None:
    raise RuntimeError(f'real bringup terminated; inspect {a.output.with_suffix(".ros.log")}')
   rclpy.spin_once(n,timeout_sec=.02)
  begin=time.monotonic();offset=n.get_clock().now().nanoseconds*1e-9-data[0][0]
  def shift(s):
   t=stamp(s)+offset;s.sec=int(t);s.nanosec=int((t-int(t))*1e9)
  first=data[0][0]
  published=Counter()
  last_published=None
  for recorded,topic,msg in data:
   if child.poll() is not None:
    raise RuntimeError(f'real bringup terminated; inspect {a.output.with_suffix(".ros.log")}')
   delta=recorded-first
   if delta>a.seconds:break
   while time.monotonic()<begin+delta:rclpy.spin_once(n,timeout_sec=.005)
   out=deepcopy(msg)
   if topic=='/tf':
    for tf in out.transforms:shift(tf.header.stamp)
   else:shift(out.header.stamp)
   pubs[topic].publish(out)
   published[topic]+=1
   last_published=recorded
   rclpy.spin_once(n,timeout_sec=0)
  until=time.monotonic()+1
  while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
  report={'scope':'recorded sensors/AMCL, drivers disabled, motion never enabled','seconds':a.seconds,'stages':{},'diagnostics':diagnostics,'scan_wall_alignment':wall_alignment,'planning_target':a.planning_target,
   'global_paths':paths,'nonzero_mppi_commands':sum(abs(v)+abs(w)>1e-8 for v,w in mppi_commands),'nonzero_final_commands':sum(abs(v)+abs(w)>1e-8 for v,w in commands)}
  report['sensor_replay']={
   'header_clock_offset_s':offset,
   'recorded_span_s':data[-1][0]-first,
   'published_span_s':None if last_published is None else last_published-first,
   'complete_sensor_window':last_published==data[-1][0],
   'recorded_messages':dict(Counter(topic for _,topic,_ in data)),
   'published_messages':dict(published)}
  for topic,rows in received.items():
   times=np.array([t for t,_ in rows if t>begin+8]);gaps=np.diff(times)
   report['stages'][topic]={'count':len(rows),'steady_count':len(times),'steady_max_gap_s':float(gaps.max()) if len(gaps) else None,'steady_p95_gap_s':float(np.percentile(gaps,95)) if len(gaps) else None}
  a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ('diagnostics','scan_wall_alignment','global_paths')}),flush=True)
  assert commands, 'no final motion gate commands observed'
  assert report['nonzero_final_commands']==0, 'final motion gate emitted nonzero command; report saved'
  assert all(stage['steady_count']>0 for stage in report['stages'].values()),'missing steady observations; inspect ROS/AMCL log'
 finally:
  if a.planning_target:
   allowed.publish(Bool(data=False));active.publish(Bool(data=False))
  child.terminate()
  try:child.wait(timeout=15)
  except subprocess.TimeoutExpired:child.kill();child.wait()
  log.close();temporary.cleanup();n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
