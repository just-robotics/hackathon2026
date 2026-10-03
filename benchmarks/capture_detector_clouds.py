#!/usr/bin/env python3
"""Read-only ROS evaluation capture of full clouds, map, own/peer poses and sensor TF.

Run with the private simulation's ROS_DOMAIN_ID. Opponent truth is an offline
label only. Writes JSON to stdout; no commands or navigation topics published.
"""
import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--wall-seconds',type=float,default=30.)
parser.add_argument('--sample-period',type=float,default=0.,help='minimum sim interval; default captures every distinct cloud')
parser.add_argument('--stream',action='store_true',help='flush JSONL samples so container cleanup cannot lose the capture')
parser.add_argument('--stop-after-match',action='store_true')
args=parser.parse_args()
if args.sample_period < 0:parser.error("sample-period must be nonnegative")
import json,time,rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer,TransformListener
from hsl_perception.cloud import cloud_xyz,seconds
rclpy.init();n=Node('capture_detector',parameter_overrides=[Parameter('use_sim_time',value=True)]);data={};tf=Buffer();listener=TransformListener(tf,n);subs=[]
match={'started':False,'finished':False}
def on_active(msg):
 if msg.data:match['started']=True
 elif match['started']:match['finished']=True
subs.append(n.create_subscription(Bool,'/match/active',on_active,10))
for key,t,typ in [('grid','/map',OccupancyGrid),('self','/navigation/self',Odometry),('peer','/opponent/navigation/self',Odometry),('scan','/navigation/scan',PointCloud2),('peer_scan','/opponent/navigation/scan',PointCloud2)]:
 subs.append(n.create_subscription(typ,t,lambda m,key=key:data.__setitem__(key,m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if key=='grid' else qos_profile_sensor_data))
end=time.monotonic()+args.wall_seconds;out={'samples':[]};last={};grid_sent=False
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.05)
 if args.stop_after_match and match['finished']:break
 if not all(k in data for k in ['grid','self','peer']):continue
 if args.stream and not grid_sent:
  g=data['grid'];print(json.dumps({'grid':{'resolution':g.info.resolution,'origin':[g.info.origin.position.x,g.info.origin.position.y],'width':g.info.width,'height':g.info.height,'data':list(g.data)}}),flush=True);grid_sent=True
 for key,sensorname,own,peer in [('scan','livox_frame','self','peer'),('peer_scan','opponent/livox_frame','peer','self')]:
  if key not in data:continue
  m=data[key];t=seconds(m.header.stamp)
  if t <= last.get(key,-1e9) or t-last.get(key,-1e9)<args.sample_period:continue
  try:p=tf.lookup_transform('map',sensorname,Time.from_msg(m.header.stamp)).transform.translation
  except Exception:continue
  last[key]=t;a=data[own].pose.pose.position;b=data[peer].pose.pose.position
  sample={'observer':key,'stamp':t,'sensor':[p.x,p.y,p.z],'own':[a.x,a.y],'peer':[b.x,b.y],'points':cloud_xyz(m).tolist()}
  if args.stream:print(json.dumps({'sample':sample}),flush=True)
  else:out['samples'].append(sample)
if 'grid' in data:
 g=data['grid'];out['grid']={'resolution':g.info.resolution,'origin':[g.info.origin.position.x,g.info.origin.position.y],'width':g.info.width,'height':g.info.height,'data':list(g.data)}
if not args.stream:print(json.dumps(out))
n.destroy_node();rclpy.shutdown()
