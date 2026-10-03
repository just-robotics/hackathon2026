#!/usr/bin/env python3
"""ROS fixture: late small-box reclassification clears dynamic cells only."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/hsl_sim_adapter'))
from hsl_sim_adapter.cloud import make_cloud


def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 import rclpy
 from ament_index_python.packages import get_package_prefix
 from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
 from geometry_msgs.msg import TransformStamped
 from nav_msgs.msg import OccupancyGrid
 from nav2_msgs.msg import Costmap
 from sensor_msgs.msg import PointCloud2
 from std_msgs.msg import Header
 from tf2_msgs.msg import TFMessage
 rclpy.init();n=rclpy.create_node('semantic_fixture')
 latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 tf=n.create_publisher(TFMessage,'/tf_static',latched);mp=n.create_publisher(OccupancyGrid,'/map',latched)
 obs=n.create_publisher(PointCloud2,'/navigation/obstacle_scan',qos_profile_sensor_data)
 ignored=n.create_publisher(PointCloud2,'/navigation/ignored_obstacles',qos_profile_sensor_data)
 received=[];sub=n.create_subscription(Costmap,'/native_mppi/costmap_raw',received.append,10)
 child=subprocess.Popen([str(Path(get_package_prefix('hsl_nav2_control'))/'lib/hsl_nav2_control/native_mppi'),'--ros-args','--params-file',str(ROOT/'src/hsl_nav2_control/config/native_mppi.yaml'),'-p','use_sim_time:=false'])
 try:
  t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_footprint';t.transform.rotation.w=1.
  grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05;grid.info.width=grid.info.height=80;grid.info.origin.position.x=grid.info.origin.position.y=-2.;grid.info.origin.orientation.w=1.;grid.data=[0]*6400
  small=(.575,.075,.3);wide=(1.075,.575,.15);wall=(.575,-.575,.3)
  def ix(v):return int((v+2)/.05)
  grid.data[ix(wall[1])*80+ix(wall[0])]=100
  def cost(m,pt):
   x=int((pt[0]-m.metadata.origin.position.x)/m.metadata.resolution);y=int((pt[1]-m.metadata.origin.position.y)/m.metadata.resolution)
   return m.data[y*m.metadata.size_x+x]
  def phase(reclassified,seconds):
   end=time.monotonic()+seconds;selected=[]
   while time.monotonic()<end:
    h=Header(frame_id='map',stamp=n.get_clock().now().to_msg());t.header.stamp=h.stamp;tf.publish(TFMessage(transforms=[t]));grid.header.stamp=h.stamp;mp.publish(grid)
    ignored.publish(make_cloud(h,[small,wall] if reclassified else []))
    obs.publish(make_cloud(h,[wide] if reclassified else [small,wide]))
    start=len(received)
    until=time.monotonic()+.1
    while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
    for m in received[start:]:selected.append([cost(m,pt) for pt in (small,wide,wall)])
   return selected
  before=phase(False,8);after=phase(True,4)
  assert before and any(row[0]==254 and row[1]==254 and row[2]==254 for row in before),before
  assert after and all(row[0]<254 and row[1]==254 and row[2]==254 for row in after[-3:]),after
  a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({'before':before,'after':after,'scope':'small dynamic hit removed; wide dynamic hit and static wall preserved'},indent=2)+'\n')
  print('semantic costmap fixture passed',flush=True)
 finally:
  child.terminate();child.wait(timeout=10);n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
