#!/usr/bin/env python3
"""Controlled Gazebo push test, NOT an autonomous duel.

Only run in an isolated prepared project after stopping its decision/referee
containers. Gazebo box pose is an evaluation label, never a planner input.
"""
import argparse
import json
from math import hypot,sin,cos
from pathlib import Path
import time
import subprocess
import re
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path as RosPath
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import SpawnEntity,DeleteEntity
from hsl_interfaces.msg import PlanningIntent
from hsl_sim_adapter.scenario_obstacles import box_sdf
from std_msgs.msg import Bool,String


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--yaw',type=float,default=0.)
    ap.add_argument('--box',type=float,nargs=2,default=[1.2,.5])
    ap.add_argument('--goal',type=float,nargs=2,default=[2.4,.5])
    ap.add_argument('--origin',type=float,nargs=2,default=[-.468,-.582])
    ap.add_argument('--active-s',type=float,default=45.)
    args=ap.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=Node('push_box_trial',parameter_overrides=[Parameter('use_sim_time',value=True)])
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state={};diag=[];mppi_diag=[];contacts=[];samples=[]
    n.create_subscription(Odometry,'/navigation/self',lambda m:state.update(own=m),10)
    n.create_subscription(String,'/navigation/planner_status',lambda m:state.update(status=m.data),10)
    n.create_subscription(String,'/navigation/obstacle_filter_diagnostics',lambda m:diag.append(json.loads(m.data)),10)
    n.create_subscription(String,'/navigation/mppi_diagnostics',lambda m:mppi_diag.append({
        't':n.get_clock().now().nanoseconds*1e-9,'diagnostic':json.loads(m.data)}),10)
    n.create_subscription(ContactsState,'/body_contacts',lambda m:contacts.extend([[s.collision1_name,s.collision2_name] for s in m.states]),10)
    allow=n.create_publisher(Bool,'/match/allowed',qos)
    active=n.create_publisher(Bool,'/match/active',qos)
    intent=n.create_publisher(PlanningIntent,'/navigation/intent',10)
    delete=n.create_client(DeleteEntity,'/delete_entity');spawn=n.create_client(SpawnEntity,'/spawn_entity')
    def wait(predicate,seconds=60):
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
        if not predicate():raise RuntimeError('fixture timed out')
    def call(client,request,required=True):
        wait(client.service_is_ready);future=client.call_async(request);wait(future.done)
        if required and not future.result().success:raise RuntimeError(future.result().status_message)
    report={'scope':'controlled fixture, not duel','yaw':args.yaw,'box_initial':args.box,'goal':args.goal}
    try:
        wait(lambda:'own' in state)
        for name in ['unknown_box_push','unknown_box_large','unknown_box_small_1','unknown_box_small_2','unknown_box_small_3']:
            req=DeleteEntity.Request();req.name=name;call(delete,req,False)
        box={'name':'unknown_box_push','size':[.15,.15,.4],'movable':True,'mass':.1,'friction':.3,'pose':[*args.box,args.yaw]}
        req=SpawnEntity.Request();req.name=box['name'];req.xml=box_sdf(box);req.reference_frame='world'
        req.initial_pose.position.x=args.box[0]+args.origin[0];req.initial_pose.position.y=args.box[1]+args.origin[1]
        req.initial_pose.position.z=.2;req.initial_pose.orientation.z=sin(args.yaw/2);req.initial_pose.orientation.w=cos(args.yaw/2)
        call(spawn,req)
        # Let the sensor pipeline observe the newly spawned physical object.
        warmup=n.get_clock().now().nanoseconds*1e-9
        end=time.monotonic()+90
        while n.get_clock().now().nanoseconds*1e-9-warmup<10.:
            if time.monotonic()>end:raise RuntimeError('warmup clock stalled')
            rclpy.spin_once(n,timeout_sec=.03)
        start=n.get_clock().now().nanoseconds*1e-9;last=-1;end=time.monotonic()+180
        allow.publish(Bool(data=True));active.publish(Bool(data=True))
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.03)
            now=n.get_clock().now().nanoseconds*1e-9;elapsed=now-start
            m=PlanningIntent();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg()
            m.behavior=PlanningIntent.GOAL;m.has_target=True;m.target.position.x=args.goal[0];m.target.position.y=args.goal[1]
            m.target.orientation.w=1.;m.max_speed=.5;m.target_tolerance=.08;m.reason='controlled push-box fixture'
            active.publish(Bool(data=True))
            intent.publish(m)
            if elapsed-last<.1:continue
            last=elapsed;p=state['own'].pose.pose.position
            velocity=state['own'].twist.twist
            samples.append({'t':elapsed,'xy':[p.x,p.y],'status':state.get('status'),
                'measured_vx_mps':velocity.linear.x,'measured_omega_radps':velocity.angular.z})
            if hypot(p.x-args.goal[0],p.y-args.goal[1])<=.08:
                report['goal_reached']=True;break
            if elapsed>=args.active_s:report['goal_reached']=False;break
        distance=sum(hypot(b['xy'][0]-a['xy'][0],b['xy'][1]-a['xy'][1]) for a,b in zip(samples,samples[1:]))
        info=subprocess.run(['gz','model','-m','unknown_box_push','-i'],capture_output=True,text=True,timeout=10,check=True).stdout
        pose=re.search(r'pose \{\s*position \{\s*x: (\S+)\s*y: (\S+)\s*z: (\S+)',info)
        if pose is None:raise RuntimeError('Gazebo model pose unavailable')
        final_box=[float(pose[1])-args.origin[0],float(pose[2])-args.origin[1],float(pose[3])]
        report.update(final_box=final_box,final_box_displacement_m=hypot(final_box[0]-args.box[0],final_box[1]-args.box[1]),duration_s=samples[-1]['t'],distance_m=distance,mean_speed_mps=distance/samples[-1]['t'],
            closest_goal_distance_m=min(hypot(s['xy'][0]-args.goal[0],s['xy'][1]-args.goal[1]) for s in samples),
            samples=samples,filter_reports=diag,mppi_reports=mppi_diag,
            contact_pairs=sorted(set(tuple(c) for c in contacts)))
    finally:
        allow.publish(Bool(data=False));active.publish(Bool(data=False))
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('samples','filter_reports','mppi_reports')},indent=2))
        n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
