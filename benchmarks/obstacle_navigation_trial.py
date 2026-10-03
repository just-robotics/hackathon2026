#!/usr/bin/env python3
"""Isolated Gazebo navigation fixture: unknown boxes, then remove/move them.

Stop this project's decision/referee containers before running. This harness
publishes GOAL intents; it is a navigation test, never an evaluated duel.
"""
import argparse,json,time,random
from math import cos,sin,hypot,ceil
from pathlib import Path as File
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid,Odometry,Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String
from gazebo_msgs.srv import SpawnEntity,DeleteEntity
from hsl_interfaces.msg import PlanningIntent
from hsl_planning.core import Pose2,VoxelWorld,astar


def rectangle_distance(x,y,box):
    dx,dy=x-box['x'],y-box['y'];c,s=cos(box['yaw']),sin(box['yaw'])
    u,v=c*dx+s*dy,-s*dx+c*dy
    return hypot(max(abs(u)-box['sx']/2,0.),max(abs(v)-box['sy']/2,0.))


def boundary(box):
    c,s=cos(box['yaw']),sin(box['yaw'])
    pts=[]
    nx,ny=ceil(box['sx']/.025),ceil(box['sy']/.025)
    for i in range(nx+1):
        u=-box['sx']/2+box['sx']*i/nx
        for v in [-box['sy']/2,box['sy']/2]:pts.append((box['x']+c*u-s*v,box['y']+s*u+c*v,.15))
    for j in range(ny+1):
        v=-box['sy']/2+box['sy']*j/ny
        for u in [-box['sx']/2,box['sx']/2]:pts.append((box['x']+c*u-s*v,box['y']+s*u+c*v,.15))
    return pts


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',required=True);ap.add_argument('--seed',type=int,default=0)
    ap.add_argument('--small-count',type=int,choices=(2,3),default=3)
    ap.add_argument('--active-s',type=float,default=150.)
    ap.add_argument('--origin',type=float,nargs=2,default=[-.468,-.582])
    ap.add_argument('--goal',type=float,nargs=2,default=[.5,3.5])
    ap.add_argument('--remove-after',type=float,default=0.)
    args=ap.parse_args();rng=random.Random(args.seed)
    rclpy.init();n=Node('obstacle_navigation_trial',parameter_overrides=[Parameter('use_sim_time',value=True)])
    data={};qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for key,topic,kind,quality in [('grid','/map',OccupancyGrid,qos),('observed','/navigation/obstacle_grid',OccupancyGrid,qos),
        ('pose','/navigation/self',Odometry,10),('path','/navigation/global_path',Path,10),
        ('cmd','/cmd_vel',Twist,10),('status','/navigation/planner_status',String,10),
        ('track','/opponent/odom',Odometry,10)]:
        n.create_subscription(kind,topic,lambda msg,key=key:data.__setitem__(key,msg),quality)
    spawn=n.create_client(SpawnEntity,'/spawn_entity');delete=n.create_client(DeleteEntity,'/delete_entity')
    allowed=n.create_publisher(Bool,'/match/allowed',qos)
    active=n.create_publisher(Bool,'/match/active',qos)
    intents=n.create_publisher(PlanningIntent,'/navigation/intent',10)
    def spin_until(predicate,wall_s=30):
        end=time.monotonic()+wall_s
        while not predicate() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
        if not predicate():raise RuntimeError('ROS fixture timed out')
    def call(client,req):
        spin_until(client.service_is_ready)
        future=client.call_async(req);spin_until(future.done)
        result=future.result()
        if not result.success:raise RuntimeError(result)
        return result
    report={'seed':args.seed,'scope':'controlled navigation, not a duel','boxes':[],'samples':[]}
    try:
        spin_until(lambda:'grid' in data and 'pose' in data)
        req=DeleteEntity.Request();req.name='opponent';call(delete,req)
        report['opponent_removed_for_navigation_fixture']=True
        grid=data['grid'];own=data['pose'].pose.pose.position;start=Pose2(own.x,own.y)
        goal=Pose2(*args.goal);walls=[];free=set();world=VoxelWorld(.15,.23)
        for i,v in enumerate(grid.data):
            row,col=divmod(i,grid.info.width)
            x=grid.info.origin.position.x+(col+.5)*grid.info.resolution
            y=grid.info.origin.position.y+(row+.5)*grid.info.resolution
            if v>=50:walls.append((x,y,.3))
            elif v==0:free.add(world.cell(x,y))
        bounds=(-.025,-.025,3.065,4.05)
        world.update(walls,[],None,free,bounds)
        original=astar(world,start,goal,max_cells=20000)
        if not original:raise RuntimeError('static route unavailable')
        # Put fixtures along actual routes, with known offline alternate paths.
        # Fixtures are never fed to robot navigation as ground truth.
        candidates=list(original[2:-2]);rng.shuffle(candidates)
        for idx,(sx,sy,sz) in enumerate([(0.4,0.6,0.2)]+[(.15,.15,.4)]*args.small_count):
            placed=False
            for pt in candidates:
                yaw=rng.choice([0.,.7853981634,1.5707963268,2.3561944902])
                box={'name':f'unknown_box_{idx}','x':pt.x,'y':pt.y,'yaw':yaw,'sx':sx,'sy':sy,'sz':sz}
                if min(hypot(pt.x-start.x,pt.y-start.y),hypot(pt.x-goal.x,pt.y-goal.y))<.75:continue
                if any(rectangle_distance(pt.x,pt.y,b)<hypot(sx,sy)/2+.20 for b in report['boxes']):continue
                points=boundary(box)
                if any(world.obstacle_clearance(x,y)<.05 for x,y,_ in points):continue
                test=VoxelWorld(.15,.23)
                all_points=walls+points+sum([boundary(b) for b in report['boxes']],[])
                test.update(all_points,[],None,free,bounds)
                alternate=astar(test,start,goal,max_cells=20000)
                if not alternate:continue
                req=SpawnEntity.Request();req.name=box['name'];req.reference_frame='world'
                req.initial_pose.position.x=pt.x+args.origin[0];req.initial_pose.position.y=pt.y+args.origin[1]
                req.initial_pose.position.z=sz/2;req.initial_pose.orientation.z=sin(yaw/2);req.initial_pose.orientation.w=cos(yaw/2)
                req.xml=f'<sdf version="1.6"><model name="{box["name"]}"><static>true</static><link name="box"><collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision><visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></visual></link></model></sdf>'
                call(spawn,req);report['boxes'].append(box);placed=True;break
            if not placed:raise RuntimeError(f'Cannot place box {idx} with a feasible detour')
        report['static_map_occupied']=sum(v>=50 for v in grid.data)
        allowed.publish(Bool(data=True))
        started=n.get_clock().now().nanoseconds*1e-9;last=-1;removed=False
        deadline=time.monotonic()+600
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.02)
            now=n.get_clock().now().nanoseconds*1e-9;elapsed=now-started
            active.publish(Bool(data=True))
            msg=PlanningIntent();msg.header.frame_id='map';msg.header.stamp=n.get_clock().now().to_msg()
            msg.behavior=PlanningIntent.GOAL;msg.has_target=True;msg.target.position.x=goal.x;msg.target.position.y=goal.y
            msg.target.orientation.w=1.;msg.max_speed=.5;msg.target_tolerance=.08;msg.reason='unknown obstacle navigation fixture'
            intents.publish(msg)
            if elapsed-last<.1:continue
            last=elapsed;p=data['pose'].pose.pose.position
            observed=data.get('observed');cmd=data.get('cmd',Twist());track=data.get('track')
            distances=[rectangle_distance(p.x,p.y,b)-.178 for b in report['boxes']] if not removed else []
            record={'t':elapsed,'xy':[p.x,p.y],'body_box_clearance':min(distances,default=None),
                'cmd':[cmd.linear.x,cmd.angular.z],'status':data.get('status',String()).data,
                'observed_occupied':sum(v>=50 for v in observed.data) if observed else None,
                'path':[[q.pose.position.x,q.pose.position.y] for q in data.get('path',Path()).poses],
                'track':[track.pose.pose.position.x,track.pose.pose.position.y] if track else None}
            report['samples'].append(record)
            if args.remove_after and not removed and elapsed>=args.remove_after:
                for b in report['boxes']:
                    req=DeleteEntity.Request();req.name=b['name'];call(delete,req)
                removed=True;report['removed_at']=elapsed
            if hypot(p.x-goal.x,p.y-goal.y)<=.08:
                report['goal_reached']=True;break
            if elapsed>=args.active_s:report['goal_reached']=False;break
        samples=report['samples'];duration=samples[-1]['t']
        distance=sum(hypot(b['xy'][0]-a['xy'][0],b['xy'][1]-a['xy'][1]) for a,b in zip(samples,samples[1:]))
        report.update(duration_s=duration,distance_m=distance,mean_speed_mps=distance/duration,
            min_body_box_clearance_m=min((r['body_box_clearance'] for r in samples if r['body_box_clearance'] is not None),default=None))
        print(json.dumps({k:v for k,v in report.items() if k!='samples'}),flush=True)
    finally:
        active.publish(Bool(data=False));allowed.publish(Bool(data=False))
        File(args.output).write_text(json.dumps(report))
        n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
