#!/usr/bin/env python3
"""Analyze recorded detector outputs against the recorded static map; never change detector.

Wall intersections are map consistency checks, not semantic object labels.
"""
import argparse,json,math
from bisect import bisect_left
from collections import Counter
from pathlib import Path
import numpy as np
from replay_branch_detector import reader
from jr_perception.robot_detector import stamp_seconds,cloud_to_xyz


def analyze(path):
    poses=[];detections=[];marker_frames=[];health=[];localization=[];grid=None
    for topic,m,t in reader(path,{'/map','/navigation/self','/opponent/odom','/opponent/markers',
        '/navigation/detector_diagnostics','/localization/status','/navigation/indication'}):
        if topic=='/map':grid=m
        elif topic=='/navigation/self':
            p=m.pose.pose.position;q=m.pose.pose.orientation
            poses.append([stamp_seconds(m.header.stamp),p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
        elif topic=='/opponent/odom':
            p=m.pose.pose.position;detections.append(dict(t=stamp_seconds(m.header.stamp),xy=[p.x,p.y],vx=m.twist.twist.linear.x,
                covariance_xy=[m.pose.covariance[0],m.pose.covariance[7]]))
        elif topic=='/opponent/markers':
            active=[x for x in m.markers if x.action!=3]
            if not active:continue
            marker_frames.append(dict(t=stamp_seconds(active[0].header.stamp),clusters=[dict(id=x.id,
                xy=[x.pose.position.x,x.pose.position.y],size=[x.scale.x,x.scale.y,x.scale.z],
                kind='strong' if x.color.r<.2 and x.color.g>.7 else 'weak' if x.color.r>.8 and x.color.g>.7 else 'rejected')
                for x in active if x.ns=='clusters'],reasons=[x.text for x in active if x.ns=='reasons']))
        elif topic=='/localization/status':
            try:localization.append(json.loads(m.data))
            except ValueError:pass
    if grid is None:raise ValueError('No map')
    a=np.array(grid.data).reshape(grid.info.height,grid.info.width);res=grid.info.resolution
    ox,oy=grid.info.origin.position.x,grid.info.origin.position.y
    rows,cols=np.nonzero(a>=50);walls=np.column_stack((ox+(cols+.5)*res,oy+(rows+.5)*res))
    def occupancy(xy):
        c,r=np.floor((np.asarray(xy)-[ox,oy])/res).astype(int).T
        ok=(r>=0)&(r<a.shape[0])&(c>=0)&(c<a.shape[1]);vals=np.full(len(r),-2)
        vals[ok]=a[r[ok],c[ok]];return vals
    def wall_distance(xy):return float(np.sqrt(((walls-xy)**2).sum(axis=1)).min())
    def line(own,peer):
        d=np.linalg.norm(np.array(peer)-own);k=max(2,int(d/(res*.2))+1)
        points=np.linspace(own,peer,k);v=occupancy(points)
        # Avoid own footprint and the endpoint; endpoint occupancy is reported separately.
        valid=(np.linalg.norm(points-own,axis=1)>.20)&(np.linalg.norm(points-peer,axis=1)>.05)
        idx=np.flatnonzero((v>=50)&valid)
        return points[idx[0]].tolist() if len(idx) else None
    poses.sort();pt=[x[0] for x in poses];marker_frames.sort(key=lambda x:x['t']);mt=[x['t'] for x in marker_frames]
    output=[];counts=Counter();clusters=Counter();reasons=Counter()
    for frame in marker_frames:
        reasons.update(frame['reasons'])
        for c in frame['clusters']:
            if c['kind']=='rejected':continue
            counts['candidate_'+c['kind']]+=1
            if wall_distance(c['xy'])<=.25:counts['candidate_'+c['kind']+'_near_wall']+=1
    for d in detections:
        counts['odom']+=1;v=int(occupancy([d['xy']])[0]);dist=wall_distance(d['xy'])
        counts['center_occupied']+=v>=50;counts['center_unknown']+=v==-1;counts['center_outside']+=v==-2
        counts['center_within_body_radius_of_wall']+=dist<.178
        i=bisect_left(pt,d['t']);own=None
        if 0<i<len(poses) and poses[i][0]-poses[i-1][0]<.5:
            f=(d['t']-poses[i-1][0])/(poses[i][0]-poses[i-1][0]);own=(np.array(poses[i-1][1:3])*(1-f)+np.array(poses[i][1:3])*f).tolist()
        elif i<len(poses) and abs(pt[i]-d['t'])<.1:own=poses[i][1:3]
        hit=None if own is None else line(own,d['xy'])
        counts['aligned_own']+=own is not None;counts['behind_wall']+=hit is not None
        j=bisect_left(mt,d['t']);frame=marker_frames[j] if j<len(mt) and abs(mt[j]-d['t'])<.02 else None
        nearby=[c for c in frame['clusters'] if c['kind']!='rejected' and np.linalg.norm(np.array(c['xy'])-d['xy'])<.35] if frame else []
        counts['blocked_with_nearby_strong_cluster']+=hit is not None and any(c['kind']=='strong' for c in nearby)
        output.append(dict(d,own=own,wall_distance_m=dist,occupancy=v,blocked=hit is not None,
                           first_wall=hit,nearby_candidates=nearby))
    summary=dict(bag=str(path),duration_s=pt[-1]-pt[0] if pt else 0,counts=dict(counts),
        localization_frames=len(localization),localization_ready_share=sum(x.get('ready',False) for x in localization)/max(1,len(localization)),
        marker_frames=len(marker_frames),rejection_reasons=dict(reasons.most_common(15)),
        own_position_step_max_m=float(np.linalg.norm(np.diff(np.array(poses)[:,1:3],axis=0),axis=1).max()) if len(poses)>1 else 0)
    return dict(summary=summary,detections=output,own=poses,marker_frames=marker_frames,
        map=dict(resolution=res,origin=[ox,oy],width=a.shape[1],height=a.shape[0],data=list(grid.data)))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bags',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    reports=[]
    for bag in args.bags:
        report=analyze(bag);reports.append(report);print(json.dumps(report['summary']),flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(reports))


if __name__=='__main__':main()
