#!/usr/bin/env python3
"""Read-only costmap/footprint diagnosis of a real recording (ROS image)."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from audit_real_bags import messages,stamp

def main():
 p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 own=None;costmap=None;static=None;rows=[];frames=set()
 topics=['/tf_static','/navigation/self','/native_mppi/costmap_raw','/map','/navigation/mppi_diagnostics']
 for file in sorted((args.session/'bag').glob('*.mcap')):
  for topic,msg,t in messages(file,topics):
   if topic=='/tf_static':
    frames.update(m.child_frame_id for m in msg.transforms)
   elif topic=='/navigation/self':own=msg
   elif topic=='/native_mppi/costmap_raw':costmap=msg
   elif topic=='/map':static=msg
   elif topic=='/navigation/mppi_diagnostics':
    d=json.loads(msg.data)
    if d.get('result')!='swept_collision' or own is None or costmap is None:continue
    m=costmap.metadata;r=m.resolution;o=m.origin.position
    a=np.asarray(costmap.data).reshape(m.size_y,m.size_x);yy,xx=np.where(a>=254)
    xy=np.column_stack((o.x+(xx+.5)*r,o.y+(yy+.5)*r));pos=own.pose.pose.position
    distance=np.linalg.norm(xy-[pos.x,pos.y],axis=1);near=xy[distance<.27]
    if len(near):
     known=np.zeros(len(near),dtype=bool)
     if static:
      g=static.info;s=np.array(static.data).reshape(g.height,g.width)
      indices=np.floor((near-[g.origin.position.x,g.origin.position.y])/g.resolution).astype(int);x,y=indices.T
      inside=(x>=0)&(y>=0)&(x<g.width)&(y<g.height);known[inside]=s[y[inside],x[inside]]>=50
     rows.append(dict(t=t,own=[pos.x,pos.y],index=d['rejected_trajectory_index'],nearest_lethal_m=float(distance.min()),near_lethal_xy=near.tolist(),static_hits=int(known.sum()),dynamic_hits=int((~known).sum())))
 args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps({'session':str(args.session),'static_frames':sorted(frames),'rows':rows},indent=2))
 print(args.session.name,'frames',sorted(frames),'failed_near_self',len(rows),'static',sum(r['static_hits'] for r in rows),'dynamic',sum(r['dynamic_hits'] for r in rows));print(rows[:2])
if __name__=='__main__':main()
