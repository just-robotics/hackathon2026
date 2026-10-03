#!/usr/bin/env python3
"""Extract one recorded foreground cluster using original segmentation; read-only."""
import argparse,json
from pathlib import Path
import numpy as np,yaml
from ament_index_python.packages import get_package_share_directory
from replay_branch_detector import reader
from jr_perception.robot_detector import stamp_seconds,cloud_to_xyz
from jr_perception.segmentation import RobotModel,cluster_xy,split_clusters,inspect_cluster,fit_circle,edge_center,line_rms,major_extent
p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('--stamp',type=float,required=True);p.add_argument('--observer',type=float,nargs=2,required=True);p.add_argument('--center',type=float,nargs=2,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
params=yaml.safe_load((Path(get_package_share_directory('jr_perception'))/'config/real.yaml').read_text())['/**']['ros__parameters'];model=RobotModel(**params['robot'])
for topic,msg,_ in reader(args.bag,{'/opponent/foreground'}):
 if abs(stamp_seconds(msg.header.stamp)-args.stamp)>.0001:continue
 points=cloud_to_xyz(msg);labels,count=cluster_xy(points[:,:2],.10);clusters=split_clusters(points,labels,count)
 results=[]
 for i,c in enumerate(clusters):
  d,reason=inspect_cluster(c,np.array(args.observer),model)
  if d is None or np.linalg.norm(d.center-args.center)>.1:continue
  rim=c[c[:,2]<=model.rim_max_z,:2];guess=edge_center(c[:,:2],rim,np.array(args.observer),model.radius)
  center,residual=fit_circle(rim,model.radius,guess);inliers=abs(residual)<=model.outlier;arc=rim[inliers]
  if len(arc) and not inliers.all():center,residual=fit_circle(arc,model.radius,center)
  angles=np.sort(np.mod(np.arctan2(arc[:,1]-center[1],arc[:,0]-center[0]),2*np.pi));span=2*np.pi-max(np.diff(np.r_[angles,angles[0]+2*np.pi])) if len(angles)>1 else 0
  results.append(dict(id=i,center=d.center.tolist(),strong=d.strong,reason=reason,points=c.tolist(),rim=rim.tolist(),
    point_count=len(c),rim_count=len(rim),rim_inliers=int(inliers.sum()),top=float(c[:,2].max()),bottom=float(c[:,2].min()),extent=major_extent(c[:,:2]),
    fit_rms=float(np.sqrt(np.mean(residual**2))),line_rms=line_rms(arc),arc_span_deg=float(np.degrees(span))))
 args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(dict(stamp=args.stamp,observer=args.observer,selected=args.center,clusters=results)))
 for x in results:print(json.dumps({k:v for k,v in x.items() if k not in ('points','rim')}))
 break
else:raise SystemExit('Frame not found')
