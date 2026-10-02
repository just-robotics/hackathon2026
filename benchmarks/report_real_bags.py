#!/usr/bin/env python3
"""Plot unlabelled real-bag clouds and sensor-only box hypotheses after audit_real_bags."""
import argparse,sys,json,numpy as np
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/hsl_planning'),str(ROOT/'src/hsl_perception')]
from hsl_planning.obstacle_filter import SmallBoxFilter
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('root',type=Path)
args=parser.parse_args()
root=args.root
summary=[]
for p in sorted(root.glob('*autonomous.json')):
 d=json.loads(p.read_text());g=d.get('grid');zpath=p.with_suffix('.npz')
 if not g or not zpath.exists():continue
 z=np.load(zpath);points=z['points'];off=z['offsets'];f=SmallBoxFilter();f.set_grid(g['resolution'],g['origin'],g['width'],g['height'],g['data'])
 rows=[]
 for sample in d['replay']['samples']:
  i=sample['chunk'];_,diag=f.filter(points[off[i]:off[i+1]],sample["t"])
  rows.append({'t':sample['t'],'ignored_points':diag['ignored_points'],'small_boxes':[c for c in diag['clusters'] if c['reason'].startswith('small_box')]})
 active=d['active_windows'];mp=[(t,v) for t,v in d['recorded']['/navigation/mppi_diagnostics'] if any(a<=t<=b for a,b in active)]
 failure=[v for t,v in mp if v.get('result')=='swept_collision']
 summary.append({'session':d['session'],'role':d['role'],'active_seconds':sum(b-a for a,b in active),'measured_speed_mps':d.get('active_measured_speed_mps'),'speed_time_coverage_s':d.get('speed_time_coverage_s'),'recovery_seconds':sum(max(0,min(b,y)-max(a,x)) for a,b in active for x,y in d['recovery_windows']),'swept_failures':len(failure),'swept_index_median':float(np.median([v['rejected_trajectory_index'] for v in failure])) if failure else None,'small_box_frames':sum(bool(r['ignored_points']) for r in rows),'ignored_points':sum(r['ignored_points'] for r in rows),'old_detector_messages':d['counts'].get('/navigation/opponent',0),'source_replay_detections':sum(bool(s['tracks']['source']['position']) for s in d['replay']['samples']),'simulation_replay_detections':sum(bool(s['tracks']['simulation']['position']) for s in d['replay']['samples']),'cloud_samples':len(rows),'rows':rows})
(root/'obstacle_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps([{k:v for k,v in row.items() if k!='rows'} for row in summary],indent=2))
fig,axes=plt.subplots(2,3,figsize=(13,10))
paths=[root/(r['session']+'.json') for r in summary if r['cloud_samples']][-6:]
for ax,p in zip(axes.ravel(),paths):
 tag=p.stem.split('T')[1][:6];d=json.loads(p.read_text());z=np.load(p.with_suffix('.npz'));points=z['points'];off=z['offsets'];g=d['grid'];f=SmallBoxFilter();f.set_grid(g['resolution'],g['origin'],g['width'],g['height'],g['data'])
 a=np.array(g['data']).reshape(g['height'],g['width']);yy,xx=np.where(a>=50);ax.scatter(g['origin'][0]+xx*g['resolution'],g['origin'][1]+yy*g['resolution'],s=3,c='#999999')
 ignored=[];foreground=[]
 for i in range(0,len(d['replay']['samples']),4):
  pts=points[off[i]:off[i+1]];band=pts[(pts[:,2]>.08)&(pts[:,2]<.6)&f.static.foreground(pts)];foreground.extend(band[:,[0,1,2]].tolist());_,diag=f.filter(pts,d["replay"]["samples"][i]["t"]);ignored.extend([c['center'] for c in diag['clusters'] if c['reason'].startswith('small_box')])
 if foreground:
  a=np.array(foreground);ax.scatter(a[:,0],a[:,1],c=a[:,2],s=1,cmap='viridis',vmin=.05,vmax=.5,alpha=.5)
 if ignored:a=np.array(ignored);ax.scatter(a[:,0],a[:,1],c='red',s=25,marker='x')
 a=np.array(d['own']);ax.plot(a[:,1],a[:,2],c='blue',lw=1)
 a=np.array([v for t,v in d['recorded']['/navigation/opponent']]);ax.scatter(a[:,0],a[:,1],s=2,c='orange') if len(a) else None
 ax.set_title(tag+' '+d['role']);ax.set_aspect('equal');ax.set_xlim(-.5,3.7);ax.set_ylim(-.6,4.5)
fig.suptitle('Real bags: blue=self, orange=recorded rival, red=small-box hypothesis; clouds colored by height')
fig.tight_layout();fig.savefig(root/'cloud_overview.png',dpi=160)
