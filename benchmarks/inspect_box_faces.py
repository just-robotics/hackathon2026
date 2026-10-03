#!/usr/bin/env python3
"""Inspect actual visible-face support of real box shape hypotheses.

Uses audit_small_box_shapes candidates; no semantic labels or production edits.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/hsl_perception'),str(ROOT/'src/hsl_planning')]
from hsl_planning.obstacle_filter import SmallBoxFilter
from hsl_perception.geometry import cluster_xy,split_clusters


def face_support(xy):
    angles=np.linspace(0,np.pi/2,46)
    axes=np.stack((np.cos(angles),np.sin(angles)),axis=1)
    x=xy@axes.T;y=xy@np.stack((-np.sin(angles),np.cos(angles)),axis=1).T
    lx,hx=np.percentile(x,[5,95],axis=0);ly,hy=np.percentile(y,[5,95],axis=0)
    spans=np.stack((hx-lx,hy-ly),axis=1)
    edge=np.minimum.reduce((abs(x-lx),abs(hx-x),abs(y-ly),abs(hy-y)))
    fractions=np.mean(edge<=.025,axis=0)
    i=int(np.argmin(np.abs(np.max(spans,axis=1)-.15)+.15*(1-fractions)))
    u,v=x[:,i],y[:,i]
    distances=np.stack((abs(u-lx[i]),abs(u-hx[i]),abs(v-ly[i]),abs(v-hy[i])),axis=1)
    labels=np.argmin(distances,axis=1);faces=[]
    for k in range(4):
        mask=(labels==k)&(distances[:,k]<=.025)
        tangent=(v if k<2 else u)[mask]
        faces.append(dict(points=int(mask.sum()),bins=len(np.unique(np.floor(tangent/.02))),
            span=float(np.ptp(tangent)) if len(tangent) else 0.,
            rms=float(np.sqrt(np.mean(distances[mask,k]**2))) if mask.any() else None))
    return dict(dimensions=np.sort(spans[i]).tolist(),edge_fraction=float(fractions[i]),faces=faces)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('input',type=Path);p.add_argument('--candidates',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    reports=[]
    for session in json.loads(a.candidates.read_text())['sessions']:
        path=a.input/(session['session']+'.json');data=json.loads(path.read_text());archive=np.load(path.with_suffix('.npz'));f=SmallBoxFilter();g=data['grid'];f.set_grid(*[g[k] for k in ('resolution','origin','width','height','data')]);by={}
        for candidate in session['candidates']:by.setdefault(candidate['chunk'],[]).append(candidate)
        for i,candidates in by.items():
            points=archive['points'][archive['offsets'][i]:archive['offsets'][i+1]];points=points[(points[:,2]>=.06)&(points[:,2]<=.65)&f.static.foreground(points)];labels,count=cluster_xy(points[:,:2],.08);groups=split_clusters(points,labels,count)
            for candidate in candidates:
                group=min(groups,key=lambda c:np.linalg.norm(np.mean(c[:,:2],axis=0)-candidate['geometry']['center']))
                reports.append(dict(session=session['session'],**candidate,face_support=face_support(group[:,:2])))
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(reports,indent=2)+'\n');print(len(reports),'measured face hypotheses')

if __name__=='__main__':main()
