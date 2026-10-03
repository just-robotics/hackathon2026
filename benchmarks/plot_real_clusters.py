#!/usr/bin/env python3
"""Inspect real measured cluster geometry. Titles are hypotheses, not labels."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/hsl_perception'),str(ROOT/'src/hsl_planning')]
from hsl_perception.geometry import cluster_xy,split_clusters
from hsl_perception.geometry import StaticBackground
from hsl_planning.obstacle_filter import classify_cluster


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('input',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected={}
    for path in sorted(a.input.glob('20*.json')):
        if not path.with_suffix('.npz').exists():continue
        data=json.loads(path.read_text());g=data['grid'];background=StaticBackground(*[g[k] for k in ('resolution','origin','width','height','data')])
        archive=np.load(path.with_suffix('.npz'));points,offsets=archive['points'],archive['offsets']
        for sample in data['replay']['samples'][::5]:
            i=sample['chunk'];cloud=points[offsets[i]:offsets[i+1]]
            cloud=cloud[np.isfinite(cloud).all(axis=1)&(cloud[:,2]>=.06)&(cloud[:,2]<=.65)]
            cloud=cloud[background.foreground(cloud)]
            labels,count=cluster_xy(cloud[:,:2],.08)
            for group in split_clusters(cloud,labels,count):
                if len(group)<20:continue
                ignored,diag=classify_cluster(group);dims=diag['dimensions'];height=diag['height']
                category=('narrow' if ignored else 'wide_low' if dims[1]>=.30 and .12<=height<=.26 else 'other')
                key=(path.stem,category)
                if category=='other':continue
                if key not in selected or len(group)>len(selected[key][0]):selected[key]=(group,diag,sample['t'])
    rows=[]
    a.output.mkdir(parents=True,exist_ok=True)
    for index,((session,category),(group,diag,stamp)) in enumerate(selected.items()):
        fig=plt.figure(figsize=(9,4));center=np.mean(group[:,:2],axis=0);local=group.copy();local[:,:2]-=center
        ax=fig.add_subplot(121);ax.scatter(local[:,0],local[:,1],c=local[:,2],s=6,vmin=0,vmax=.45);ax.set_aspect('equal');ax.set_xlabel('x relative (m)');ax.set_ylabel('y relative (m)');ax.grid()
        ax=fig.add_subplot(122,projection='3d');ax.scatter(local[:,0],local[:,1],local[:,2],c=local[:,2],s=6,vmin=0,vmax=.45);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.set_zlabel('z (m)');ax.set_zlim(0,.5)
        fig.suptitle(f'{session[9:15]} {category} hypothesis; dimensions {diag["dimensions"][0]:.2f}/{diag["dimensions"][1]:.2f}, top {diag["height"]:.2f}m')
        fig.tight_layout();file=a.output/f'{index:02d}-{category}.png';fig.savefig(file,dpi=130);plt.close(fig)
        rows.append(dict(session=session,stamp=stamp,category=category,diagnostics=diag,image=file.name))
    (a.output/'index.json').write_text(json.dumps(rows,indent=2)+'\n');print(f'{len(rows)} measured clusters exported')

if __name__=='__main__':main()
