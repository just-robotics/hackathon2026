#!/usr/bin/env python3
"""Inspect measured vertical support of box hypotheses; no semantic labels.

Input is audit_real_bags JSON/NPZ. Optional tracks come from replay_real_shapes;
proximity to a track is an ambiguity marker, not a robot/box label.
"""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import argparse
import json
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/hsl_perception'), str(ROOT/'src/hsl_planning')]
from hsl_perception.geometry import cluster_xy, split_clusters
from hsl_planning.obstacle_filter import SmallBoxFilter, classify_cluster


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('input', type=Path)
    p.add_argument('--tracks', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    reports = []
    for path in sorted(a.input.glob('*autonomous.json')):
        archive_path = path.with_suffix('.npz')
        if not archive_path.exists():
            continue
        data = json.loads(path.read_text())
        archive = np.load(archive_path)
        g = data['grid']
        f = SmallBoxFilter()
        f.set_grid(*[g[k] for k in ('resolution','origin','width','height','data')])
        rows = json.loads((a.tracks/(path.stem+'-shapes.json')).read_text())['rows']
        by_chunk = {r['chunk']: r['after']['track'] for r in rows}
        candidates = []
        for sample in data['replay']['samples']:
            i = sample['chunk']
            points = archive['points'][archive['offsets'][i]:archive['offsets'][i+1]]
            points = points[np.isfinite(points).all(axis=1) & (points[:,2]>=.06) & (points[:,2]<=.65)]
            points = points[f.static.foreground(points)]
            labels, count = cluster_xy(points[:,:2], .08)
            for cluster in split_clusters(points, labels, count):
                positive, diag = classify_cluster(cluster)
                if not positive:
                    continue
                center = np.mean(cluster[:,:2], axis=0)
                track = by_chunk.get(i)
                candidates.append(dict(chunk=i, stamp=sample['t'], geometry=diag,
                    near_track=bool(track is not None and np.linalg.norm(center-track)<.25),
                    low_p5_m=float(np.percentile(cluster[:,2], 5)),
                    gap_share=float(np.mean((cluster[:,2]>=.25)&(cluster[:,2]<.34))),
                    lower_band_share=float(np.mean((cluster[:,2]>=.115)&(cluster[:,2]<.20))),
                    height_histogram=np.histogram(cluster[:,2], bins=np.arange(.06,.47,.04))[0].tolist()))
        reports.append(dict(session=path.stem, candidates=candidates))
        print(path.stem, len(candidates), 'shape hypotheses', flush=True)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(dict(scope='measured shape hypotheses, not object labels',
                                        sessions=reports), indent=2)+'\n')

if __name__ == '__main__':
    main()
