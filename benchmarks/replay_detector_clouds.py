#!/usr/bin/env python3
"""Replay saved map-frame clouds through the current feature/detector core.

Truth positions only label the output offline; they never enter detect().
Requires NumPy, no ROS installation.
"""
import argparse
import collections
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
import sys
import numpy as np
sys.path.insert(0, str(ROOT/'src/hsl_perception'))
from hsl_perception.core import Detector, StaticBackground
from hsl_perception.segmentation import RobotModel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--height', type=float, default=0.46)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--max-gap-share', type=float, default=.12)
    parser.add_argument('--line-ratio', type=float, default=.35)
    parser.add_argument('--arc-span', type=float, default=90.)
    parser.add_argument('--rectangle-ratio', type=float, default=.70)
    args = parser.parse_args()
    if not math.isfinite(args.height) or not 0.08 <= args.height <= 0.60:
        parser.error("height must be within [0.08, 0.60]")
    if args.recording.suffix == '.jsonl':
        data = {'samples': []}
        for line in args.recording.read_text().splitlines():
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue  # interrupted final line after evaluation cleanup
            if 'grid' in row:
                data['grid'] = row['grid']
            if 'sample' in row:
                data['samples'].append(row['sample'])
    else:
        data = json.loads(args.recording.read_text())
    full_clouds = 'grid' in data
    grid = data['grid'] if full_clouds else data['replay_grid']
    if full_clouds:
        grid = dict(grid, origin_x=grid['origin'][0], origin_y=grid['origin'][1])
    static = StaticBackground(grid['resolution'], [grid['origin_x'],grid['origin_y']],
                              grid['width'],grid['height'],grid['data'])
    detectors = [Detector(RobotModel(max_height=args.height, max_gap_share=args.max_gap_share, line_ratio=args.line_ratio), strong_arc_min_span_deg=args.arc_span, allow_merged_strong=False, strong_min_inlier_fraction=.95, strong_rectangle_ratio=args.rectangle_ratio) for _ in range(2)]
    clouds = data['cloud_samples'] if not full_clouds else [
        dict(observer=s['observer'], stamp_sim_s=s['stamp'], own_xy=s['own'],
             sensor=s['sensor'], peer_truth_xy=s['peer'], points=s['points'])
        for s in data['samples']]
    outputs=[]
    for sample in clouds:
        track, diagnostic = detectors[int(sample['observer']=='peer_scan')].step(
            sample['points'], sample.get('sensor', [*sample['own_xy'], .374]), static, sample['stamp_sim_s'])
        detected = track is not None and abs(track.last_update-sample['stamp_sim_s']) < 1e-6
        outputs.append(None if not detected else (track.mean[0],track.mean[1],track.hits))
    rows=[]
    counts=collections.Counter()
    for sample, output in zip(clouds, outputs):
        row = dict(observer=sample['observer'], stamp_sim_s=sample['stamp_sim_s'])
        if output is None:
            row['label']='no_detection'
        else:
            x,y,hits=output
            peer_error=math.dist((x,y),sample['peer_truth_xy'])
            box_error=math.dist((x,y),data['fixture_map_xy']) if 'fixture_map_xy' in data else math.inf
            label=('ambiguous' if peer_error<.4 and box_error<.4 else
                   'peer_near' if peer_error<.3 else 'fixture_near' if box_error<.4 else 'other')
            row.update(label=label,centre=[x,y],hits=int(hits),
                       peer_error_m=peer_error,fixture_error_m=box_error if math.isfinite(box_error) else None)
        counts[row['label']]+=1
        rows.append(row)
    report=dict(recording=str(args.recording),height=args.height,cloud_count=len(clouds),
                counts=dict(counts),rows=rows,
                scope='offline replay; proximity labels are not exhaustive semantic truth or visibility recall')
    if args.output:
        args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({key:value for key,value in report.items() if key!='rows'},indent=2))


if __name__=='__main__':
    main()
