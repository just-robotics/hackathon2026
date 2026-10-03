#!/usr/bin/env python3
"""Compare real detector/filter policies on clouds reconstructed by audit_real_bags.

No semantic labels or recall are inferred. Box overlap is a conflict between
two shape hypotheses. Both profiles run the current C++ detector executable.
"""
import argparse
import json
import os
from pathlib import Path
import sys

os.environ['OPENBLAS_NUM_THREADS'] = '1'
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/hsl_perception'), str(ROOT/'src/hsl_planning')]
from hsl_perception.geometry import StaticBackground
from hsl_perception.profiles import REAL_PARAMETERS
from native_detector import NativeReplay
from hsl_planning.obstacle_filter import SmallBoxFilter


def replay(path):
    data = json.loads(path.read_text())
    archive = np.load(path.with_suffix('.npz'))
    points, offsets = archive['points'], archive['offsets']
    grid = data['grid']
    grid_args = [grid[k] for k in ('resolution', 'origin', 'width', 'height', 'data')]
    background = StaticBackground(*grid_args)
    old_background = StaticBackground(*grid_args)
    detectors = {'before': NativeReplay('default', max_height=.46), 'after': NativeReplay('real')}
    filters = {key: SmallBoxFilter() for key in detectors}
    filters['before'].static = old_background
    filters['after'].static = background
    # Independent box hypothesis: production track protection must not hide
    # a detector/box conflict in the evaluation report. No effect on navigation.
    independent_boxes = SmallBoxFilter()
    independent_boxes.static = background
    previous = None
    counts = {key: dict(fresh=0, box_hypothesis_overlap=0, invalid_center=0,
                       ignored_frames=0, ignored_points=0, protected_clusters=0,
                       independent_box_overlap=0)
              for key in detectors}
    rows = []
    for sample in data['replay']['samples']:
        i, stamp = sample['chunk'], sample['t']
        cloud = points[offsets[i]:offsets[i+1]]
        protected = ([previous[1]] if previous and 0 <= stamp-previous[0] <= .3 else [])
        row = dict(t=stamp, chunk=i)
        _, independent_diag = independent_boxes.filter(cloud, stamp)
        independent_centers = [c['center'] for c in independent_diag['clusters']
                               if c['reason'] in ('small_box', 'small_box_cached', 'small_box_cached_sparse')]
        row['independent_boxes'] = independent_centers
        row['independent_filter'] = independent_diag
        for key, detector in detectors.items():
            # Only earlier observations protect the body: no future-track leak.
            _, box_diag = filters[key].filter(cloud, stamp,
                protected_centers=protected if key == 'after' else ())
            box_centers = [c['center'] for c in box_diag['clusters']
                           if c['reason'] in ('small_box', 'small_box_cached', 'small_box_cached_sparse')]
            track, diag = detector.step(cloud, sample['sensor'],
                old_background if key == 'before' else background, stamp)
            fresh = track is not None and abs(track.last_update-stamp) < 1e-5
            center = track.mean[:2].tolist() if fresh else None
            count = counts[key]
            count['fresh'] += int(fresh)
            count['ignored_frames'] += int(box_diag['ignored_points'] > 0)
            count['ignored_points'] += box_diag['ignored_points']
            count['protected_clusters'] += sum(c['reason'] == 'tracked_robot' for c in box_diag['clusters'])
            if fresh:
                count['invalid_center'] += int(not background.free_center(center))
                count['box_hypothesis_overlap'] += int(any(np.linalg.norm(track.mean[:2]-b) < .25 for b in box_centers))
                count['independent_box_overlap'] += int(any(
                    np.linalg.norm(track.mean[:2]-b) < .25 for b in independent_centers))
                if key == 'after':
                    previous = (stamp, center)
            row[key] = dict(track=center, speed=float(track.speed) if fresh else None,
                            detector=diag, boxes=box_centers, filter=box_diag)
        rows.append(row)
    return dict(session=data['session'], cloud_samples=len(rows), counts=counts, rows=rows)


def plot(input_root, reports, destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    tags = ('145734', '151308', '151820', '152152', '152423')
    selected = [r for tag in tags for r in reports if 'T'+tag in r['session']]
    if not selected:
        selected = reports[:5]
    if not selected:
        return
    fig, axes = plt.subplots(1, len(selected), figsize=(4*len(selected), 5), squeeze=False)
    for ax, report in zip(axes[0], selected):
        data = json.loads((input_root/(report['session']+'.json')).read_text())
        g = data['grid']; cells = np.array(g['data']).reshape(g['height'], g['width'])
        y, x = np.where(cells >= 50)
        ax.scatter(g['origin'][0]+x*g['resolution'], g['origin'][1]+y*g['resolution'], s=2, c='gray')
        for key, color in [('before', 'orange'), ('after', 'blue')]:
            xy = [r[key]['track'] for r in report['rows'] if r[key]['track'] is not None]
            if xy:
                a = np.array(xy); ax.scatter(a[:,0], a[:,1], s=5, c=color, label=key)
        ax.set_title(report['session'].split('T')[1][:6]); ax.set_aspect('equal')
        ax.set_xlim(-.5,3.6); ax.set_ylim(-.6,4.5); ax.legend()
    fig.suptitle('Real cloud replay: native default / real profile track hypotheses (no semantic ground truth)')
    fig.tight_layout(); fig.savefig(destination/'tracks-before-after.png', dpi=130)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.input.is_dir():
        parser.error(f'archive directory does not exist: {args.input}')
    args.output.mkdir(parents=True, exist_ok=True)
    reports = []
    for path in sorted(args.input.glob('*autonomous.json')):
        if not path.with_suffix('.npz').exists():
            continue
        report = replay(path)
        reports.append(report)
        (args.output/(path.stem+'-shapes.json')).write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k != 'rows'}), flush=True)
    if not reports:
        raise RuntimeError('no JSON/NPZ cloud archives replayed; no evaluation performed')
    summary = dict(baseline_profile="native default", real_parameters=REAL_PARAMETERS,
        scope='recorded scan-time TF; offline tracker; no semantic ground truth; hypothesis overlap is not false-positive rate; independent_box_overlap ignores track protection to expose circular masking',
        sessions=[{k:v for k,v in report.items() if k != 'rows'} for report in reports])
    (args.output/'shapes-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    plot(args.input, reports, args.output)


if __name__ == '__main__':
    main()
