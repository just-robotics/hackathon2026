#!/usr/bin/env python3
"""Replay the real upstream LiDAR filter at recorded TF without DDS/hardware.

Near-cell counts diagnose sensor marks around the current footprint; they are
not collision outcomes or closed-loop navigation metrics.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/hsl_real')]
from hsl_real.lidar_filter import filter_cloud
from hsl_real.lidar_filter_core import DEFAULT_PARAMETERS
from audit_real_bags import messages, stamp, xyz, transform


def replay(session, output):
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer
    tf = Buffer(cache_time=Duration(seconds=6000))
    files = sorted((session/'bag').glob('*.mcap'))
    for file in files:
        for topic, msg, _ in messages(file, ['/tf','/tf_static']):
            for item in msg.transforms:
                (tf.set_transform_static if topic == '/tf_static' else tf.set_transform)(item, 'bag')
    totals = Counter(); rows = []; snapshots = []
    for file in files:
        for _, msg, _ in messages(file, ['/livox/lidar']):
            filtered, diagnostic = filter_cloud(msg, DEFAULT_PARAMETERS)
            totals.update({k:v for k,v in diagnostic.items() if k.endswith('points')})
            try:
                pose = tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            except Exception:
                totals['missing_tf_scans'] += 1
                continue
            center = np.array([pose.transform.translation.x, pose.transform.translation.y])
            values = {}; near_clouds = []
            for key, cloud in [('before',msg),('after',filtered)]:
                points = transform(xyz(cloud), pose)
                distance = np.linalg.norm(points[:,:2]-center,axis=1)
                band = points[(distance >= .25) & (points[:,2] >= .08) & (points[:,2] <= .60)]
                cells = np.unique(np.floor(band[:,:2]/.05).astype(int), axis=0)
                near = np.linalg.norm((cells+.5)*.05-center,axis=1) < .27
                count = int(near.sum())
                values[key+'_near_cells'] = count
                totals[key+'_near_cell_scans'] += int(count > 0)
                totals[key+'_near_cells'] += count
                near_clouds.append(band[np.linalg.norm(band[:,:2]-center,axis=1)<.6].tolist())
            rows.append(dict(stamp_s=stamp(msg.header.stamp), sensor=center.tolist(), **values))
            if values['before_near_cells'] and len(snapshots)<8:
                snapshots.append(dict(sensor=center.tolist(), before=near_clouds[0], after=near_clouds[1]))
    report = dict(session=str(session), scans=len(rows), parameters=DEFAULT_PARAMETERS,
        scope='all recorded scans; scan-time TF; 0.05m cell raster; near cells within 0.27m; not closed-loop collision or speed validation',
        totals=dict(totals), rows=rows, snapshots=snapshots)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','snapshots')}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import rclpy
    rclpy.init()
    sessions = [args.root] if (args.root/'bag').exists() else []
    sessions += sorted(args.root.glob('*autonomous'))
    for session in sessions:
        replay(session, args.output/(session.name+'-lidar-filter.json'))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
