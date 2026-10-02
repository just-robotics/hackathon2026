#!/usr/bin/env python3
"""Evaluate paired detector traces against the other robot's own simulation pose.

Read-only report: truth and fixture positions are never navigation inputs.
No visibility recall is claimed (occlusion/observable surfaces are not labelled).
"""
import argparse
from bisect import bisect_left
import json
from math import hypot, cos, sin
from pathlib import Path
import statistics
import yaml


def interpolate(rows, stamps, moment):
    index = bisect_left(stamps, moment)
    if not 0 < index < len(rows):
        return None
    a,b = rows[index-1],rows[index]
    gap=b['sim_t_s']-a['sim_t_s']
    if not 0 < gap <= .2:
        return None
    f=(moment-a['sim_t_s'])/gap
    return [a[k]+f*(b[k]-a[k]) for k in ('own_x_m','own_y_m')]


def box_distance(point, box):
    x,y,yaw=box['pose'];dx,dy=point[0]-x,point[1]-y
    u,v=cos(yaw)*dx+sin(yaw)*dy,-sin(yaw)*dx+cos(yaw)*dy
    return hypot(max(abs(u)-box['size'][0]/2,0), max(abs(v)-box['size'][1]/2,0))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('series',type=Path);p.add_argument('--output',type=Path)
    p.add_argument('--obstacles',type=Path,default=Path(__file__).resolve().parents[1]/'config/simulation_obstacles.yaml')
    args=p.parse_args();cfg=yaml.safe_load(args.obstacles.read_text());boxes=cfg['boxes'] if cfg['enabled'] else []
    runs=json.loads((args.series/'index.json').read_text());reports=[]
    for index,run in enumerate(runs):
        if 'outcome' not in run:continue
        outcome=run['outcome'];start,end=outcome['started_at_sim_s'],outcome['finished_at_sim_s']
        traces=[json.loads((args.series/f'{index:02d}-trace-{which}.json').read_text()) for which in ('first','second')]
        for observer,trace in enumerate(traces):
            rows=traces[1-observer].get('time_series',[]);stamps=[r['sim_t_s'] for r in rows]
            errors=[];box_false=0;cycles=[];detected=0;eligible=0
            for d in trace.get('detector_reports',[]):
                if not start <= d['stamp_s'] <= end:continue
                cycles.append(d['cycle_ms'])
                if not d['detected']:continue
                detected+=1;truth=interpolate(rows,stamps,d['stamp_s'])
                if truth is None:continue
                eligible+=1;error=hypot(d['track_xy'][0]-truth[0],d['track_xy'][1]-truth[1]);errors.append(error)
                if error>.3 and any(box_distance(d['track_xy'],b)<.23 for b in boxes):box_false+=1
            reports.append(dict(seed=run['seed'],observer=observer,role=run['roles'][observer],outcome=outcome['event'],
                diagnostics=len(cycles),detections=detected,aligned_detections=eligible,
                position_error_median_m=statistics.median(errors) if errors else None,
                position_error_p90_m=sorted(errors)[int(.9*(len(errors)-1))] if errors else None,
                error_over_03m=sum(e>.3 for e in errors),box_near_false_samples=box_false,
                cycle_ms_median=statistics.median(cycles) if cycles else None,
                cycle_ms_p90=sorted(cycles)[int(.9*(len(cycles)-1))] if cycles else None))
    report={'series':str(args.series),'rows':reports,
            'scope':'active referee window only; interpolation <=0.2s; proximity error, not semantic or visibility recall; box poses evaluation-only and INITIAL, so box-near counts cannot label boxes displaced by contact',
            'obstacle_config':cfg}
    text=json.dumps(report,indent=2)
    if args.output:args.output.write_text(text+'\n')
    print(text)


if __name__=='__main__':main()
