#!/usr/bin/env python3
"""Score recorded detector outputs against explicitly limited real-bag labels.

The orbit center label is approximate; reports retain its tolerance and source.
Unlabelled frames are never counted as true or false classifications.
"""

import argparse
import json
import math
import re
import statistics
from pathlib import Path


def session(path):
    parts = Path(path).parts
    pattern = re.compile(r'^20\d{6}T\d{6}\.\d+Z-(?:autonomous|bag)$')
    return next((part for part in parts if pattern.fullmatch(part)), str(path))


def normalize(row):
    stamp = row.get('stamp_s', row.get('stamp'))
    if stamp is None:
        return None
    measured = bool(row.get('has_measurement', row.get('measured', False)))
    xy = row.get('measurement_xy')
    if xy is None and 'measurement' in row:
        m = row['measurement']
        if isinstance(m, dict):
            xy = m.get('xy', m.get('center_xy', m.get('center_map')))
    if isinstance(xy, dict):
        xy = [xy.get('x'), xy.get('y')]
    if xy is None and row.get('x') is not None and row.get('y') is not None:
        xy = [row['x'], row['y']]
    if not measured:
        xy = None
    prediction = row.get('prediction_xy')
    if prediction is None and row.get('has_prediction') and isinstance(row.get('prediction'), dict):
        p = row['prediction']
        prediction = p.get('xy', p.get('center_map'))
    if isinstance(prediction, dict):
        prediction = [prediction.get('x'), prediction.get('y')]
    if prediction is None and not measured and row.get('x') is not None and row.get('y') is not None:
        prediction = [row['x'], row['y']]
    timing = row.get('timing_ms')
    if timing is None and isinstance(row.get('timing'), dict):
        timing = row['timing'].get('total_ms')
    if xy is not None:
        xy = [float(xy[0]), float(xy[1])]
    if prediction is not None:
        prediction = [float(prediction[0]), float(prediction[1])]
    return dict(stamp_s=float(stamp), measured=measured and xy is not None,
                xy=xy, prediction_xy=prediction,
                track_id=row.get('track_id'), status=row.get('status', 'ready'),
                timing_ms=float(timing) if timing is not None else None)


def load_runs(paths):
    runs = {}
    for path in paths:
        path = Path(path)
        if path.suffix == '.jsonl':
            for line in path.open():
                if not line.strip():
                    continue
                raw = json.loads(line)
                bag = session(raw.get('bag', path.parent.name))
                row = normalize(raw)
                if row is not None:
                    runs.setdefault(bag, []).append(row)
        else:
            data = json.loads(path.read_text())
            for result in data if isinstance(data, list) else [data]:
                bag = session(result.get('summary', {}).get('bag', result.get('bag', path.parent.name)))
                for raw in result.get('frames', []):
                    row = normalize(raw)
                    if row is not None:
                        runs.setdefault(bag, []).append(row)
    for bag in runs:
        runs[bag].sort(key=lambda r: r['stamp_s'])
    return runs


def quantiles(values):
    values = sorted(values)
    if not values:
        return None
    def at(q):
        p = q * (len(values)-1)
        lo = int(math.floor(p)); hi = int(math.ceil(p))
        return values[lo] + (values[hi]-values[lo]) * (p-lo)
    return {'p50': at(.50), 'p95': at(.95), 'p99': at(.99)}


def nominal_period(rows):
    gaps = [b['stamp_s']-a['stamp_s'] for a,b in zip(rows, rows[1:])]
    gaps = [x for x in gaps if 0 < x < 0.5]
    return statistics.median(gaps) if gaps else 0.1


def episodes(stamps, period):
    if not stamps:
        return []
    groups = [[stamps[0]]]
    for stamp in stamps[1:]:
        if stamp - groups[-1][-1] <= max(.25, period*2.5):
            groups[-1].append(stamp)
        else:
            groups.append([stamp])
    return [{'start_s': g[0], 'end_s': g[-1], 'duration_s': g[-1]-g[0]+period,
             'frames': len(g)} for g in groups]


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def score_label(label, rows, period):
    start = label.get('start_s'); end = label.get('end_s')
    frames = [r for r in rows if (start is None or r['stamp_s'] >= start) and
              (end is None or r['stamp_s'] <= end)]
    answer = {'visibility': label['visibility'], 'source': label['source'],
              'confidence': label['confidence'], 'scans': len(frames)}
    if 'frame' in label:
        answer['frame'] = label['frame']
    if 'center' in label:
        answer['center'] = label['center']
    kind = label['visibility']
    if kind == 'absent':
        false = [r['stamp_s'] for r in frames if r['measured']]
        events = episodes(false, period)
        answer.update(false_confirmed_frames=len(false), false_episodes=events,
                      false_episode_count=len(events),
                      false_total_s=sum(e['duration_s'] for e in events))
    elif kind == 'no_robot_in_region':
        c = label['region']['center_xy_m']; radius = label['region']['radius_m']
        false = [r['stamp_s'] for r in frames if r['measured'] and distance(r['xy'], c) <= radius]
        answer.update(false_confirmed_frames=len(false), false_episodes=episodes(false, period))
    elif kind == 'visible':
        c = label['center']['xy_m']; tol = label['center']['tolerance_m']
        seen = [r for r in frames if r['measured'] and distance(r['xy'], c) <= tol]
        errors = [distance(r['xy'], c) for r in seen]
        displacements = [distance(a['xy'], b['xy']) for a,b in zip(seen, seen[1:])
                         if b['stamp_s']-a['stamp_s'] <= max(.25, period*2.5)]
        missing = [r['stamp_s'] for r in frames if r not in seen]
        missed = episodes(missing, period)
        identities = [r['track_id'] for r in seen if r['track_id'] is not None]
        answer.update(measured_in_tolerance=len(seen), measured_anywhere=sum(r['measured'] for r in frames),
                      visible_detection_share=len(seen)/len(frames) if frames else None,
                      position_error_m=quantiles(errors), jitter_step_m=quantiles(displacements),
                      first_acquisition_delay_s=(seen[0]['stamp_s']-frames[0]['stamp_s']) if seen and frames else None,
                      max_loss_s=max((e['duration_s'] for e in missed), default=0.0),
                      loss_episodes=missed,
                      identity_switches=sum(a != b for a,b in zip(identities, identities[1:])))
    elif kind == 'measurement_required_for_fresh_output':
        last = None; drift = []
        for r in rows:
            if r['stamp_s'] > (end if end is not None else float('inf')):
                break
            if r['measured']:
                last = r['xy']
            elif r in frames and r['prediction_xy'] is not None and last is not None:
                drift.append(distance(r['prediction_xy'], last))
        answer.update(prediction_without_measurement_frames=sum(
            r['prediction_xy'] is not None and not r['measured'] for r in frames),
            max_prediction_drift_from_last_measurement_m=max(drift) if drift else None,
            measured_frames=sum(r['measured'] for r in frames))
    return answer


def evaluate(runs, labels):
    reports = {}
    for bag, rows in runs.items():
        period = nominal_period(rows)
        measured = [r for r in rows if r['measured']]
        predicted = [r for r in rows if r['prediction_xy'] is not None and not r['measured']]
        timings = [r['timing_ms'] for r in rows if r['timing_ms'] is not None]
        reports[bag] = {
            'scans': len(rows), 'nominal_scan_period_s': period,
            'measured_outputs': len(measured), 'prediction_only_frames': len(predicted),
            'measured_share': len(measured)/len(rows) if rows else None,
            'timing_ms': quantiles(timings),
            'core_throughput_hz': len(timings)/(sum(timings)/1000) if timings and sum(timings) else None,
            'status_counts': {status: sum(r['status'] == status for r in rows)
                              for status in sorted(set(r['status'] for r in rows))},
            'labels': [score_label(l, rows, period) for l in labels if session(l['bag']) == bag],
        }
    return reports


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('runs', nargs='+', type=Path)
    ap.add_argument('--labels', type=Path, default=Path(__file__).parent/'labels/real_detector_key_intervals.json')
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    labels = json.loads(args.labels.read_text())['labels']
    report = evaluate(load_runs(args.runs), labels)
    rendered = json.dumps(report, indent=2, ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered+'\n')
    else:
        print(rendered)


if __name__ == '__main__':
    main()
