#!/usr/bin/env python3
"""Offline straight-corridor corrections and arc motion in referee windows.

Uses observed global-path heads and measured odometry, never changes control.
Straight classification describes the observed reference, not corridor clearance.
"""
import argparse
import json
from math import atan2, hypot, pi, sqrt
from pathlib import Path


def straight_reference_heading(sample, lookahead=0.45, heading_tolerance=0.05):
    points = sample.get('global_head_xy_m') or []
    if len(points) < 2:
        return None
    x, y = sample.get('own_x_m'), sample.get('own_y_m')
    if x is None or y is None:
        return None
    segments = []
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = hypot(dx, dy)
        if length > 1e-6:
            projection = max(0., min(1., ((x-a[0])*dx + (y-a[1])*dy)/length**2))
            distance = hypot(x-a[0]-projection*dx, y-a[1]-projection*dy)
            segments.append((length, atan2(dy, dx), projection, distance))
    if not segments:
        return None
    nearest = min(range(len(segments)), key=lambda i: segments[i][3])
    # Missing or distant route geometry cannot prove a straight reference.
    if segments[nearest][3] > 0.45:
        return None
    heading = segments[nearest][1]
    length = 0.
    for i in range(nearest, len(segments)):
        segment_length, angle, projection, _ = segments[i]
        if abs((angle-heading+pi) % (2*pi)-pi) > heading_tolerance:
            return None
        length += segment_length * (1-projection) if i == nearest else segment_length
        if length >= lookahead - 1e-6:
            return heading
    return None


def straight_reference(sample, lookahead=0.45, heading_tolerance=0.05):
    return straight_reference_heading(sample, lookahead, heading_tolerance) is not None


def analyze_samples(samples, start, end, max_gap=0.3, omega_deadband=0.05):
    # Time weights need two samples inside the referee window. No extrapolation.
    samples = sorted((s for s in samples if start <= s['sim_t_s'] <= end),
                     key=lambda s: s['sim_t_s'])
    straight_s = arc_s = observed_s = omega_square = lateral_square = lateral_s = 0.
    flips = 0
    last_sign = None
    previous = None
    for sample in samples:
        speed, omega = sample.get('speed_mps'), sample.get('measured_omega_radps')
        heading = straight_reference_heading(sample)
        eligible = (speed is not None and omega is not None and speed >= 0.05 and
                    heading is not None)
        dt = sample['sim_t_s']-previous['sim_t_s'] if previous else 0.
        contiguous = previous is not None and 0 < dt <= max_gap
        previous_eligible = previous.get('_straight', False) if previous else False
        if previous_eligible and eligible:
            previous_eligible = (abs((heading-previous['_heading']+pi) % (2*pi)-pi) <= 0.05 and
                                 sample.get('behavior') == previous.get('behavior'))
        if contiguous:
            observed_s += dt
            if (speed is not None and omega is not None and
                    previous.get('speed_mps') is not None and
                    previous.get('measured_omega_radps') is not None and
                    min(speed, previous['speed_mps']) >= 0.05 and
                    min(abs(omega), abs(previous['measured_omega_radps'])) >= 0.1):
                arc_s += dt
        if not eligible or not contiguous or not previous_eligible:
            last_sign = None
        if eligible:
            sign = 1 if omega > omega_deadband else -1 if omega < -omega_deadband else 0
            if sign:
                if last_sign is not None and sign != last_sign:
                    flips += 1
                last_sign = sign
            if contiguous and previous_eligible:
                straight_s += dt
                omega_square += dt * (omega**2 + previous['measured_omega_radps']**2)/2
                lateral, prior_lateral = sample.get('lateral_global_m'), previous.get('lateral_global_m')
                if lateral is not None and prior_lateral is not None:
                    lateral_s += dt
                    lateral_square += dt * (lateral**2 + prior_lateral**2)/2
        previous = dict(sample, _straight=eligible, _heading=heading)
    return {
        'observed_interval_s': observed_s,
        'straight_moving_s': straight_s,
        'straight_omega_sign_changes': flips,
        'straight_sign_changes_per_minute': 60*flips/straight_s if straight_s else None,
        'straight_measured_omega_rms_radps': sqrt(omega_square/straight_s) if straight_s else None,
        'straight_lateral_rms_m': sqrt(lateral_square/lateral_s) if lateral_s else None,
        'straight_lateral_observed_s': lateral_s,
        'arc_motion_s': arc_s,
        'arc_fraction_of_observed_time': arc_s/observed_s if observed_s else None,
    }


def analyze(series):
    result = {'series': str(series), 'definition': {
        'straight_reference': 'at least 0.45 m ahead in observed global head; segment heading spread <=0.05 rad; moving speed >=0.05 m/s',
        'sign_change': 'measured omega exceeds +/-0.05 rad/s; deadband keeps last sign; reset at turn, stop, reference direction change >0.05 rad, behavior change or gap >0.3 s',
        'arc': 'both ends of observed interval have speed >=0.05 m/s and abs(measured omega)>=0.1 rad/s',
        'weighting': 'intervals with both endpoints inside referee window, no extrapolation; classification requires both endpoints',
        'limitations': 'reference straightness does not prove free space; trace has at most six global points and cached geometry; no full footprint clearance measurement',
    }, 'runs': []}
    for i, record in enumerate(json.loads((series/'index.json').read_text())):
        if 'robots' not in record or 'error' in record:
            continue
        for side, robot in zip(('first', 'second'), record['robots']):
            path = series/f'{i:02d}-trace-{side}.json'
            if not path.exists():
                continue
            row = {'seed': record['seed'], 'role': robot['role'], 'event': record['outcome']['event']}
            row.update(analyze_samples(json.loads(path.read_text())['time_series'],
                                      robot['window_start_sim_s'], robot['window_end_sim_s']))
            result['runs'].append(row)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('series', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.series), indent=2))
