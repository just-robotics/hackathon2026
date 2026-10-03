#!/usr/bin/env python3
"""Replay every cached real bag sequentially with bounded JSONL output.

The standalone C++ runner still processes every original cloud in bag order.
Only large per-candidate samples are omitted from the saved evaluation rows.
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path


KEEP = ('bag', 'stamp_s', 'stamp_ns', 'status', 'has_measurement',
        'measurement_xy', 'track_id', 'velocity_xy', 'velocity_valid',
        'has_prediction', 'prediction_xy', 'prediction_age_s', 'timing_ms',
        'map_alignment_share')


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--map-cache', type=Path, required=True)
    parser.add_argument('--local-cache', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'binary_sha256': sha256(args.binary), 'bags': {}}
    sources = [(p, False) for p in sorted(args.map_cache.glob('20*')) if (p / 'frames.tsv').is_file()]
    sources += [(p, True) for p in sorted(args.local_cache.glob('20*')) if (p / 'frames.tsv').is_file()]
    if not sources:
        raise ValueError('no cached real bags found')
    for cache, mapless in sources:
        command = [str(args.binary.resolve()), str(cache.resolve()), '/dev/stdout', 'filtered']
        if mapless:
            command.append('--mapless')
        process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True)
        output = args.output / (cache.name + '.jsonl')
        scans = measured = 0
        with output.open('w') as stream:
            for line in process.stdout:
                row = json.loads(line)
                stream.write(json.dumps({key: row.get(key) for key in KEEP},
                                        separators=(',', ':')) + '\n')
                scans += 1
                measured += bool(row.get('has_measurement'))
        error = process.stderr.read()
        if process.wait() != 0:
            raise RuntimeError(f'{cache}: C++ replay failed: {error}')
        report['bags'][cache.name] = {
            'cache_manifest_sha256': sha256(cache / 'manifest.json'),
            'output_sha256': sha256(output), 'scans': scans,
            'measured': measured, 'mapless_local_odom': mapless,
            'runner_summary': error.strip(),
        }
        print(cache.name, 'scans', scans, 'measured', measured, flush=True)
    report['total_scans'] = sum(bag['scans'] for bag in report['bags'].values())
    (args.output / 'manifest.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
