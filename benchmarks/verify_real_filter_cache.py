#!/usr/bin/env python3
"""Compare offline Livox filter output byte for byte with paired real clouds."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import real_detector_cache as base
from replay_cached_baseline import Zstd, cloud_from_cache


REPOSITORY = Path(__file__).resolve().parents[1]


def verify(cache):
    sys.path.insert(0, str(REPOSITORY / 'src/hsl_real'))
    from hsl_real.lidar_filter import filter_cloud
    import yaml

    manifest = json.loads((cache / 'manifest.json').read_text())
    if manifest['schema'] != 'real-detector-cache-v1':
        raise ValueError('requires recorded raw and filtered clouds in map-anchored cache')
    parameters = yaml.safe_load((REPOSITORY / 'config/lidar_filter.yaml').read_text())[
        'real_lidar_filter']['ros__parameters']
    artifacts = {item['name']: item['sha256'] for item in manifest['artifacts']}
    for name in ('frames.tsv', 'pairs.tsv', 'clouds.zstbin'):
        if base.sha256(cache / name) != artifacts[name]:
            raise ValueError(f'{name} does not match manifest')
    with (cache / 'frames.tsv').open(newline='') as stream:
        frames = {int(row['record_index']): row
                  for row in csv.DictReader(stream, delimiter='\t')}
    zstd = Zstd()
    mismatches = []
    count = 0
    with (cache / 'clouds.zstbin').open('rb') as binary, \
         (cache / 'pairs.tsv').open(newline='') as pair_file:
        for pair in csv.DictReader(pair_file, delimiter='\t'):
            raw_row = frames[int(pair['raw_record_index'])]
            filtered_row = frames[int(pair['filtered_record_index'])]
            def load(row):
                binary.seek(int(row['offset']))
                encoded = binary.read(int(row['compressed_size']))
                payload = zstd.decompress(encoded, int(row['uncompressed_size']))
                return cloud_from_cache(row, manifest['layouts'][int(row['layout_id'])], payload)
            raw = load(raw_row)
            recorded = load(filtered_row)
            reconstructed, _ = filter_cloud(raw, parameters)
            count += 1
            if reconstructed.width != recorded.width or \
                    reconstructed.row_step != recorded.row_step or \
                    reconstructed.fields != recorded.fields or \
                    bytes(reconstructed.data) != bytes(recorded.data):
                mismatches.append({'stamp_ns': int(pair['header_ns']),
                                   'raw_points': raw.width,
                                   'recorded_points': recorded.width,
                                   'reconstructed_points': reconstructed.width,
                                   'recorded_sha256': hashlib.sha256(bytes(recorded.data)).hexdigest(),
                                   'reconstructed_sha256': hashlib.sha256(bytes(reconstructed.data)).hexdigest()})
    report = {
        'cache': str(cache), 'cache_signature': manifest['signature'],
        'pairs_compared': count, 'byte_identical': count - len(mismatches),
        'mismatch_count': len(mismatches), 'first_mismatches': mismatches[:5],
        'oracle_sha256': base.sha256(REPOSITORY / 'src/hsl_real/hsl_real/lidar_filter.py'),
        'core_sha256': base.sha256(REPOSITORY / 'src/hsl_real/hsl_real/lidar_filter_core.py'),
        'production_cpp_sha256': base.sha256(REPOSITORY / 'src/hsl_lidar_filter/src/lidar_filter.cpp'),
        'parameters_sha256': base.sha256(REPOSITORY / 'config/lidar_filter.yaml'),
    }
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cache', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = verify(args.cache)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report), flush=True)
    if report['mismatch_count']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
