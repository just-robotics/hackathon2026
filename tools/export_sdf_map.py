#!/usr/bin/env python3
"""Export the simulator's SDF occupancy grid as a Nav2 YAML/PGM pair."""
import argparse
from pathlib import Path
import sys
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/jr_map'))
from jr_map.sdf_geometry import collect_boxes, rasterize


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/match.yaml')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--padding', type=float, default=1.0)
    parser.add_argument('--z-slice', type=float, default=0.25)
    parser.add_argument('--output', type=Path, default=ROOT / 'config/maps/polygon_rosbag.yaml')
    args = parser.parse_args()
    if args.resolution <= 0 or args.padding < 0:
        parser.error('resolution must be positive and padding nonnegative')
    sim = yaml.safe_load(args.config.read_text())['simulation']
    world = ROOT / 'src/sim_kobuki/worlds' / (sim['world'] + '.world')
    boxes = collect_boxes(str(world), args.z_slice)
    if not boxes:
        parser.error('no box collisions at the requested height')
    grid, x, y = rasterize(boxes, args.resolution, args.padding, sim['map_origin_world'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    pgm = args.output.with_suffix('.pgm')
    pixels = np.where(grid[::-1] == 100, 0, 254).astype(np.uint8)
    pgm.write_bytes(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode() + pixels.tobytes())
    args.output.write_text(yaml.safe_dump(dict(image=pgm.name, mode='trinary',
        resolution=args.resolution, origin=[float(x), float(y), 0.0], negate=0,
        occupied_thresh=0.65, free_thresh=0.196), sort_keys=False))
    print(f'{args.output}: {grid.shape[1]}x{grid.shape[0]}, {len(boxes)} boxes, '
          f'{np.count_nonzero(grid == 100)} occupied cells, origin=[{x}, {y}]')


if __name__ == '__main__':
    main()
