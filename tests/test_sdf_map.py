import sys
from pathlib import Path
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
for package in ('jr_map', 'hsl_planning'):
    sys.path.insert(0, str(ROOT / 'src' / package))
from jr_map.sdf_geometry import collect_boxes, rasterize
from hsl_planning.core import Pose2, VoxelWorld, astar


def test_polygon_static_export_and_mission_clearance():
    cfg = yaml.safe_load((ROOT / 'config/match.yaml').read_text())
    sim = cfg['simulation']
    boxes = collect_boxes(str(ROOT / 'src/sim_kobuki/worlds' / (sim['world'] + '.world')), .25)
    grid, x, y = rasterize(boxes, .05, 1., sim['map_origin_world'])
    meta = yaml.safe_load((ROOT / 'config/maps/polygon_rosbag.yaml').read_text())
    raw = (ROOT / 'config/maps' / meta['image']).read_bytes().split(b'\n', 3)
    assert list(map(int, raw[1].split())) == [grid.shape[1], grid.shape[0]]
    assert meta['origin'] == [x, y, 0.0]
    pixels = np.frombuffer(raw[3], np.uint8).reshape(grid.shape)
    assert np.array_equal(pixels == 0, grid[::-1] == 100)
    iy, ix = np.where(grid == 100)
    points = [(x + (cx+.5)*.05, y + (cy+.5)*.05, .25) for cy,cx in zip(iy,ix)]
    fy, fx = np.where(grid == 0)
    world = VoxelWorld(resolution=.15, robot_radius=.23)
    free = {world.cell(x+(cx+.5)*.05,y+(cy+.5)*.05) for cy,cx in zip(fy,fx)}
    world.update(points, [], None, known_free=free, map_bounds=sim['arena_bounds'])
    first = Pose2(*cfg['robot']['start'])
    second = Pose2(*cfg['opponent']['start'])
    assert not world.blocked(first.x, first.y)
    assert not world.blocked(second.x, second.y)
    assert astar(world, first, second)
