"""Convert occupied static map cells into the unchanged branch detector NPZ format."""
import numpy as np
from jr_perception.background import cell_keys


def grid_background(grid):
    info = grid.info
    q = info.origin.orientation
    if abs(q.x)+abs(q.y)+abs(q.z) > 1e-6:
        raise ValueError('Detector background requires an axis-aligned map')
    occupied = np.asarray(grid.data).reshape(info.height, info.width) >= 50
    rows, cols = np.nonzero(occupied)
    if not len(rows):
        raise ValueError('Static map contains no occupied cells for detector background')
    lower = np.column_stack((cols, rows))*info.resolution + [info.origin.position.x,info.origin.position.y]
    # Cover the whole source cell even if the NPZ lattice has a different origin.
    keys = np.unique(np.concatenate([cell_keys(lower + np.array(offset)*info.resolution, info.resolution)
        for offset in ((1e-6,1e-6),(1-1e-6,1e-6),(1e-6,1-1e-6),(1-1e-6,1-1e-6))]))
    return dict(keys=keys, tops=np.full(len(keys),.70), cell=info.resolution,
                plane=np.zeros(3), frame=grid.header.frame_id)
