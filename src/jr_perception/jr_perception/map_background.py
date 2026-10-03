"""Convert occupied static map cells into the unchanged branch detector NPZ format."""
import numpy as np


class OccupiedCenters:
    """Check centers against original map cells, without inflation or resampling."""

    def __init__(self, grid):
        info = grid.info
        self.cells = np.asarray(grid.data).reshape(info.height, info.width) >= 50
        self.resolution = info.resolution
        if self.resolution <= 0:
            raise ValueError('Map resolution must be positive')
        self.origin = np.array([info.origin.position.x, info.origin.position.y])

    def contains(self, xy):
        xy = np.asarray(xy)
        if not np.isfinite(xy).all():
            return True
        col, row = np.floor((xy - self.origin) / self.resolution).astype(int)
        if row < 0 or col < 0 or row >= self.cells.shape[0] or col >= self.cells.shape[1]:
            return False
        return bool(self.cells[row, col])

    def visible_fraction(self, observer, points, endpoint_margin=.08, observer_margin=.20):
        """Visible supporting points; a partially exposed body can pass at a corner."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if not len(points) or not np.isfinite(points).all():
            return 0.0
        # Bound work for dense clouds without changing the rim count requirement.
        if len(points) > 64:
            angles = np.arctan2(points[:, 1]-observer[1], points[:, 0]-observer[0])
            points = points[np.argsort(angles)[np.linspace(0, len(points)-1, 64).astype(int)]]
        delta = points - observer
        distances = np.linalg.norm(delta, axis=1)
        if not np.isfinite(distances).all():
            return 0.0
        # Clip to the map before sampling: a far predicted center must not
        # allocate a ray array proportional to its distance outside the map.
        lower = np.zeros(len(points))
        upper = np.ones(len(points))
        crosses = np.ones(len(points), dtype=bool)
        bounds_max = self.origin + self.resolution*np.array([self.cells.shape[1], self.cells.shape[0]])
        for axis in (0, 1):
            parallel = np.abs(delta[:, axis]) < 1e-12
            crosses &= ~parallel | ((observer[axis] >= self.origin[axis]) & (observer[axis] <= bounds_max[axis]))
            near = np.divide(self.origin[axis]-observer[axis], delta[:, axis],
                             out=np.full(len(points), -np.inf), where=~parallel)
            far = np.divide(bounds_max[axis]-observer[axis], delta[:, axis],
                            out=np.full(len(points), np.inf), where=~parallel)
            lower = np.maximum(lower, np.minimum(near, far))
            upper = np.minimum(upper, np.maximum(near, far))
        crosses &= upper >= lower
        lengths = distances*np.maximum(upper-lower, 0)
        lengths[~crosses] = 0
        count = max(2, int(np.ceil(lengths.max() / (self.resolution*.5))) + 1)
        fractions = lower[:, None] + np.maximum(upper-lower, 0)[:, None]*np.linspace(0, 1, count)
        rays = np.asarray(observer) + delta[:, None, :] * fractions[:, :, None]
        indices = np.floor((rays-self.origin)/self.resolution).astype(int)
        cols, rows = indices[:, :, 0], indices[:, :, 1]
        valid = ((cols >= 0) & (rows >= 0) & (cols < self.cells.shape[1]) & (rows < self.cells.shape[0]) &
                 crosses[:, None] & (fractions*distances[:, None] > observer_margin) &
                 ((1-fractions)*distances[:, None] > endpoint_margin))
        hits = np.zeros(valid.shape, dtype=bool)
        hits[valid] = self.cells[rows[valid], cols[valid]]
        return float(np.mean(~hits.any(axis=1)))

    def body_visible(self, observer, center, radius, minimum_fraction, endpoint_margin):
        """Check potential front surface for a coasted center, not just its center ray."""
        angles = np.linspace(0, 2*np.pi, 32, endpoint=False)
        normals = np.column_stack((np.cos(angles), np.sin(angles)))
        center = np.asarray(center)
        front = normals @ (np.asarray(observer)-center) >= 0
        points = center + radius*normals[front]
        return self.visible_fraction(observer, points, endpoint_margin) >= minimum_fraction

