"""Compare a fitted circular rim with straight edges of a rotated rectangle."""
import numpy as np


def rectangle_edge_mse(xy):
    """Rotation-invariant edge residual, with robust 5/95 percentile bounds.

    No box dimensions or scene coordinates are assumed. A circle should fit
    its rim better than a rectangle; an L-shaped box corner has straight edges.
    """
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) < 4:
        return float('inf')
    angles = np.linspace(0., np.pi/2, 46)
    axes = np.stack((np.cos(angles), np.sin(angles)), axis=1)
    x = xy @ axes.T
    y = xy @ np.stack((-np.sin(angles), np.cos(angles)), axis=1).T
    lx, hx = np.percentile(x, [5, 95], axis=0)
    ly, hy = np.percentile(y, [5, 95], axis=0)
    residual = np.minimum.reduce((abs(x-lx), abs(x-hx), abs(y-ly), abs(y-hy)))
    return float(np.min(np.mean(residual**2, axis=0)))
