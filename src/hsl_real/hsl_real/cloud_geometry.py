"""Full measured XYZ transport; sampling must not destroy object geometry."""
import numpy as np


def map_xyz(points, translation, quaternion, base_xy, self_radius=.25):
    points=np.asarray(points,dtype=float).reshape(-1,3)
    points=points[np.isfinite(points).all(axis=1)]
    q=np.asarray(quaternion,dtype=float)
    twice_cross=2*np.cross(q[:3],points)
    mapped=points+q[3]*twice_cross+np.cross(q[:3],twice_cross)+translation
    return mapped[np.linalg.norm(mapped[:,:2]-base_xy,axis=1)>=self_radius]
