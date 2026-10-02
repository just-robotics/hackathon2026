"""Livox quality and measured near-return shadows of the robot's four rods.

Angles and range apply in the LiDAR frame, independent of map/robot pose.
Reserved tag bits are ignored; medium-confidence returns are retained.
"""
import numpy as np

DEFAULT_PARAMETERS = {
    'self_occlusion_centers_deg': [15.0, 165.0, -165.0, -15.0],
    'self_occlusion_half_angle_deg': 5.0,
    'self_occlusion_max_range': 0.45,
    'filter_low_confidence': True,
}


def point_mask(points, tags=None, *, self_occlusion_centers_deg=(),
               self_occlusion_half_angle_deg=5., self_occlusion_max_range=.45,
               filter_low_confidence=True):
    points = np.asarray(points).reshape(-1, 3)
    finite = np.isfinite(points).all(axis=1)
    bad_quality = np.zeros(len(points), dtype=bool)
    if tags is not None and filter_low_confidence:
        tags = np.asarray(tags, dtype=np.uint8)
        # SDK2 Mid-360 tag: three 2-bit confidence groups; 2=low, 3=reserved.
        bad_quality = ((tags & 3) >= 2) | (((tags >> 2) & 3) >= 2) | (((tags >> 4) & 3) >= 2)
    occluded = np.zeros(len(points), dtype=bool)
    azimuth = np.degrees(np.arctan2(points[:,1], points[:,0]))
    close = np.linalg.norm(points, axis=1) <= self_occlusion_max_range
    for center in self_occlusion_centers_deg:
        delta = (azimuth-center+180) % 360-180
        occluded |= close & (np.abs(delta) <= self_occlusion_half_angle_deg)
    keep = finite & ~bad_quality & ~occluded
    return keep, dict(input_points=len(points), kept_points=int(keep.sum()),
        nonfinite_points=int((~finite).sum()), low_confidence_points=int((finite & bad_quality).sum()),
        rod_shadow_points=int((finite & ~bad_quality & occluded).sum()))
