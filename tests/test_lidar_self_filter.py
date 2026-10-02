import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/hsl_real'))
from hsl_real.lidar_filter_core import DEFAULT_PARAMETERS, point_mask


def test_four_near_rod_shadows_are_removed_but_far_obstacles_and_other_angles_stay():
    angles = np.radians([15,165,-165,-15,90,15])
    ranges = np.array([.28,.28,.28,.28,.28,1.])
    points = np.column_stack((ranges*np.cos(angles), ranges*np.sin(angles), np.full(6,.1)))
    mask, diag = point_mask(points, **DEFAULT_PARAMETERS)
    assert mask.tolist() == [False,False,False,False,True,True]
    assert diag['rod_shadow_points']==4


def test_livox_confidence_groups_preserve_medium_and_reserved_upper_bits():
    points = np.tile([1.,0.,0.],(9,1))
    tags = [0,1,4,16,64,2,8,32,128]
    mask, diag = point_mask(points, tags)
    assert mask.tolist() == [True,True,True,True,True,False,False,False,True]
    assert diag['low_confidence_points']==3


def test_disabled_masks_and_missing_tags_keep_nearby_real_obstacles():
    points = [[.28,.075,.1],[.2,0,.1],[float('nan'),0,0]]
    mask, diag = point_mask(points)
    assert mask.tolist()==[True,True,False]
    assert diag['nonfinite_points']==1
