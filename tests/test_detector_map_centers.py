"""Original occupancy coordinates must gate centers without inflating free space."""
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/jr_perception'))
from jr_perception.map_background import OccupiedCenters


def checker():
    grid=NS(info=NS(height=2,width=3,resolution=.1,origin=NS(position=NS(x=-.35,y=-.50))),data=[100,0,-1,0,50,49])
    return OccupiedCenters(grid)


def test_shifted_map_uses_original_occupied_cell_and_threshold():
    m=checker()
    assert m.contains([-.30,-.45])
    assert m.contains([-.20,-.35])
    assert not m.contains([-.10,-.35])


def test_unknown_outside_and_neighboring_free_centers_are_not_blocked():
    m=checker()
    assert not m.contains([-.10,-.45])
    assert not m.contains([-1.,-1.])
    assert not m.contains([-.249,-.45])


def test_nonfinite_center_is_rejected():
    assert checker().contains([np.nan,0.])


def wall_checker():
    data=np.zeros((10,10),dtype=int);data[3:7,5]=100
    grid=NS(info=NS(height=10,width=10,resolution=.1,origin=NS(position=NS(x=0.,y=0.))),data=data.ravel().tolist())
    return OccupiedCenters(grid)


def test_visible_surface_points_can_pass_while_center_ray_is_blocked():
    m=wall_checker();observer=np.array([.2,.5])
    assert m.visible_fraction(observer,np.array([[.9,.5]]))==0.
    assert m.visible_fraction(observer,np.array([[.9,.5],[.9,.95]]))==.5


def test_completely_hidden_surface_is_rejected():
    m=wall_checker();observer=np.array([.2,.5])
    assert m.visible_fraction(observer,np.array([[.9,.45],[.9,.55]]))==0.
    assert not m.body_visible(observer,[.9,.5],.1,.5,.08)


def test_very_distant_prediction_does_not_skip_wall_or_allocate_unbounded_ray():
    assert wall_checker().visible_fraction(np.array([.2,.5]),np.array([[1e8,.5]]))==0.
