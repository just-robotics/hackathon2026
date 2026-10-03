"""Distinguish a real-radius body arc from two visible flat box faces."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/jr_perception'))
from jr_perception.segmentation import RobotModel,plane_test


def test_body_arc_survives_rotated_views():
    model=RobotModel(plane_ratio=.8)
    for yaw in (0.,.4,1.2):
        angles=np.linspace(np.pi*.55,np.pi*1.45,120)+yaw
        center=np.array([1.,.4])
        xy=center+model.radius*np.column_stack((np.cos(angles),np.sin(angles)))
        planar,faces,circle=plane_test(xy,center,model)
        assert not planar


def test_two_flat_faces_are_rejected_in_different_orientations():
    model=RobotModel(plane_ratio=.8)
    side=np.linspace(-.075,.075,80)
    xy=np.vstack((np.column_stack((side,np.full(80,-.075))),
                  np.column_stack((np.full(80,-.075),side))))
    for yaw in (0.,.4,1.2):
        rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        center=np.array([1.,.4]);points=xy@rotation.T+center
        planar,faces,circle=plane_test(points,center,model)
        assert planar
        assert faces < model.plane_ratio*circle


def test_short_partial_body_arc_is_not_vetoed_as_a_plane():
    model=RobotModel(plane_ratio=100.,plane_min_improvement=0.)
    angles=np.linspace(np.pi*.9,np.pi*1.1,30)
    center=np.array([1.,.4]);xy=center+model.radius*np.column_stack((np.cos(angles),np.sin(angles)))
    planar,_,_=plane_test(xy,center,model)
    assert not planar


def test_noisy_body_arc_survives_small_model_fit_differences():
    model=RobotModel(plane_ratio=.8,plane_min_improvement=.01)
    rng=np.random.default_rng(67)
    angles=np.linspace(np.pi*.55,np.pi*1.45,150)
    center=np.array([1.,.4])
    xy=center+model.radius*np.column_stack((np.cos(angles),np.sin(angles)))
    xy+=rng.normal(0,.008,xy.shape)
    planar,_,_=plane_test(xy,center,model)
    assert not planar
