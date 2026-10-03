"""Regression for sparse real wall returns being accepted as a robot."""
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/jr_perception'))
from jr_perception.segmentation import RobotModel,inspect_cluster


def real_model(**kwargs):
    return RobotModel(max_height=.50,min_top=.25,min_points=10,min_extent=.22,**kwargs)


def test_recorded_six_point_wall_fragment_is_not_a_measurement():
    fixture=json.loads((Path(__file__).parent/'fixtures/detector_wall_fragment.json').read_text())
    points=np.array(fixture['points']);observer=np.array(fixture['observer'])[:2]
    # Verify that the fixture reproduces the original failure.
    old,_=inspect_cluster(points,observer,real_model(min_rim_points=4,plane_ratio=0.))
    assert old is not None and old.strong
    detection,reason=inspect_cluster(points,observer,real_model())
    assert detection is None and ('мало точек обода' in reason or 'плоскость' in reason)


def test_dense_visible_body_arc_remains_a_strong_measurement():
    angles=np.linspace(np.pi*.65,np.pi*1.35,24)
    rim=np.column_stack((1+.178*np.cos(angles),.178*np.sin(angles),np.full(24,.08)))
    upper=rim.copy();upper[:,2]=.35
    detection,reason=inspect_cluster(np.vstack((rim,upper)),np.array([0.,0.]),real_model())
    assert detection is not None and detection.strong,reason
    assert np.linalg.norm(detection.center-np.array([1.,0.]))<.001


def test_no_rim_points_cannot_extend_track_through_weak_fallback():
    angles=np.linspace(np.pi*.65,np.pi*1.35,24)
    points=np.column_stack((1+.178*np.cos(angles),.178*np.sin(angles),np.full(24,.35)))
    detection,_=inspect_cluster(points,np.array([0.,0.]),real_model())
    assert detection is None
