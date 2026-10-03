"""A lost remembered target must not indefinitely hide a stationary robot."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/jr_perception'))
from jr_perception.segmentation import Detection
from jr_perception.tracker import Tracker,TrackerConfig


def detection(x,strong=True):return Detection(np.array([x,0.]),.03,strong)


def test_stationary_robot_reacquired_after_remembered_target_expires():
    t=Tracker(TrackerConfig(move_min_hits=3,move_threshold=.1))
    for time,x in [(0.,0.),(.1,.12),(.2,.24)]:t.step(time,[detection(x)])
    assert t.robot_position is not None
    for time in [2.,2.1,2.2]:selected=t.step(time,[detection(2.)])
    assert selected is not None and not selected.moved
    assert abs(selected.mean[0]-2.)<.001


def test_weak_updates_cannot_confirm_a_new_stationary_target():
    t=Tracker(TrackerConfig())
    assert t.step(0.,[detection(1.)]) is None
    assert t.step(.1,[detection(1.,False)]) is None
    assert t.step(.2,[detection(1.,False)]) is None
    for time in [.3,.4,.5]:selected=t.step(time,[detection(1.)])
    assert selected is not None


def test_reacquisition_requires_fresh_strong_series_not_only_lifetime_hits():
    t=Tracker(TrackerConfig())
    for time in [0.,.1,.2]:t.step(time,[detection(1.)])
    t.selected=None;t.robot_position=np.array([10.,0.]);t.robot_last_seen=-10.
    t.tracks[0].strong_streak=0
    assert t.step(.3,[detection(1.,False)]) is None


def test_new_moving_candidate_does_not_replace_fresh_stationary_target():
    t=Tracker(TrackerConfig(move_min_hits=3,move_threshold=.1))
    for time in [0.,.1,.2]:t.step(time,[detection(0.)])
    original=t.selected
    for time,x in [(.3,2.),(.4,2.12),(.5,2.24)]:t.step(time,[detection(0.),detection(x)])
    assert any(track.moved for track in t.tracks if track is not original)
    assert t.selected is original


def test_weak_only_support_does_not_keep_selected_target_forever():
    t=Tracker(TrackerConfig())
    for time in [0.,.1,.2]:t.step(time,[detection(0.)])
    assert t.selected is not None
    for time in np.arange(.3,2.,.1):t.step(float(time),[detection(0.,False)])
    assert t.selected is None
