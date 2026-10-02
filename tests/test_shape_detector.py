"""Robot/box discrimination and tracking regressions for the imported algorithm."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/hsl_perception'))
from hsl_perception.core import Detector, StaticBackground
from hsl_perception.segmentation import RobotModel, inspect_cluster, Detection
from hsl_perception.tracker import Tracker, TrackerConfig


def robot_cloud(center=(1., 1.)):
    rng = np.random.default_rng(21)
    angle = np.linspace(np.pi*.60, np.pi*1.40, 45)
    arc = np.column_stack((np.cos(angle),np.sin(angle)))*.178+center
    return np.concatenate([np.column_stack((arc+rng.normal(0,.002,arc.shape),np.full(len(arc),z)))
                           for z in (.045,.08,.105)])


def test_round_robot_is_detected_while_flat_box_faces_are_rejected():
    robot = robot_cloud()
    detection,_ = inspect_cluster(robot,np.array([0,1]),RobotModel())
    assert detection is not None and detection.strong
    assert np.linalg.norm(detection.center-[1,1]) < .02
    for length,height in ((.15,.4),(.4,.2),(.6,.2)):
        box = np.array([[1.,y,z] for y in np.linspace(1-length/2,1+length/2,40)
                        for z in (.04,.08,.105,height)])
        assert inspect_cluster(box,np.array([0,1]),RobotModel())[0] is None


def test_stationary_robot_can_open_track_and_missing_clouds_do_not_refresh_it():
    detector = Detector()
    static = StaticBackground(.05,[-1,-1],80,80,[0]*6400)
    for i in range(3):
        track,diagnostic = detector.step(robot_cloud(),[0,1,.374],static,i*.1)
    assert track is not None and track.confirmed
    last=track.last_update
    track,_ = detector.step([], [0,1,.374],static,.5)
    assert track is not None and track.last_update==last
    track,_ = detector.step([], [0,1,.374],static,2.)
    assert track is None


def test_static_background_and_time_reset():
    grid=[0]*400;grid[10*20+10]=100
    static=StaticBackground(.1,[0,0],20,20,grid)
    assert not static.foreground(np.array([[1.05,1.05,.10]]))[0]
    assert static.foreground(np.array([[.5,.5,.10]]))[0]
    tracker=Tracker(TrackerConfig())
    for i in range(3):tracker.step(10+i*.1,[Detection(np.array([1.,1.]),.03,True)])
    assert tracker.selected is not None
    assert tracker.step(0,[]) is None and not tracker.tracks


def test_vertical_box_corner_is_not_a_robot_but_partial_robot_remains_candidate():
    # The original circle fit accepts two visible narrow-box faces. The height
    # gap distinguishes their solid wall from Kobuki plates on this simulator.
    rng = np.random.default_rng(5)
    box = np.array([[.925, 1+y, z] for y in np.linspace(-.075,.075,25)
                    for z in np.linspace(.04,.4,12)] +
                   [[1+x, .925, z] for x in np.linspace(-.075,.075,25)
                    for z in np.linspace(.04,.4,12)])
    box += rng.normal(0,.01,box.shape)
    model = RobotModel(max_height=.46, max_gap_share=.12, line_ratio=.35)
    assert inspect_cluster(box,np.array([0,0]),model)[0] is None
    detection,_ = inspect_cluster(robot_cloud(),np.array([0,1]),model)
    assert detection is not None and detection.strong


def test_short_round_fragment_cannot_initialize_track_but_stationary_semicircle_can():
    static=StaticBackground(.05,[-1,-1],80,80,[0]*6400)
    detector=Detector(RobotModel(max_gap_share=.12,line_ratio=.35),strong_arc_min_span_deg=90.)
    short=robot_cloud()[10:20]
    for i in range(5):
        track,_=detector.step(short,[0,1,.374],static,i*.1)
        assert track is None
    for i in range(5,8):track,_=detector.step(robot_cloud(),[0,1,.374],static,i*.1)
    assert track is not None and track.confirmed


def test_robot_extracted_from_oversized_cluster_can_only_continue_known_track():
    static=StaticBackground(.05,[-1,-1],80,80,[0]*6400)
    model=RobotModel(max_gap_share=.12,line_ratio=.35)
    detector=Detector(model,strong_arc_min_span_deg=90.,allow_merged_strong=False,strong_min_inlier_fraction=.95)
    wall=np.array([[.80,y,.08] for y in np.linspace(.7,1.8,50)])
    merged=np.vstack([robot_cloud(),wall])
    for i in range(4):
        track,diagnostic=detector.step(merged,[0,1,.374],static,i*.1)
        assert diagnostic['strong_candidates']==0 and track is None
    for i in range(4,7):track,_=detector.step(robot_cloud(),[0,1,.374],static,i*.1)
    assert track is not None and track.confirmed


def test_unknown_background_and_impossible_fitted_center_are_rejected():
    grid=np.zeros((40,40),dtype=int);grid[5:10,5:10]=-1
    static=StaticBackground(.1,[0,0],40,40,grid.ravel())
    assert not static.foreground(np.array([[.75,.75,.2]]))[0]
    assert not static.free_center([.75,.75])
    assert static.free_center([1.,1.])
    # Returns outside the wall margin can still extrapolate a center into it.
    grid[10,10]=100
    static=StaticBackground(.1,[0,0],40,40,grid.ravel(),margin=0)
    detector=Detector()
    points=robot_cloud((1.05,1.05))
    for i in range(4):
        track,diag=detector.step(points,[0,1,.31],static,i*.1)
        assert track is None


def test_real_birth_width_does_not_disable_partial_updates():
    detector=Detector(strong_min_extent=.25)
    static=StaticBackground(.05,[-1,-1],80,80,[0]*6400)
    partial=robot_cloud()[:15]
    for i in range(4):
        track,diag=detector.step(partial,[0,1,.31],static,i*.1)
        assert track is None and diag['strong_candidates']==0
    for i in range(4,7):track,_=detector.step(robot_cloud(),[0,1,.31],static,i*.1)
    assert track is not None
    track,_=detector.step(partial,[0,1,.31],static,.7)
    assert track is not None and track.last_update==.7


def test_rectangle_corner_cannot_initialize_robot_track():
    rng = np.random.default_rng(17)
    length = .28
    xy = np.concatenate([np.column_stack((np.linspace(0, length, 35), np.zeros(35))),
                         np.column_stack((np.zeros(35), np.linspace(0, length, 35)))]) + [1, 1]
    corner = np.concatenate([np.column_stack((xy + rng.normal(0, .005, xy.shape),
                             np.full(len(xy), z))) for z in [.085, .10, .115, .14, .18, .20]])
    static = StaticBackground(.05, [0, 0], 80, 80, [0]*6400)
    baseline = Detector(RobotModel(max_gap_share=.12, line_ratio=.35))
    fixed = Detector(RobotModel(max_gap_share=.12, line_ratio=.35), strong_rectangle_ratio=.70)
    for i in range(4):
        old, _ = baseline.step(corner, [0, 0, .374], static, i*.1)
        new, diag = fixed.step(corner, [0, 0, .374], static, i*.1)
    assert old is not None  # reproduce the geometric ambiguity first
    assert new is None and diag['strong_candidates'] == 0
    assert diag['weak_reasons']['rectangle fits better: weak only'] > 0


def test_rectangle_check_preserves_curved_robot_rim():
    fixed = Detector(strong_rectangle_ratio=.70)
    static = StaticBackground(.05, [-1, -1], 80, 80, [0]*6400)
    for i in range(4):
        track, diag = fixed.step(robot_cloud(), [0, 1, .31], static, i*.1)
    assert track is not None and diag['strong_candidates'] > 0
