"""Unknown-box policy must not discard walls, a broad box, or Kobuki bodies."""
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/hsl_planning'),str(ROOT/'src/hsl_perception')]
from hsl_planning.obstacle_filter import SmallBoxFilter,classify_cluster


def faces(length,width,height,yaw=0,center=(1,1)):
    xy=np.concatenate([np.column_stack((np.linspace(-length/2,length/2,25),np.full(25,-width/2))),
                       np.column_stack((np.full(25,-length/2),np.linspace(-width/2,width/2,25)))])
    rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    xy=xy@rot.T+center
    return np.concatenate([np.column_stack((xy,np.full(len(xy),z))) for z in np.linspace(.08,height,12)])


@pytest.mark.parametrize('yaw',[0,.25,np.pi/4,np.pi/2,2.1])
def test_narrow_tall_box_ignored_and_broad_low_box_retained(yaw):
    small=faces(.15,.15,.40,yaw);large=faces(.4,.6,.2,yaw,center=(2,2))
    assert classify_cluster(small)[0]
    assert not classify_cluster(large)[0]
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    kept,diag=f.filter(np.concatenate([small,large]))
    assert diag['ignored_points']==len(small)
    assert np.allclose(kept,large)


def test_known_walls_preserved_even_if_return_fragment_looks_like_box():
    points=faces(.15,.15,.4)
    grid=np.zeros((80,80),dtype=int);grid[17:23,17:23]=100
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,grid.ravel())
    kept,diag=f.filter(points)
    assert len(kept)==len(points) and diag['ignored_points']==0


def test_partial_merged_cluster_and_round_robot_preserved():
    assert not classify_cluster(faces(.15,.15,.2))[0]
    assert not classify_cluster(faces(.35,.35,.4))[0]
    a=np.linspace(np.pi/2,3*np.pi/2,45)
    xy=np.column_stack((np.cos(a),np.sin(a)))*.178+[1,1]
    robot=np.concatenate([np.column_stack((xy,np.full(len(xy),z))) for z in (.045,.08,.105,.14,.20,.41)])
    assert not classify_cluster(robot)[0]
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    kept,diag=f.filter(robot)
    assert diag['ignored_points']==0 and len(kept)==len(robot)


def test_no_map_is_fail_closed():
    points=faces(.15,.15,.4)
    kept,diag=SmallBoxFilter().filter(points)
    assert len(kept)==len(points) and not diag['ready']


def test_reclassification_erases_previous_occupancy_before_new_hits():
    from hsl_planning.obstacle_memory import ObstacleMemory
    from hsl_planning.core import Pose2
    memory=ObstacleMemory()
    box=faces(.15,.15,.4);large=faces(.4,.6,.2,center=(2,2))
    memory.update(np.concatenate([box,large]),Pose2(0,0),1.)
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    kept,_=f.filter(np.concatenate([box,large]))
    memory.forget(f.ignored);memory.update(kept,Pose2(0,0),1.1)
    points=np.asarray(memory.points(1.1))
    assert not np.any(np.linalg.norm(points[:,:2]-[1,1],axis=1)<.25)
    assert np.any(np.linalg.norm(points[:,:2]-[2,2],axis=1)<.5)


def test_known_box_partial_view_can_continue_but_does_not_initialize():
    full=faces(.15,.15,.4);partial=full[full[:,2]>=.29]
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    kept,_=f.filter(partial,0.)
    assert len(kept)==len(partial)
    f.filter(full,.1);f.filter(full,.2);f.filter(full,.3);kept,diag=f.filter(partial,.4)
    assert len(kept)==0
    kept,_=f.filter(partial,9.)
    assert len(kept)==len(partial)


def test_noisy_box_faces_are_ignored_without_dropping_round_body():
    rng=np.random.default_rng(3)
    noisy=faces(.15,.15,.4,np.pi/4)
    noisy[:,:2]+=rng.normal(0,.012,noisy[:,:2].shape)
    assert classify_cluster(noisy)[0]


def test_sparse_contact_returns_maintain_box_but_cannot_initialize():
    points=faces(.15,.15,.4)
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    sparse=points[[10]]
    assert len(f.filter(sparse,0)[0])==1
    f.filter(points,.1);f.filter(points,.2);f.filter(points,.3)
    assert len(f.filter(sparse,.4)[0])==0
    assert len(f.filter(sparse,9)[0])==1


def test_current_tracked_robot_is_preserved_even_if_shape_or_cache_says_box():
    cloud=faces(.15,.15,.4)
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    f.filter(cloud,0.);f.filter(cloud,.1);f.filter(cloud,.2)
    kept,diag=f.filter(cloud,.3,protected_centers=[[1,1]])
    assert len(kept)==len(cloud) and diag['ignored_points']==0
    assert diag['clusters'][0]['reason']=='tracked_robot'
    # The conflicting box identity was invalidated. An unrelated robot does
    # not prevent a fresh three-view confirmation of this actual box.
    assert len(f.filter(cloud,.4,protected_centers=[[2,2]])[0])==len(cloud)
    f.filter(cloud,.5,protected_centers=[[2,2]])
    assert len(f.filter(cloud,.6,protected_centers=[[2,2]])[0])==0


@pytest.mark.parametrize('yaw',[0,.4,np.pi/4,1.2])
def test_long_flat_fragment_cannot_become_small_square_by_rotating_fit(yaw):
    x=np.linspace(-.115,.115,60)
    xy=np.column_stack((x,np.zeros_like(x)))
    rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    xy=xy@rotation.T+[1,1]
    points=np.concatenate([np.column_stack((xy,np.full(len(xy),z))) for z in np.linspace(.08,.40,8)])
    assert not classify_cluster(points)[0]


def test_blurred_faces_remain_obstacle_until_shape_is_confident():
    points=faces(.15,.15,.4,np.pi/4)
    points[:,:2]+=np.random.default_rng(3).normal(0,.018,points[:,:2].shape)
    assert not classify_cluster(points)[0]


def test_transient_or_repeated_same_scan_does_not_establish_box_identity():
    cloud=faces(.15,.15,.4)
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    assert len(f.filter(cloud,0.)[0])==len(cloud)
    assert len(f.filter(cloud,0.)[0])==len(cloud)
    assert len(f.filter(cloud,.1)[0])==len(cloud)
    assert len(f.filter(cloud,.2)[0])==0
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    f.filter(cloud,0.)
    assert len(f.filter(cloud,1.)[0])==len(cloud)


def test_robot_observation_invalidates_cached_box_before_track_expires():
    cloud=faces(.15,.15,.4)
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    for t in [0.,.1,.2]:f.filter(cloud,t)
    assert f.boxes
    f.filter(cloud,.3,protected_centers=[[1,1]])
    assert not f.boxes and not f.pending
    # A weak fragment after loss cannot resurrect the old box decision.
    sparse=cloud[[10]]
    kept,diag=f.filter(sparse,.7)
    assert len(kept)==1 and diag['ignored_points']==0


def test_independent_box_fragments_must_have_consistent_combined_shape():
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    # Individually identical plausible fragments moving across the scene do
    # not prove a stationary small box before contact/identity is established.
    for t,x in [(0.,1.),(.1,1.05),(.2,1.10)]:
        cloud=faces(.15,.15,.4,center=(x,1.))
        assert classify_cluster(cloud)[0]
        kept,diag=f.filter(cloud,t)
        assert len(kept)==len(cloud)
    assert diag['clusters'][0]['reason']=='small_box_inconsistent_views'
    assert not f.boxes
    # Old inconsistent evidence expires; a now stable box can be confirmed.
    for t in [2.,2.1,2.2]:kept,diag=f.filter(cloud,t)
    assert len(kept)==0


def test_established_pushable_box_identity_can_move_after_confirmation():
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    for t in [0.,.1,.2]:f.filter(faces(.15,.15,.4),t)
    for t,x in [(.3,1.05),(.4,1.1),(.5,1.15)]:
        cloud=faces(.15,.15,.4,center=(x,1.))
        assert len(f.filter(cloud,t)[0])==0


def test_three_consistent_scans_can_confirm_at_real_processed_cadence():
    cloud=faces(.15,.15,.4)
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    assert len(f.filter(cloud,0.)[0])==len(cloud)
    assert len(f.filter(cloud,.4)[0])==len(cloud)
    assert len(f.filter(cloud,.8)[0])==0


@pytest.mark.parametrize('length,width,height',[(.35,.35,.4),(.4,.6,.2),(.15,.15,.6)])
def test_fuller_non_box_view_cancels_identity_before_sparse_view(length,width,height):
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    fragment=faces(.15,.15,.4)
    for t in [0.,.1,.2]:f.filter(fragment,t)
    assert f.boxes
    full=faces(length,width,height)
    kept,diag=f.filter(full,.3)
    assert len(kept)==len(full)
    assert not f.boxes
    assert any(c.get('invalidated_box_hypotheses',0)>0 for c in diag['clusters'])
    # Partial returns must not resurrect a disproven box identity.
    sparse=fragment[[10]]
    assert len(f.filter(sparse,.4)[0])==len(sparse)


def test_disproved_pending_fragment_needs_three_new_views():
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    fragment=faces(.15,.15,.4)
    f.filter(fragment,0.);f.filter(fragment,.1)
    f.filter(faces(.4,.6,.2),.2)
    assert not f.pending
    assert len(f.filter(fragment,.3)[0])==len(fragment)
    assert len(f.filter(fragment,.4)[0])==len(fragment)
    assert len(f.filter(fragment,.5)[0])==0
