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
    f.filter(full,.1);kept,diag=f.filter(partial,.2)
    assert len(kept)==0
    kept,_=f.filter(partial,9.)
    assert len(kept)==len(partial)


def test_noisy_box_faces_are_ignored_without_dropping_round_body():
    rng=np.random.default_rng(3)
    noisy=faces(.15,.15,.4,np.pi/4)
    noisy[:,:2]+=rng.normal(0,.018,noisy[:,:2].shape)
    assert classify_cluster(noisy)[0]


def test_sparse_contact_returns_maintain_box_but_cannot_initialize():
    points=faces(.15,.15,.4)
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    sparse=points[[10]]
    assert len(f.filter(sparse,0)[0])==1
    f.filter(points,.1)
    assert len(f.filter(sparse,.2)[0])==0
    assert len(f.filter(sparse,9)[0])==1


def test_current_tracked_robot_is_preserved_even_if_shape_or_cache_says_box():
    cloud=faces(.15,.15,.4)
    f=SmallBoxFilter();f.set_grid(.05,[0,0],80,80,[0]*6400)
    f.filter(cloud,0.)
    kept,diag=f.filter(cloud,.1,protected_centers=[[1,1]])
    assert len(kept)==len(cloud) and diag['ignored_points']==0
    assert diag['clusters'][0]['reason']=='tracked_robot'
    # An unrelated observed robot does not prevent this box being ignored.
    assert len(f.filter(cloud,.2,protected_centers=[[2,2]])[0])==0
