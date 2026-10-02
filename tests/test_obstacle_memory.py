import sys
from pathlib import Path
from math import cos,sin,pi
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/hsl_planning'))
from hsl_planning.obstacle_memory import ObstacleMemory
from hsl_planning.core import Pose2,VoxelWorld,astar,safe_segment


def test_unmapped_box_memory_clear_and_expire():
    memory=ObstacleMemory(lifetime=8.)
    own=Pose2(0,0)
    memory.update([(1,0,.2)],own,0.)
    assert memory.points(1.)==[(1,0,.2)]
    memory.update([(0.5,0,.2)],own,1.)  # nearer obstacle occludes old hit
    assert len(memory.points(1.))==2
    memory.update([(2,0,.2)],own,2.)  # ray now sees through old boxes
    assert memory.points(2.)==[(2,0,.2)]
    assert not memory.points(11.)
    memory.update([(1,0,.4)],own,0.)  # simulation reset
    assert memory.points(0.)==[(1,0,.4)]


def test_diagonal_disk_can_pass_but_does_not_cut_a_corner():
    world=VoxelWorld(.15,.23)
    # 0.60m corridor, rotated 45deg. Full protected diameter is .46m.
    walls=[]
    for i in range(-30,31):
        t=i*.04
        for side in (-.30,.30):
            walls.append(((t-side)/2**.5,(t+side)/2**.5,.3))
    world.update(walls,[],None)
    route=astar(world,Pose2(-.6,-.6),Pose2(.6,.6))
    assert route
    assert all(safe_segment(world,a,b,safety_margin=0.) for a,b in zip(route,route[1:]))
    # The former extra .14m requires .74m and rejects this physically safe corridor.
    assert not safe_segment(world,Pose2(-.6,-.6),Pose2(.6,.6),safety_margin=.14)
    world.update([(0,0,.3)],[],None)
    route=astar(world,Pose2(-.6,-.6),Pose2(.6,.6))
    assert route
    assert all(safe_segment(world,a,b,safety_margin=0.) for a,b in zip(route,route[1:]))
