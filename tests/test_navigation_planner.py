import importlib.util
import numpy as np
import pytest
from test_occupancy import implementation


def planner():
    assert importlib.util.find_spec('uav_lab_navigation.planner'), '3D planning missing'
    from uav_lab_navigation.planner import plan
    return plan


def room():
    grid = implementation().VoxelMap(.5,(0,0,0),(6,6,4))
    grid.observe_body((3,3,2),(3,3,2))
    return grid


def test_3d_route_over_wall_and_every_simplified_segment_clear():
    grid = room()
    # An x wall spans all y, but its height leaves a genuinely 3D route.
    grid.score[5:7,:,0:4] = 4
    collision = grid.snapshot((.25,.25,.25))
    a, b = (1.25,3.25,1.25),(4.75,3.25,1.25)
    assert not collision.segment_clear(a,b)
    result = planner()(collision,a,b)
    assert result.success and max(p[2] for p in result.points)>2.25
    assert all(collision.segment_clear(a,b) for a,b in zip(result.points,result.points[1:]))
    assert np.allclose(result.points[0],a) and np.allclose(result.points[-1],b)


def test_sealed_goal_returns_no_path_without_unbounded_search():
    grid = room(); grid.score[5:7,:,:] = 4
    result = planner()(grid.snapshot((0,0,0)),(1.25,3.25,1.25),(4.75,3.25,1.25))
    assert not result.success and result.reason == 'NO_PATH' and result.expanded<50000


def test_unknown_goal_rejected_and_budget_is_distinct_from_no_path():
    grid = room(); grid.score[9,6,2]=0
    result=planner()(grid.snapshot((0,0,0)),(1.25,3.25,1.25),(4.75,3.25,1.25))
    assert not result.success and result.reason=='GOAL_BLOCKED_OR_UNOBSERVED'
    grid.score[9,6,2]=-1; grid.score[5:7,:,0:4]=4
    result=planner()(grid.snapshot((0,0,0)),(1.25,3.25,1.25),(4.75,3.25,1.25),max_expansions=1)
    assert not result.success and result.reason=='SEARCH_BUDGET_EXCEEDED'


@pytest.mark.parametrize('target', [(np.nan,1,1),(8,1,1),(1,1,np.inf)])
def test_bad_target_has_deterministic_failure(target):
    result=planner()(room().snapshot((0,0,0)),(1.25,3.25,1.25),target)
    assert not result.success and result.reason in ('INVALID_GOAL','OUTSIDE_BOUNDS')


def test_planner_never_connects_diagonal_free_cells_through_blocked_corner():
    m=implementation(); grid=m.VoxelMap(1,(0,0,0),(3,3,3))
    grid.observe_body((.5,.5,1.5),(0,0,0)); grid.observe_body((1.5,1.5,1.5),(0,0,0))
    result=planner()(grid.snapshot((0,0,0)),(.5,.5,1.5),(1.5,1.5,1.5))
    assert not result.success and result.reason=='NO_PATH'
