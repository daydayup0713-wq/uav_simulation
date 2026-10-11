import numpy as np
import pytest
pytest.importorskip('uav_lab_interfaces.action')
from uav_lab_navigation.occupancy import CollisionMap
from uav_lab_navigation.route_navigation import local_target


def test_long_fixed_route_uses_observed_local_target_without_expanding_the_map():
    grid=CollisionMap(.2,np.array([-6.,-6.,-1.]),np.ones((60,60,30),dtype=bool),1)
    target,partial=local_target(grid,np.array([4.5,0.,2.]),np.array([60.,0.,2.]))
    assert partial and 4.5<target[0]<6
    assert grid.point_clear(target) and grid.free.shape==(60,60,30)


def test_blocked_in_window_semantic_goal_is_not_replaced_or_declared_reached():
    free=np.ones((60,60,30),dtype=bool);free[45,30,15]=False
    grid=CollisionMap(.2,np.array([-6.,-6.,-1.]),free,1)
    goal=np.array([3.1,0.,2.1]);target,partial=local_target(grid,np.array([0.,0.,2.]),goal)
    assert not partial and np.array_equal(target,goal) and not grid.point_clear(target)
