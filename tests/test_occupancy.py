import importlib.util
import numpy as np
import pytest


def implementation():
    assert importlib.util.find_spec('uav_lab_navigation'), 'navigation mapping missing'
    from uav_lab_navigation import occupancy
    return occupancy


def test_ray_leaves_occluded_and_unobserved_space_unknown():
    m = implementation(); grid = m.VoxelMap(1., (0,0,0), (6,3,3))
    grid.integrate((.5,1.5,1.5), [[3.5,1.5,1.5]])
    assert grid.state((1.5,1.5,1.5)) == m.FREE
    assert grid.state((3.5,1.5,1.5)) == m.OCCUPIED
    assert grid.state((4.5,1.5,1.5)) == m.UNKNOWN
    assert grid.state((1.5,2.5,1.5)) == m.UNKNOWN


def test_same_scan_obstacle_has_priority_over_free_rays():
    m = implementation(); grid = m.VoxelMap(1., (0,0,0), (6,3,3))
    grid.integrate((.5,1.5,1.5), [[3.5,1.5,1.5], [5.5,1.5,1.5]]*20)
    assert grid.state((3.5,1.5,1.5)) == m.OCCUPIED
    # Duplicate hits must not change one scan's confidence.
    assert grid.score[3,1,1] == 2


def test_outside_endpoint_is_clipped_without_inventing_boundary_obstacle():
    m = implementation(); grid = m.VoxelMap(1., (0,0,0), (3,3,3))
    grid.integrate((.5,1.5,1.5), [[10.,1.5,1.5]])
    assert grid.state((2.5,1.5,1.5)) == m.FREE
    assert grid.state((3,1.5,1.5)) == m.UNKNOWN


def test_unknown_and_bounds_block_full_vehicle_volume():
    m = implementation(); grid = m.VoxelMap(1., (0,0,0), (5,5,5))
    grid.observe_body((2.5,2.5,2.5), (0,0,0))
    assert grid.snapshot((0,0,0)).point_clear((2.5,2.5,2.5))
    assert not grid.snapshot((1.,1.,1.)).point_clear((2.5,2.5,2.5))
    grid.observe_body((2.5,2.5,2.5), (2.,2.,2.))
    collision = grid.snapshot((1.,1.,1.))
    assert collision.point_clear((2.5,2.5,2.5))
    assert not collision.point_clear((.5,2.5,2.5))


def test_actual_body_volume_never_clears_an_obstacle():
    m = implementation(); grid = m.VoxelMap(1., (0,0,0), (5,5,5))
    grid.integrate((.5,2.5,2.5), [[2.5,2.5,2.5]])
    grid.observe_body((2.5,2.5,2.5), (1,1,1))
    assert grid.state((2.5,2.5,2.5)) == m.OCCUPIED


def test_collision_supercover_rejects_diagonal_corner_cutting():
    m = implementation(); grid = m.VoxelMap(1., (0,0,0), (4,4,4))
    grid.observe_body((.5,.5,1.5), (0,0,0)); grid.observe_body((1.5,1.5,1.5), (0,0,0))
    assert not grid.snapshot((0,0,0)).segment_clear((.5,.5,1.5), (1.5,1.5,1.5))


@pytest.mark.parametrize('bad', [(np.nan,0,0), (np.inf,0,0), (0,0)])
def test_invalid_ray_rejected_atomically(bad):
    m = implementation(); grid = m.VoxelMap(1., (0,0,0), (4,4,4))
    with pytest.raises(ValueError): grid.integrate((.5,.5,.5), [bad])
    assert grid.version == 0 and not grid.seen.any()


def test_snapshot_is_immutable_across_new_obstacle_scan():
    m = implementation(); grid = m.VoxelMap(1., (0,0,0), (4,4,4))
    grid.observe_body((1.5,1.5,1.5), (1,1,1)); old = grid.snapshot((0,0,0))
    grid.integrate((.5,1.5,1.5), [[1.5,1.5,1.5]])
    assert old.point_clear((1.5,1.5,1.5))
    assert not grid.snapshot((0,0,0)).point_clear((1.5,1.5,1.5))
