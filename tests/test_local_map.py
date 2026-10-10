"""Long routes keep a fixed memory budget and never turn newly exposed cells free."""
import importlib
import importlib.util
import numpy as np
import pytest


def api():
    assert importlib.util.find_spec('uav_lab_navigation.local_map'),'bounded rolling map missing'
    return importlib.import_module('uav_lab_navigation.local_map')


def test_recentering_keeps_world_evidence_and_new_cells_unknown():
    grid=api().RollingMap(.2,[-6.,-6.,-1.],[6.,6.,5.])
    grid.score[40,30,15]=4;grid.seen[40,30,15]=True
    point=grid.lower+(np.array([40,30,15])+.5)*grid.resolution
    old_lower=grid.lower.copy();version=grid.version
    assert grid.recenter([4.,0.,2.])
    assert grid.state(point)==1 and grid.version>version
    assert np.all(grid.score[-20:,:,:]==0) and not grid.seen[-20:,:,:].any()
    assert np.allclose(grid.lower,old_lower+[4.,0.,0.])


def test_hundred_meter_route_keeps_array_bytes_constant():
    grid=api().RollingMap();bytes_=grid.score.nbytes+grid.seen.nbytes
    for x in range(0,121,2):
        grid.recenter([x,0.,2.]);grid.observe_body([x,0.,2.],[.4,.4,.3])
        assert grid.score.nbytes+grid.seen.nbytes==bytes_
        assert grid.inside(np.array([x,0.,2.]))
    assert grid.score.size==108000


def test_disjoint_window_drops_old_evidence_and_remains_unknown():
    grid=api().RollingMap();grid.score[:]=-4;grid.seen[:]=True
    old=grid.snapshot([.4,.4,.3])
    grid.recenter([100.,0.,2.])
    assert not grid.score.any() and not grid.seen.any()
    assert old.free.any() and np.allclose(old.lower,[-6.,-6.,-1.])


def test_timed_scan_registration_uses_each_measured_beam_pose():
    from uav_lab_navigation.poses import PoseHistory
    history=PoseHistory();history.add(1.,[0,0,2],[0,0,0,1]);history.add(1.1,[.1,0,2],[0,0,0,1])
    origins,points=api().register_beams(np.array([[1.,0.,0.],[1.,0.,0.]]),np.array([1.,1.1]),
                                      history,np.eye(4),np.eye(4))
    assert np.allclose(origins,[[0,0,2],[.1,0,2]])
    assert np.allclose(points,[[1,0,2],[1.1,0,2]])
    with pytest.raises(ValueError,match='bracketed'):
        api().register_beams(np.array([[1.,0.,0.]]),np.array([1.2]),history,np.eye(4),np.eye(4))


def test_ray_free_evidence_uses_measured_origins_instead_of_common_scan_start():
    grid=api().RollingMap(.2,[-2.,-2.,0.],[4.,4.,4.])
    grid.integrate_beams(np.array([[0.,0.,1.],[0.,2.,1.]]),np.array([[2.,0.,1.],[2.,2.,1.]]))
    assert grid.state([1.,0.,1.])==-1 and grid.state([1.,2.,1.])==-1
    assert grid.state([1.,1.,1.])==0


def test_local_map_snapshot_refuses_negative_or_nonfinite_source_time():
    grid=api().RollingMap()
    with pytest.raises(ValueError,match='source'):
        grid.mark_source(float('nan'))
    with pytest.raises(ValueError,match='source'):
        grid.mark_source(-1.)
