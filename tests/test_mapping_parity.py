import numpy as np
from uav_lab_navigation.occupancy import VoxelMap
from uav_lab_navigation.local_map import RollingMap


def test_static_and_timed_rays_have_identical_evidence_for_fixed_origin():
    fixed=VoxelMap();timed=RollingMap()
    source=np.array([0.,0.,2.])
    points=np.random.default_rng(7).uniform([-9,-9,-2],[9,9,7],(2000,3))
    fixed.integrate(source,points)
    timed.integrate_beams(np.repeat(source[None,:],len(points),axis=0),points)
    assert np.array_equal(fixed.score,timed.score)
    assert np.array_equal(fixed.seen,timed.seen)
    assert np.array_equal(fixed.snapshot([.65,.65,.55]).free,timed.snapshot([.65,.65,.55]).free)


def test_empty_static_scan_keeps_original_no_evidence_semantics():
    fixed=VoxelMap()
    fixed.integrate([0,0,2],np.empty((0,3)))
    assert fixed.version==1 and not fixed.seen.any() and not fixed.score.any()
