"""Real upstream optimizers must produce executable, independently checked curves."""
from pathlib import Path
import numpy as np
import pytest
from uav_lab_bridge.continuous_trajectory import State
from uav_lab_navigation.occupancy import CollisionMap


@pytest.mark.parametrize('backend',['ego','fast_planner','gcopter'])
def test_real_planner_core_runs_and_preserves_nonzero_initial_derivatives(tmp_path,backend):
    root=Path('.').resolve()
    if not (root/'.deps/ego').exists():pytest.skip('native planner sources absent')
    binary=root/'.deps/planners'/backend/'install/bin/planner_core'
    assert binary.is_file(),'build the real '+backend+' core'
    from uav_lab_navigation.planner_backends import native_plan
    collision=CollisionMap(.2,np.array([-2.,-2.,0.]),np.ones((30,30,25),dtype=bool),4,10.)
    initial=State(np.array([0.,0.,2.]),np.array([.1,0.,0.]),np.array([.02,0.,0.]))
    result=native_plan(root,backend,collision,initial,[2.,2.,2.],tmp_path/backend,timeout=30.)
    assert result['success'],result['reason']
    curve=result['curve']
    assert np.allclose(curve.sample(0).velocity,initial.velocity,atol=1e-6)
    assert np.allclose(curve.sample(0).acceleration,initial.acceleration,atol=1e-6)
    assert np.allclose(curve.sample(curve.duration).position,[2.,2.,2.],atol=1e-6)
    assert curve.collision_free(collision)
    assert result['core_source_ref'] and result['binary_sha256']
    assert result['core_peak_rss_bytes']>0
    assert result['fallback'] is False
