"""Fail closed on map identity, unsafe curves and broken replanning boundaries."""
import importlib
import importlib.util
import json
import numpy as np
import pytest
from uav_lab_bridge.continuous_trajectory import Trajectory,State,Limits
from uav_lab_navigation.occupancy import CollisionMap


def api():
    assert importlib.util.find_spec('uav_lab_navigation.planner_backends'), 'planner backend adapter missing'
    return importlib.import_module('uav_lab_navigation.planner_backends')


def space():
    return CollisionMap(.2,np.array([-2.,-2.,0.]),np.ones((30,30,25),dtype=bool),4,10.)


def packet(collision,curve):
    return {'schema':1,'success':True,'map_sha256':api().map_identity(collision),
            'trajectory':curve.to_dict()}


def test_identical_observed_map_bytes_and_identity_for_every_core(tmp_path):
    m=api();collision=space();collision.free[10,10,10]=False
    ids=[];raw=[]
    for backend in ('ego','fast_planner','gcopter'):
        request=m.write_request(tmp_path/backend,backend,collision,State(np.array([0.,0.,1.])),[2.,2.,1.],[[0.,0.,1.],[2.,2.,1.]])
        ids.append(request['map_sha256']);raw.append(open(request['free_file'],'rb').read())
    assert len(set(ids))==1 and len(set(raw))==1
    assert raw[0][np.ravel_multi_index((10,10,10),collision.free.shape)]==0


def test_native_reply_for_other_map_is_rejected():
    m=api();collision=space();state=State(np.array([0.,0.,1.]))
    curve=Trajectory.generate([state.position,[2.,2.,1.]])
    p=packet(collision,curve);p['map_sha256']='0'*64
    with pytest.raises(ValueError,match='map identity'):
        m.validate_curve(p,collision,state,[2.,2.,1.])


def test_native_curve_unknown_crossing_is_rejected_even_with_safe_endpoints():
    m=api();collision=space();collision.free[14,:,:]=False
    state=State(np.array([0.,0.,1.]));goal=[2.,0.,1.]
    curve=Trajectory.generate([state.position,goal])
    with pytest.raises(ValueError,match='collision|unknown'):
        m.validate_curve(packet(collision,curve),collision,state,goal)


def test_replanning_native_curve_preserves_pva_and_terminal_rest():
    m=api();collision=space();state=State(np.array([0.,0.,1.]),np.array([.1,0.,0.]),np.array([.02,0.,0.]))
    goal=[2.,2.,1.];curve=Trajectory.generate([state.position,goal],initial=state)
    recovered=m.validate_curve(packet(collision,curve),collision,state,goal)
    assert np.allclose(recovered.sample(0).velocity,state.velocity)
    wrong=State(state.position,np.zeros(3),np.zeros(3))
    with pytest.raises(ValueError,match='initial'):
        m.validate_curve(packet(collision,curve),collision,wrong,goal)


def test_native_derivative_overrun_cannot_be_hidden_by_claimed_limits():
    m=api();collision=space();state=State(np.array([0.,0.,1.]));goal=[2.,2.,1.]
    curve=Trajectory.generate([state.position,goal],limits=Limits(speed=2.,acceleration=2.,jerk=4.))
    p=packet(collision,curve);p['trajectory']['limits']={'speed':.5,'acceleration':.5,'jerk':1.}
    with pytest.raises(ValueError,match='limit'):
        m.validate_curve(p,collision,state,goal)


def test_every_safe_corridor_voxel_is_observed_free():
    m=api();collision=space();collision.free[12:14,8:16,3:14]=False
    route=[[0.,-1.,1.],[2.,-1.,1.],[2.,2.,1.]]
    boxes=m.safe_corridors(collision,route)
    assert boxes and np.all(np.asarray(boxes[0]['lower'])<route[0])
    for box in boxes:
        lo=np.asarray(collision.index(box['lower']));hi=np.asarray(collision.index(box['upper']))
        assert collision.free[tuple(slice(a,b+1) for a,b in zip(lo,hi))].all()


def test_long_thin_grid_cannot_allocate_unbounded_native_search_pool(tmp_path):
    collision=CollisionMap(.2,np.zeros(3),np.ones((10000,2,2),dtype=bool),4,10.)
    state=State(np.array([.1,.1,.1]))
    with pytest.raises(ValueError,match='bounded'):
        api().write_request(tmp_path/'long','ego',collision,state,[.3,.1,.1],[[.1,.1,.1],[.3,.1,.1]])


@pytest.mark.parametrize('timeout',[0.,-1.,float('nan'),float('inf'),3600.])
def test_native_budget_is_finite_positive_and_bounded_before_artifact_lookup(tmp_path,timeout):
    with pytest.raises(ValueError,match='budget'):
        api().native_plan(tmp_path,'ego',space(),State(np.array([0.,0.,1.])),[2.,2.,1.],tmp_path/'new',timeout=timeout)
    assert not (tmp_path/'new').exists()


def test_native_position_curve_keeps_initial_yaw_rate_and_shortest_final_heading():
    state=State(np.array([0.,0.,1.]),np.array([.1,0.,0.]),np.zeros(3),yaw=3.0,yaw_rate=.1)
    curve=Trajectory.generate([state.position,[2.,2.,1.]],initial=state)
    result=api().with_heading(curve,state,-3.0)
    assert result.sample(0).yaw==pytest.approx(3.)
    assert result.sample(0).yaw_rate==pytest.approx(.1)
    assert result.sample(result.duration).yaw==pytest.approx(2*np.pi-3.)
    assert result.sample(result.duration).yaw_rate==pytest.approx(0.,abs=1e-8)
    for t in np.linspace(0.,curve.duration,100):
        assert np.allclose(result.sample(t).position,curve.sample(t).position)


def test_unified_pipeline_refuses_unknown_planner_without_an_astar_fallback(tmp_path):
    with pytest.raises(ValueError,match='planner'):
        api().plan_curve(tmp_path,'missing',space(),State(np.array([0.,0.,1.])),[2.,2.,1.],tmp_path/'new')


def test_missing_native_installation_is_an_explicit_archived_failure(tmp_path):
    result=api().native_plan(tmp_path,'ego',space(),State(np.array([0.,0.,1.])),[2.,2.,1.],tmp_path/'run')
    assert result['success'] is False and result['fallback'] is False
    assert result['implementation_sha256'] is None and result['core_source_ref'] is None
    assert (tmp_path/'run/report.json').exists()
