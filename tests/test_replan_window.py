"""Native solving time is reserved before the exact future P/V/A join."""
import numpy as np
import pytest
pytest.importorskip('uav_lab_interfaces.action')
from uav_lab_bridge.continuous_trajectory import Trajectory
from uav_lab_navigation import continuous_navigation as navigation


def test_replan_join_reserves_selected_solver_lead_and_preserves_state():
    assert hasattr(navigation,'replan_join'),'solver scheduling window missing'
    curve=Trajectory.generate([[0.,0.,2.],[4.,0.,2.]])
    join=navigation.replan_join(curve,1.,10_000_000_000,1.5)
    assert join is not None
    elapsed,state,starts=join
    assert elapsed==2.5 and starts==11_500_000_000
    original=curve.sample(2.5)
    for key in ('position','velocity','acceleration'):
        assert np.allclose(getattr(state,key),getattr(original,key))


def test_near_endpoint_cannot_promise_a_shorter_than_reserved_scheduling_window():
    assert hasattr(navigation,'replan_join'),'solver scheduling window missing'
    curve=Trajectory.generate([[0.,0.,2.],[4.,0.,2.]])
    assert navigation.replan_join(curve,curve.duration-.1,10_000_000_000,1.5) is None


def test_reserved_join_cannot_exceed_adapter_two_second_window():
    curve=Trajectory.generate([[0.,0.,2.],[4.,0.,2.]])
    with pytest.raises(ValueError,match='scheduling'):
        navigation.replan_join(curve,1.,10_000_000_000,2.5)
