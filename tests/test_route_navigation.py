"""Semantic route handoffs happen before terminal rest, with exact future derivatives."""
import importlib,importlib.util
import numpy as np
import pytest
pytest.importorskip('uav_lab_interfaces.action')
from uav_lab_bridge.continuous_trajectory import Trajectory


def api():
    assert importlib.util.find_spec('uav_lab_navigation.route_navigation'),'continuous multi-control navigation missing'
    return importlib.import_module('uav_lab_navigation.route_navigation')


def test_future_checkpoint_triggers_before_programmatic_stop():
    curve=Trajectory.generate([[0.,0.,2.],[2.,0.,2.]])
    elapsed=curve.duration-1.51
    join=api().checkpoint_join(curve,elapsed,10_000_000_000,[2.,0.,2.],1.5)
    assert join is not None and np.linalg.norm(join[1].velocity)>0
    assert join[0]<curve.duration
    assert np.linalg.norm(join[1].position-[2.,0.,2.])<=.2


def test_early_or_late_reference_cannot_skip_control_or_claim_a_moving_handoff():
    curve=Trajectory.generate([[0.,0.,2.],[2.,0.,2.]])
    assert api().checkpoint_join(curve,0.,10_000_000_000,[2.,0.,2.],1.5) is None
    assert api().checkpoint_join(curve,curve.duration-.1,10_000_000_000,[2.,0.,2.],1.5) is None
