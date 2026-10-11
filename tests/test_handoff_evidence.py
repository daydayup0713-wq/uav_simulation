import importlib,importlib.util
import numpy as np
from uav_lab_bridge.continuous_trajectory import Trajectory


def api():
    assert importlib.util.find_spec('handoff_evidence'),'independent accepted-curve handoff checks missing'
    return importlib.import_module('handoff_evidence')


def fixture():
    first=Trajectory.generate([[0.,0.,2.],[2.,0.,2.]])
    join=first.sample(2.)
    second=Trajectory.generate([join.position,[2.,1.,2.]],initial=join)
    return [{'event':'trajectory_accepted','trajectory_id':'first','replaces_id':'','starts_at':10.,'trajectory':first.to_dict()},
            {'event':'trajectory_accepted','trajectory_id':'second','replaces_id':'first','starts_at':12.,'trajectory':second.to_dict()}]


def test_actual_accepted_curves_join_at_future_pva_and_have_analytic_limits():
    report=api().recheck(fixture())
    assert report['passed'] and report['joins']==1 and report['maximum_join_error']<1e-8


def test_a_forged_success_flag_cannot_hide_a_discontinuous_replacement():
    rows=fixture();rows[1]['starts_at']+=.3
    report=api().recheck(rows)
    assert not report['passed'] and 'C2_HANDOFF' in report['failed_checks']
