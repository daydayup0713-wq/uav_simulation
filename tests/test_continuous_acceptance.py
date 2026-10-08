"""Acceptance must reject missing heartbeat and avoid rate bias between routes."""
import numpy as np
from uav_lab_experiments.evaluation import evaluate_continuous


def fixture():
    references, actual, heartbeats, setpoints = [], [], [], []
    for identifier, start in [('first', 0.), ('second', 20.)]:
        for t in np.arange(0., 2.01, .02):
            references.append({'id': identifier, 'stamp': start+t, 'wall': start+t,
                               'position': [.1*t, 0, 2], 'velocity': [.1, 0, 0], 'acceleration': [0, 0, 0]})
        for t in np.arange(0., 2.01, .1):
            actual.append({'stamp': start+t, 'position': [.1*t, .1, 2]})
        heartbeats.extend(np.arange(start, start+2.001, .05))
        setpoints.extend(np.arange(start, start+2.001, .02))
    return references, actual, heartbeats, setpoints


def test_gaps_between_runs_do_not_make_healthy_heartbeat_appear_slow():
    result = evaluate_continuous(*fixture())
    assert result['passed']
    assert abs(result['heartbeat_rate_hz'] - 20) < 1
    assert abs(result['tracking_p95_m'] - .1) < 1e-8


def test_missing_heartbeat_fails_even_when_tracking_and_reference_are_good():
    references, actual, _, setpoints = fixture()
    result = evaluate_continuous(references, actual, [], setpoints)
    assert not result['passed']
    assert 'heartbeat' in result['failed_checks']


def test_actual_pose_outside_reference_interval_is_not_extrapolated():
    references, actual, heartbeats, setpoints = fixture()
    actual.append({'stamp': 10., 'position': [100, 100, 100]})
    result = evaluate_continuous(references, actual, heartbeats, setpoints)
    assert result['tracking_samples'] == 42
    assert result['passed']


def test_tracking_or_jerk_violation_is_a_failed_report():
    references, actual, heartbeats, setpoints = fixture()
    actual[0]['position'] = [10, 0, 2]
    references[1]['acceleration'] = [.5, 0, 0]
    result = evaluate_continuous(references, actual, heartbeats, setpoints)
    assert not result['passed']
    assert 'jerk' in result['failed_checks']
