import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from uav_lab_localization.evaluation import evaluate, validate_trajectory


def trajectory():
    t = np.arange(1, 9, 0.1)
    xyz = np.column_stack([np.sin(t), np.cos(t), t/5])
    quat = Rotation.from_euler('z', t/10).as_quat()
    return np.column_stack([t, xyz, quat])


def test_one_rigid_alignment_removes_gauge_without_scale():
    truth = trajectory()
    estimate = truth.copy()
    r = Rotation.from_euler('xyz', [0.1, 0.2, 0.3])
    estimate[:, 1:4] = r.apply(truth[:, 1:4]) + [3, 4, 5]
    estimate[:, 4:] = (r * Rotation.from_quat(truth[:, 4:])).as_quat()
    report = evaluate(estimate, truth)
    assert report['ate_rmse_m'] < 1e-10
    assert report['attitude_rmse_deg'] < 1e-9
    assert report['raw_position_rmse_m'] > 5
    assert report['rpe_translation_rmse_m'] < 1e-10
    assert report['coverage'] == 1.0
    estimate[:, 1:4] *= 2
    assert evaluate(estimate, truth)['ate_rmse_m'] > 0.5


def test_timestamp_regression_and_invalid_quaternion_rejected():
    t = trajectory()
    t[2, 0] = t[1, 0]
    with pytest.raises(ValueError, match='increasing'):
        validate_trajectory(t)
    t = trajectory()
    t[2, 4:] = 0
    with pytest.raises(ValueError, match='quaternion'):
        validate_trajectory(t)


def test_missing_estimates_reduce_coverage_and_large_gaps_fail():
    truth = trajectory()
    estimate = truth[::2]
    report = evaluate(estimate, truth, expected_stamps=truth[:, 0])
    assert report['coverage'] == pytest.approx(0.5)
    assert report['max_output_gap_s'] == pytest.approx(0.2)
    with pytest.raises(ValueError, match='association'):
        evaluate(truth, np.column_stack([truth[:, 0]+100, truth[:, 1:]]))
