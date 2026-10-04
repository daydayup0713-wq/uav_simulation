import numpy as np
import pytest

from uav_lab_localization.quality import LocalizationQuality, normalize_pose


def ready():
    gate = LocalizationQuality(warmup_samples=3)
    for i in range(3):
        now, source = i*0.1, 10+i*0.1
        gate.observe_source('imu', source, now)
        gate.observe_source('points', source, now)
        gate.observe_pose(source, [i*0.01, 0, 0], [0, 0, 0, 1], now)
    assert gate.check(10.2, 0.2)['ready']
    return gate


def test_stale_or_paused_stream_latches_failure():
    gate = ready()
    assert not gate.check(10.2, 2.0)['ready']
    for i in range(4):
        gate.observe_source('imu', 11+i*0.1, 2+i*0.1)
        gate.observe_source('points', 11+i*0.1, 2+i*0.1)
        gate.observe_pose(11+i*0.1, [0, 0, 0], [0, 0, 0, 1], 2+i*0.1)
    assert not gate.check(11.3, 2.3)['ready']
    assert gate.reason


@pytest.mark.parametrize('stamp,position,quaternion', [
    (10.1, [0, 0, 0], [0, 0, 0, 1]),
    (10.3, [20, 0, 0], [0, 0, 0, 1]),
    (10.3, [np.nan, 0, 0], [0, 0, 0, 1]),
    (10.3, [0, 0, 0], [0, 0, 0, 0]),
])
def test_regression_jump_and_invalid_pose_rejected(stamp, position, quaternion):
    gate = ready()
    assert not gate.observe_pose(stamp, position, quaternion, 0.3)
    assert not gate.check(10.3, 0.3)['ready']


def test_no_truth_or_imu_orientation_needed_and_twist_is_body_frame():
    result = normalize_pose([1, 2, 3], [0, 0, 2**-0.5, 2**-0.5], [1, 0, 0])
    assert np.allclose(result['body_velocity'], [0, -1, 0])
    gate = LocalizationQuality(warmup_samples=1)
    gate.observe_pose(10, [0, 0, 0], [0, 0, 0, 1], 0)
    assert not gate.check(10, 0)['ready']  # Odometry alone is insufficient evidence.
