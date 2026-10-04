import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from uav_lab_bridge.clock import Px4Clock
from uav_lab_bridge.external_odometry import convert_external, ExternalOdometryGate


def sample():
    return {'source_ns': 9_950_000_000, 'position': [1., 2., 3.],
            'quaternion': Rotation.from_euler('xyz', [.1, -.2, .4]).as_quat(),
            'body_velocity': [1., 2., 3.], 'body_angular_velocity': [.1, .2, .3],
            'pose_covariance': np.diag([.01, .02, .03, .04, .05, .06]).ravel(),
            'twist_covariance': np.diag([.1, .2, .3, .4, .5, .6]).ravel(),
            'frame': 'lio_odom', 'child_frame': 'lio_base_link'}


def test_full_orientation_vectors_covariance_and_delayed_sample_timestamp():
    clock = Px4Clock(); clock.observe(2_000_000, 10_000_000_000)
    output = convert_external(sample(), clock, 10_020_000_000)
    assert output['timestamp'] == 2_020_000
    assert output['timestamp_sample'] == 1_950_000
    assert output['position'] == [2., 1., -3.]
    assert output['velocity'] == [1., -2., -3.]
    assert output['angular_velocity'] == [.1, -.2, -.3]
    assert output['position_variance'] == [.02, .01, .03]
    rotation = Rotation.from_quat(sample()['quaternion']).as_matrix()
    assert np.allclose(output['orientation_variance'], np.diag(rotation.T @ np.diag([.04, .05, .06]) @ rotation))
    assert output['velocity_variance'] == [.1, .2, .3]
    q = output['q']
    actual = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    enu_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    flu_frd = np.diag([1, -1, -1])
    assert np.allclose(actual, enu_ned @ Rotation.from_quat(sample()['quaternion']).as_matrix() @ flu_frd)


def test_invalid_frames_covariances_and_old_samples_rejected():
    clock = Px4Clock(); clock.observe(2_000_000, 10_000_000_000)
    for field, invalid in [('frame', 'map'), ('source_ns', 8_000_000_000),
                           ('quaternion', [0, 0, 0, 0]),
                           ('pose_covariance', [-1.]*36)]:
        with pytest.raises((ValueError, RuntimeError)):
            convert_external({**sample(), field: invalid}, clock, 10_000_000_000)


def test_gate_fails_closed_after_loss_without_auto_resume():
    gate = ExternalOdometryGate()
    assert not gate.ready(1)
    gate.observe_quality(True, 1); gate.observe_sample(sample(), 1)
    assert gate.ready(1.1)
    assert not gate.ready(3)
    gate.observe_quality(True, 3); gate.observe_sample(sample(), 3)
    assert not gate.ready(3.1)
    assert gate.failed


def test_px4_sample_mapping_allows_past_sample_but_not_wrong_epoch():
    clock = Px4Clock(); clock.observe(2_000_000, 10_000_000_000)
    assert clock.sample_timestamp(9_900_000_000) == 1_900_000
    with pytest.raises(RuntimeError): clock.sample_timestamp(7_000_000_000)
