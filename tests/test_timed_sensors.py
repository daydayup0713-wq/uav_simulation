import numpy as np
import pytest


def test_each_beam_measures_wall_at_its_own_pose_and_time():
    from uav_lab_experiments.timed_sensors import PoseBuffer, measure_rays
    poses = PoseBuffer()
    poses.add(1., [0, 0, 1], [0, 0, 0, 1])
    poses.add(1.1, [1, 0, 1], [0, 0, 0, 1])
    boxes = np.array([[[5, -5, 0], [5.1, 5, 5]]])
    xyz = measure_rays(poses, np.array([1., 1.05, 1.1]),
                       np.tile([1., 0, 0], (3, 1)), boxes, np.eye(4), .1, 20, 0, np.random.default_rng(1))
    assert xyz[:, 0] == pytest.approx([5., 4.5, 4.])
    assert xyz[:, 1:] == pytest.approx(np.zeros((3, 2)))
    with pytest.raises(ValueError, match='bracket'):
        measure_rays(poses, np.array([1.11]), np.array([[1., 0, 0]]), boxes,
                     np.eye(4), .1, 20, 0, np.random.default_rng(1))


def test_pose_interpolation_rotates_measurement_and_refuses_regression():
    from uav_lab_experiments.timed_sensors import PoseBuffer
    poses = PoseBuffer()
    poses.add(1., [0, 0, 0], [0, 0, 0, 1])
    poses.add(1.1, [1, 0, 0], [0, 0, np.sqrt(.5), np.sqrt(.5)])
    p, r = poses.at_many(np.array([1.05]))
    assert p[0] == pytest.approx([.5, 0, 0])
    assert r[0] @ [1, 0, 0] == pytest.approx([np.sqrt(.5), np.sqrt(.5), 0], abs=1e-8)
    with pytest.raises(ValueError, match='monotonic'):
        poses.add(1.09, [0, 0, 0], [0, 0, 0, 1])


def test_mechanical_ring_and_livox_line_are_physical_channels():
    from uav_lab_experiments.timed_sensors import scan_pattern, point_records
    config = {'kind': 'mechanical', 'hz': 10, 'horizontal_samples': 200,
              'vertical_samples': 16, 'vertical_fov_rad': .6}
    times, directions, channels = scan_pattern(config, 2.)
    assert len(times) == 3200
    assert np.min(times) == 2.
    assert np.max(times) < 2.1
    assert set(channels) == set(range(16))
    assert np.linalg.norm(directions, axis=1) == pytest.approx(np.ones(3200))
    records = point_records(np.ones((3200, 3)), times - 2, channels, 'mechanical')
    assert records.dtype.names == ('x', 'y', 'z', 'intensity', 'time', 'ring', 'padding')
    config.update(kind='livox', vertical_samples=4)
    _, first, line = scan_pattern(config, 2.)
    _, next_scan, _ = scan_pattern(config, 2.1)
    assert not np.allclose(first, next_scan)
    assert set(line) == {0, 1, 2, 3}
    assert 'ring' not in point_records(first, np.zeros(len(first)), line, 'livox').dtype.names


def test_rtk_lost_outlier_recovery_preserve_source_time():
    from uav_lab_experiments.timed_sensors import rtk_measurement
    config = {'origin': [47., 8., 488.], 'fixed_stddev_m': .02, 'seed': 42,
              'events': [{'start_s': 2., 'end_s': 3., 'mode': 'lost'},
                         {'start_s': 3., 'end_s': 4., 'mode': 'outlier', 'offset_m': [30, 0, 0]}]}
    rng = np.random.default_rng(42)
    fixed = rtk_measurement(config, 1., [0, 0, 0], rng)
    lost = rtk_measurement(config, 2.5, [0, 0, 0], rng)
    bad = rtk_measurement(config, 3.5, [0, 0, 0], rng)
    recovered = rtk_measurement(config, 4.5, [0, 0, 0], rng)
    assert fixed['mode'] == recovered['mode'] == 'fixed'
    assert fixed['stamp'] == 1 and lost['stamp'] == 2.5
    assert lost['status'] == -1 and np.isnan(lost['lla']).all()
    assert bad['enu'][0] > 29 and bad['mode'] == 'outlier'
    assert np.linalg.norm(fixed['enu']) > 0
    assert fixed['covariance'][0] > 0


def test_attitude_measurement_has_declared_noise_and_no_truth_identity():
    from uav_lab_experiments.timed_sensors import noisy_attitude
    quat, covariance = noisy_attitude([0, 0, 0, 1], .005, np.random.default_rng(2))
    assert np.linalg.norm(quat) == pytest.approx(1)
    assert not np.allclose(quat, [0, 0, 0, 1])
    assert np.diag(covariance) == pytest.approx([.005**2]*3)
    with pytest.raises(ValueError, match='positive'):
        noisy_attitude([0, 0, 0, 1], 0, np.random.default_rng(2))
