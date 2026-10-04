import numpy as np
import pytest
from types import SimpleNamespace as S
from uav_lab_localization.ingress import validated_scan, validate_imu, RejectedScan


def cloud(points, frame='lidar_link'):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    return S(header=S(frame_id=frame), fields=[S(name=n, datatype=7, count=1, offset=i*4) for i,n in enumerate('xyz')],
             width=len(points), height=1, point_step=12, row_step=len(points)*12,
             is_bigendian=False, data=points.tobytes())


def test_blank_duplicate_and_no_return_scans_are_rejected_before_backend():
    for points in (np.empty((0,3)), np.zeros((600,3)), np.full((600,3), np.inf), np.tile([1,1,1],(600,1))):
        with pytest.raises(RejectedScan): validated_scan(cloud(points))
    rng = np.random.default_rng(2)
    points = rng.uniform(-3, 3, (1000,3))
    assert len(validated_scan(cloud(points))) > 900
    with pytest.raises(ValueError, match='frame'): validated_scan(cloud(points, 'map'))


def test_imu_truth_orientation_and_nonfinite_observations_are_rejected():
    msg = S(header=S(frame_id='imu_link'), orientation_covariance=[-1],
            angular_velocity=S(x=0.,y=0.,z=0.),linear_acceleration=S(x=0.,y=0.,z=9.81))
    validate_imu(msg)
    msg.linear_acceleration.z = float('nan')
    with pytest.raises(ValueError): validate_imu(msg)
    msg.linear_acceleration.z = 9.81; msg.orientation_covariance = [0]
    with pytest.raises(ValueError): validate_imu(msg)
