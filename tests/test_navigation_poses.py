import importlib.util
import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def implementation():
    assert importlib.util.find_spec('uav_lab_navigation.poses'), 'sensor-time alignment missing'
    from uav_lab_navigation import poses
    return poses


def test_source_time_interpolation_and_no_extrapolation():
    m=implementation();history=m.PoseHistory()
    history.add(10,[0,0,0],[0,0,0,1]);history.add(10.2,[2,0,0],Rotation.from_euler('z',90,degrees=True).as_quat())
    pose=history.at(10.1)
    assert np.allclose(pose[:3,3],[1,0,0])
    assert np.allclose(pose[:3,:3]@[1,0,0],[2**-.5,2**-.5,0])
    with pytest.raises(ValueError):history.at(9)
    with pytest.raises(ValueError):history.at(12)
    with pytest.raises(ValueError):history.add(9,[0,0,0],[0,0,0,1])

def test_missing_pose_interval_is_not_interpolated_as_valid_motion():
    history=implementation().PoseHistory()
    history.add(10,[0,0,0],[0,0,0,1]);history.add(11,[0,0,0],[0,0,0,1])
    with pytest.raises(ValueError,match='gap'):history.at(10.5)


def test_frozen_alignment_and_extrinsics_apply_to_origin_and_endpoints():
    m=implementation();body=np.eye(4);body[:3,3]=[10,2,0]
    alignment=np.eye(4);alignment[:3,3]=[9,0,0]
    extrinsic=np.eye(4);extrinsic[:3,3]=[0,0,.16];extrinsic[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
    origin,points=m.register_scan([[1,0,0]],body,alignment,extrinsic)
    assert np.allclose(origin,[1,2,.16]) and np.allclose(points,[[1,3,.16]])


@pytest.mark.parametrize('quaternion',[[0,0,0,0],[0,0,0,2],[0,0,np.nan,1]])
def test_bad_pose_is_rejected_without_publishing_map(quaternion):
    with pytest.raises(ValueError):implementation().PoseHistory().add(10,[0,0,0],quaternion)


def test_self_returns_are_removed_only_inside_calibrated_body():
    from uav_lab_navigation.poses import remove_self_returns
    extrinsic=np.eye(4);extrinsic[:3,3]=[0,0,.16]
    points=np.array([[.2,.2,-.1],[.6,0,-.1],[0,0,1.]])
    filtered=remove_self_returns(points,extrinsic,[.4,.4,.3])
    assert np.array_equal(filtered,points[1:])
