import numpy as np
from scipy.spatial.transform import Rotation
from uav_lab_localization.frames import control_frame_alignment


def test_one_stationary_frame_alignment_preserves_body_pose_and_control_target():
    lio = np.eye(4); lio[:3, 3] = [0, 0, 0]
    flight = np.eye(4); flight[:3, 3] = [.1, -.2, .225]
    flight[:3, :3] = Rotation.from_euler('z', .01).as_matrix()
    alignment = control_frame_alignment(lio, flight)
    assert np.allclose(alignment @ flight, lio)
    target = np.array([3, 3, 2, 1.])
    before = target.copy()
    map_correction = np.eye(4); map_correction[:3, 3] = [1, 2, 0]
    _ = map_correction @ alignment @ target
    assert np.array_equal(target, before)
