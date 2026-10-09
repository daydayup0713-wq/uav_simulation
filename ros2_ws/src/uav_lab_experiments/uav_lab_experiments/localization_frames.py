import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    result[:3, 3] = [p.x,p.y,p.z]
    if not np.isfinite(result).all(): raise ValueError('non-finite frame pose')
    return result


def control_frame_alignment(lio_body_pose, flight_body_pose):
    """A one-time stationary frame adapter; never a prior fed into LIO."""
    return np.asarray(lio_body_pose) @ np.linalg.inv(flight_body_pose)
