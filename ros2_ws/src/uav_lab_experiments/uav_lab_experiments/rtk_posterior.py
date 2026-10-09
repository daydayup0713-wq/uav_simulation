"""Normalize posterior RTK artifacts; continuous local estimates remain untouched."""
import numpy as np
from scipy.spatial.transform import Rotation
from .trajectory_evaluation import validate_trajectory,associate


def normalize_posterior(local,antenna,calibration):
    local=validate_trajectory(local);antenna=validate_trajectory(antenna)
    supported=antenna[(antenna[:,0]>=local[0,0])&(antenna[:,0]<=local[-1,0])]
    if len(supported)<3:raise ValueError('posterior/local source-time association unavailable')
    reference=associate(supported,local)
    body=supported.copy()
    body_rotation=Rotation.from_quat(body[:,4:])*Rotation.from_euler('xyz',calibration['imu']['rpy']).inv()
    body[:,1:4]-=body_rotation.apply(calibration['gnss']['xyz'])
    body[:,4:]=body_rotation.as_quat()
    map_pose=np.eye(4);local_pose=np.eye(4)
    map_pose[:3,:3]=body_rotation[-1].as_matrix();map_pose[:3,3]=body[-1,1:4]
    local_pose[:3,:3]=Rotation.from_quat(reference[-1,4:]).as_matrix();local_pose[:3,3]=reference[-1,1:4]
    return {'body_trajectory':body,'map_to_local':map_pose@np.linalg.inv(local_pose),
            'source_time_s':float(body[-1,0]),'outside_local_support':len(antenna)-len(supported)}
