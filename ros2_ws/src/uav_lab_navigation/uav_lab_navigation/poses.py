"""Bounded source-time pose interpolation, without extrapolating sensor position."""
from collections import deque
import math
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from .occupancy import vector


class PoseHistory:
    def __init__(self): self.samples=deque(maxlen=250)

    def add(self,stamp,position,quaternion):
        position=vector(position);quaternion=np.asarray(quaternion,dtype=float)
        if (not math.isfinite(stamp) or stamp<0 or quaternion.shape!=(4,)
                or not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion)-1)>.001):
            raise ValueError('invalid source pose')
        if self.samples and stamp<=self.samples[-1][0]:
            raise ValueError('non-increasing pose source time')
        self.samples.append((stamp,position,quaternion))

    def at(self,stamp):
        if not self.samples or not self.samples[0][0]<=stamp<=self.samples[-1][0]:
            raise ValueError('sensor time not bracketed by odometry')
        for i,(t,p,q) in enumerate(self.samples):
            if t==stamp:
                pose=np.eye(4);pose[:3,3]=p;pose[:3,:3]=Rotation.from_quat(q).as_matrix();return pose
            if t>stamp:
                previous=self.samples[i-1];fraction=(stamp-previous[0])/(t-previous[0])
                if t-previous[0]>.3: raise ValueError('odometry interpolation gap')
                pose=np.eye(4);pose[:3,3]=previous[1]+fraction*(p-previous[1])
                pose[:3,:3]=Slerp([previous[0],t],Rotation.from_quat([previous[2],q]))([stamp]).as_matrix()[0]
                return pose
        raise ValueError('sensor time not bracketed')


def register_scan(points,body,alignment,extrinsic):
    matrices=[np.asarray(m,dtype=float) for m in (body,alignment,extrinsic)]
    if any(m.shape!=(4,4) or not np.isfinite(m).all() for m in matrices): raise ValueError('invalid registration transform')
    transform=np.linalg.inv(matrices[1])@matrices[0]@matrices[2]
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():raise ValueError('invalid point coordinates')
    return transform[:3,3].copy(),points@transform[:3,:3].T+transform[:3,3]


def remove_self_returns(points,extrinsic,halfsize):
    points=np.asarray(points,dtype=float);halfsize=vector(halfsize)
    matrix=np.asarray(extrinsic,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all() or matrix.shape!=(4,4) or not np.isfinite(matrix).all() or np.any(halfsize<0):
        raise ValueError('invalid self-return filter geometry')
    body_points=points@matrix[:3,:3].T+matrix[:3,3]
    return points[~np.all(np.abs(body_points)<=halfsize,axis=1)]
