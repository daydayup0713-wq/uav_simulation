"""Fixed-size rolling evidence and per-beam pose registration; no truth input."""
import math
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from .occupancy import VoxelMap,vector


class RollingMap(VoxelMap):
    def recenter(self,position,threshold=2.):
        position=vector(position)
        if not math.isfinite(threshold) or threshold<=0:raise ValueError('positive recenter threshold required')
        delta=position-(self.lower+self.upper)/2
        shift=np.where(np.abs(delta)>=threshold,np.round(delta/self.resolution),0).astype(int)
        if not shift.any():return False
        score=np.zeros_like(self.score);seen=np.zeros_like(self.seen)
        if np.all(np.abs(shift)<self.shape):
            old=tuple(slice(max(0,d),min(n,n+d)) for n,d in zip(self.shape,shift))
            new=tuple(slice(max(0,-d),min(n,n-d)) for n,d in zip(self.shape,shift))
            score[new]=self.score[old];seen[new]=self.seen[old]
        self.score,self.seen=score,seen
        self.lower=self.lower+shift*self.resolution;self.upper=self.upper+shift*self.resolution
        self.version+=1
        return True

    def mark_source(self,stamp):
        if not math.isfinite(stamp) or stamp<0:raise ValueError('finite nonnegative map source required')
        if hasattr(self,'source_stamp') and stamp<=self.source_stamp:raise ValueError('non-increasing map source')
        self.source_stamp=stamp

    def integrate_beams(self,origins,endpoints):
        return self._integrate_beams(origins,endpoints)


def register_beams(points,times,history,alignment,extrinsic):
    points=np.asarray(points,dtype=float);times=np.asarray(times,dtype=float)
    alignment,extrinsic=map(lambda value:np.asarray(value,dtype=float),(alignment,extrinsic))
    if (points.ndim!=2 or points.shape[1]!=3 or times.shape!=(len(points),) or not 0<len(points)<=50000
            or not np.isfinite(points).all() or not np.isfinite(times).all()
            or any(m.shape!=(4,4) or not np.isfinite(m).all() for m in (alignment,extrinsic))):
        raise ValueError('bounded measured beams and rigid transforms required')
    samples=list(history.samples)
    if not samples or times.min()<samples[0][0] or times.max()>samples[-1][0]:
        raise ValueError('beam times not bracketed by source odometry')
    timestamps=np.array([s[0] for s in samples]);positions=np.array([s[1] for s in samples])
    unique,inverse=np.unique(times,return_inverse=True)
    if len(samples)==1:
        translations=np.repeat(positions, len(unique),axis=0)
        rotations=Rotation.from_quat([samples[0][2]]*len(unique)).as_matrix()
    else:
        hi=np.clip(np.searchsorted(timestamps,unique,side='right'),1,len(timestamps)-1);lo=hi-1
        gaps=timestamps[hi]-timestamps[lo]
        if np.any((gaps>.3)&(unique!=timestamps[lo])&(unique!=timestamps[hi])):
            raise ValueError('source odometry interpolation gap')
        fractions=(unique-timestamps[lo])/gaps
        translations=positions[lo]*(1-fractions[:,None])+positions[hi]*fractions[:,None]
        rotations=Slerp(timestamps,Rotation.from_quat([s[2] for s in samples]))(unique).as_matrix()
    odom_from_local=np.linalg.inv(alignment)
    rotations=np.einsum('ij,njk->nik',odom_from_local[:3,:3],rotations)[inverse]
    translations=(translations@odom_from_local[:3,:3].T+odom_from_local[:3,3])[inverse]
    sensor=points@extrinsic[:3,:3].T+extrinsic[:3,3]
    origins=translations+np.einsum('nij,j->ni',rotations,extrinsic[:3,3])
    endpoints=translations+np.einsum('nij,nj->ni',rotations,sensor)
    return origins,endpoints
