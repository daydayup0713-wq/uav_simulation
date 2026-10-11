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
        origins=np.asarray(origins,dtype=float);endpoints=np.asarray(endpoints,dtype=float)
        if (origins.ndim!=2 or origins.shape[1]!=3 or endpoints.shape!=origins.shape
                or len(origins)>50000 or not np.isfinite(origins).all() or not np.isfinite(endpoints).all()
                or not self.inside(origins).all()):
            raise ValueError('bounded finite measured beam origins/endpoints within local window required')
        if not len(origins):raise ValueError('measured beam returns missing')
        _,unique=np.unique(np.column_stack([self.index(origins),self.index(endpoints)]),axis=0,return_index=True)
        origins,endpoints=origins[unique],endpoints[unique]
        hit_points=endpoints[self.inside(endpoints)]
        hits=np.ravel_multi_index(self.index(hit_points).T,self.shape) if len(hit_points) else np.array([],dtype=int)
        delta=endpoints-origins;fractions=np.ones(len(delta))
        for axis in range(3):
            moving=np.abs(delta[:,axis])>1e-12
            edge=np.where(delta[:,axis]>0,self.upper[axis]-1e-9,self.lower[axis]+1e-9)
            values=np.ones(len(delta));values[moving]=(edge[moving]-origins[moving,axis])/delta[moving,axis]
            fractions=np.minimum(fractions,values)
        delta*=np.clip(fractions,0,1)[:,None]
        free_mask=np.zeros(self.score.size,dtype=bool)
        for start in range(0,len(delta),256):
            rays=delta[start:start+256]
            steps=np.maximum(1,np.ceil(np.linalg.norm(rays,axis=1)/(self.resolution*.5)).astype(int))
            times=np.arange(steps.max()+1)[None,:]/steps[:,None]
            points=(origins[start:start+256,None,:]+rays[:,None,:]*times[:,:,None])[times<=1]
            points=points[self.inside(points)]
            if len(points):free_mask[np.ravel_multi_index(self.index(points).T,self.shape)]=True
        free_mask[hits]=False;free=np.flatnonzero(free_mask)
        scores,seen=self.score.ravel(),self.seen.ravel()
        scores[free]=np.maximum(-4,scores[free].astype(int)-1);scores[hits]=np.minimum(4,scores[hits].astype(int)+2)
        seen[free]=True;seen[hits]=True;self.version+=1


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
