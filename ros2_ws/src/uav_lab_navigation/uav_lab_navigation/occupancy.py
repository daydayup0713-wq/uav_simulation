"""Observed voxel evidence, conservative body inflation and segment supercover."""
from dataclasses import dataclass
from itertools import combinations, product
import math
import numpy as np
from scipy.ndimage import maximum_filter

UNKNOWN, FREE, OCCUPIED = 0, -1, 1


def vector(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('finite three-dimensional coordinate required')
    return result


def supercover(a, b):
    """Yield all unit cells touched by a closed line, including edge/corner ties."""
    a, b = vector(a), vector(b)
    cell, end = np.floor(a).astype(int), np.floor(b).astype(int)
    delta = b-a; step = np.sign(delta).astype(int)
    zero = [i for i in range(3) if delta[i] == 0 and abs(a[i]-round(a[i])) < 1e-10]
    def emit(c):
        for shifts in product((0,-1), repeat=len(zero)):
            item = c.copy()
            for axis, shift in zip(zero, shifts): item[axis] += shift
            yield tuple(item)
    yield from emit(cell)
    tmax, tdelta = np.full(3,np.inf), np.full(3,np.inf)
    for i in range(3):
        if step[i]:
            boundary = cell[i]+(1 if step[i]>0 else 0)
            tmax[i], tdelta[i] = (boundary-a[i])/delta[i], 1/abs(delta[i])
    for _ in range(int(np.abs(end-cell).sum())+4):
        t = float(tmax.min())
        if t > 1+1e-10 or not math.isfinite(t): break
        axes = np.flatnonzero(np.abs(tmax-t)<1e-10)
        for n in range(1,len(axes)+1):
            for subset in combinations(axes,n):
                neighbor = cell.copy()
                for axis in subset: neighbor[axis] += step[axis]
                yield from emit(neighbor)
        cell[axes] += step[axes]; tmax[axes] += tdelta[axes]


@dataclass(frozen=True)
class CollisionMap:
    resolution: float
    lower: np.ndarray
    free: np.ndarray
    version: int

    def index(self, point):
        return tuple(np.floor((vector(point)-self.lower)/self.resolution).astype(int))

    def cell_clear(self, index):
        return all(0<=v<s for v,s in zip(index,self.free.shape)) and bool(self.free[index])

    def point_clear(self, point):
        return self.cell_clear(self.index(point))

    def center(self, index):
        return self.lower+(np.asarray(index)+.5)*self.resolution

    def segment_clear(self, a, b):
        a, b = ((vector(v)-self.lower)/self.resolution for v in (a,b))
        return all(self.cell_clear(cell) for cell in supercover(a,b))


class VoxelMap:
    def __init__(self, resolution=.2, lower=(-6,-6,-1), upper=(6,6,5)):
        self.resolution = float(resolution); self.lower, self.upper = vector(lower), vector(upper)
        if not math.isfinite(self.resolution) or self.resolution<=0 or np.any(self.upper<=self.lower):
            raise ValueError('positive resolution and ordered bounds required')
        size = (self.upper-self.lower)/self.resolution
        if not np.allclose(size,np.round(size)) or np.prod(size)>2_000_000:
            raise ValueError('integral bounded grid up to 2000000 cells required')
        self.shape = tuple(np.round(size).astype(int))
        self.score = np.zeros(self.shape,dtype=np.int8)
        self.seen = np.zeros(self.shape,dtype=bool)
        self.version = 0

    def inside(self, points):
        return np.all((points>=self.lower)&(points<self.upper),axis=-1)

    def index(self, points):
        return np.floor((np.asarray(points)-self.lower)/self.resolution).astype(int)

    def state(self, point):
        point = vector(point)
        if not self.inside(point): return UNKNOWN
        index = tuple(self.index(point))
        return OCCUPIED if self.score[index]>0 else FREE if self.score[index]<0 else UNKNOWN

    def integrate(self, origin, endpoints):
        origin = vector(origin); endpoints = np.asarray(endpoints,dtype=float)
        if endpoints.ndim!=2 or endpoints.shape[1]!=3 or not np.isfinite(endpoints).all():
            raise ValueError('finite Nx3 endpoints required')
        if not self.inside(origin): raise ValueError('sensor origin outside mapping bounds')
        if len(endpoints)>50000: raise ValueError('scan exceeds bounded input')
        # Reduce duplicate endpoint voxels before vectorized sub-voxel ray sampling.
        _, unique = np.unique(self.index(endpoints),axis=0,return_index=True)
        endpoints = endpoints[unique]
        hit_points = endpoints[self.inside(endpoints)]
        hits = np.ravel_multi_index(self.index(hit_points).T,self.shape) if len(hit_points) else np.array([],dtype=int)
        delta = endpoints-origin
        fractions = np.ones(len(delta))
        for axis in range(3):
            moving = np.abs(delta[:,axis])>1e-12
            edge = np.where(delta[:,axis]>0,self.upper[axis]-1e-9,self.lower[axis]+1e-9)
            values = np.ones(len(delta)); values[moving] = (edge[moving]-origin[axis])/delta[moving,axis]
            fractions = np.minimum(fractions,values)
        delta *= np.clip(fractions,0,1)[:,None]
        free_indices = []
        for start in range(0,len(delta),256):
            rays = delta[start:start+256]
            steps = np.maximum(1,np.ceil(np.linalg.norm(rays,axis=1)/(self.resolution*.25)).astype(int))
            times = np.arange(steps.max()+1)[None,:]/steps[:,None]
            valid = times<=1
            points = (origin+rays[:,None,:]*times[:,:,None])[valid]
            points = points[self.inside(points)]
            if len(points): free_indices.append(np.ravel_multi_index(self.index(points).T,self.shape))
        free = np.unique(np.concatenate(free_indices)) if free_indices else np.array([],dtype=int)
        free = np.setdiff1d(free,hits,assume_unique=True)
        scores, seen = self.score.ravel(), self.seen.ravel()
        scores[free] = np.maximum(-4,scores[free].astype(int)-1)
        scores[hits] = np.minimum(4,scores[hits].astype(int)+2)
        seen[free], seen[hits] = True, True
        self.version += 1

    def observe_body(self, position, halfsize):
        position, halfsize = vector(position), vector(halfsize)
        if np.any(halfsize<0): raise ValueError('nonnegative body dimensions required')
        if not self.inside(position): raise ValueError('body outside mapping bounds')
        lo = np.maximum(0,self.index(position-halfsize)); hi = np.minimum(np.asarray(self.shape)-1,self.index(position+halfsize))
        area = tuple(slice(a,b+1) for a,b in zip(lo,hi))
        values = self.score[area]; observed = self.seen[area]
        values[~observed] = -1; observed[:] = True
        self.version += 1

    def snapshot(self, halfsize):
        halfsize = vector(halfsize)
        if np.any(halfsize<0): raise ValueError('nonnegative collision envelope required')
        radii = np.ceil(halfsize/self.resolution-1e-12).astype(int)
        blocked = maximum_filter(self.score>=0,size=tuple(2*radii+1),mode='constant',cval=1)
        free = ~blocked
        free.setflags(write=False)
        return CollisionMap(self.resolution,self.lower.copy(),free,self.version)

    def occupied_points(self):
        return self.lower+(np.argwhere(self.score>0)+.5)*self.resolution
