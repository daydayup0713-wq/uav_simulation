"""Bounded source-current observed maps for navigation braking admission."""
import math
from types import SimpleNamespace
import numpy as np


class StopAdmission:
    def __init__(self, required_envelope):
        self.required_envelope=np.asarray(required_envelope,dtype=float)
        if self.required_envelope.shape!=(3,) or not np.isfinite(self.required_envelope).all() or np.any(self.required_envelope<=0):
            raise ValueError('positive finite body envelope required')
        self.snapshot=None;self.received=-math.inf

    def observe(self, value, now):
        shape=np.asarray(value['shape']);lower=np.asarray(value['lower'],dtype=float)
        envelope=np.asarray(value['envelope'],dtype=float)
        resolution=float(value['resolution']);source=float(value['source_stamp']);version=value['version']
        if (value['frame']!='odom' or shape.shape!=(3,) or not np.issubdtype(shape.dtype,np.integer)
            or np.any(shape<=0) or np.any(shape>2000) or math.prod(int(x) for x in shape)>2000000
            or lower.shape!=(3,) or not np.isfinite(lower).all() or np.any(np.abs(lower)>1000)
            or envelope.shape!=(3,) or not np.isfinite(envelope).all() or np.any(envelope+1e-9<self.required_envelope)
            or not math.isfinite(resolution) or not .05<=resolution<=1.
            or not math.isfinite(source) or source<0 or not isinstance(version,int) or isinstance(version,bool) or version<0):
            raise ValueError('invalid bounded inflated stop map')
        if self.snapshot is not None and (version<self.snapshot.version or source<self.snapshot.source_stamp):return False
        data=value['free']
        if len(data)!=math.prod(int(x) for x in shape):raise ValueError('stop map cell count mismatch')
        cells=np.frombuffer(bytes(data),dtype=np.uint8)
        if np.any(cells>1):raise ValueError('stop map must explicitly encode blocked/unknown as zero')
        free=cells.astype(bool).reshape(tuple(shape));free.setflags(write=False)
        self.snapshot=SimpleNamespace(lower=lower.copy(),resolution=resolution,free=free,
            envelope=envelope.copy(),version=version,source_stamp=source)
        self.received=now;return True

    def admit(self, trajectory, now, sim):
        if self.snapshot is None:raise ValueError('stop map unavailable')
        if now-self.received>.75 or not -.1<=sim-self.snapshot.source_stamp<=1.:
            raise ValueError('stop map expired or invalid source time')
        if not trajectory.collision_free(self.snapshot):raise ValueError('stop curve blocked or unknown')
        return True
