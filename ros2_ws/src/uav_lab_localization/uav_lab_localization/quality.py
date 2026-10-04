"""Source-time validation and a monotonic, latched localization watchdog."""
import numpy as np
from scipy.spatial.transform import Rotation


def normalize_pose(position, quaternion, world_velocity):
    p, q, v = map(lambda x: np.asarray(x, dtype=float), (position, quaternion, world_velocity))
    if p.shape != (3,) or q.shape != (4,) or v.shape != (3,) or not all(np.isfinite(a).all() for a in (p, q, v)):
        raise ValueError('non-finite localization pose/velocity')
    if not abs(np.linalg.norm(q)-1) < 1e-3:
        raise ValueError('invalid localization quaternion')
    return {'position': p, 'quaternion': q, 'body_velocity': Rotation.from_quat(q).inv().apply(v)}


class LocalizationQuality:
    def __init__(self, warmup_samples=20):
        self.warmup_samples = warmup_samples
        self.sources = {}
        self.pose = None
        self.samples = 0
        self.reason = ''
        self.was_ready = False

    def fail(self, reason):
        if not self.reason:
            self.reason = reason
        return False

    def observe_source(self, kind, stamp, now):
        if kind not in ('imu', 'points') or not np.isfinite(stamp) or stamp <= 0:
            return self.fail('invalid source timestamp')
        previous = self.sources.get(kind)
        if previous and stamp <= previous[0]:
            return self.fail(kind+' source clock regression')
        self.sources[kind] = (stamp, now)
        return not self.reason

    def observe_pose(self, stamp, position, quaternion, now):
        if self.reason:
            return False
        try:
            normalized = normalize_pose(position, quaternion, [0, 0, 0])
        except ValueError as error:
            return self.fail(str(error))
        if not np.isfinite(stamp) or stamp <= 0:
            return self.fail('invalid odometry timestamp')
        if self.pose:
            old_stamp, _, old_p, old_q = self.pose
            dt = stamp-old_stamp
            if dt <= 0:
                return self.fail('odometry clock regression')
            if np.linalg.norm(normalized['position']-old_p) > max(0.15, dt*3):
                return self.fail('continuous odometry position jump')
            angle = (Rotation.from_quat(old_q).inv()*Rotation.from_quat(quaternion)).magnitude()
            if angle > max(0.15, dt*3):
                return self.fail('continuous odometry orientation jump')
        self.pose = (stamp, now, normalized['position'], normalized['quaternion'])
        self.samples += 1
        return True

    def check(self, clock, now):
        fresh = (self.pose is not None and now-self.pose[1] <= 1.5 and
                 -0.15 <= clock-self.pose[0] <= 0.5 and
                 all(k in self.sources and now-self.sources[k][1] <= 1.5 and
                     -0.15 <= clock-self.sources[k][0] <= limit
                     for k, limit in (('imu', 0.25), ('points', 0.4))))
        if self.was_ready and not fresh:
            self.fail('localization source/output stale or simulation paused')
        ready = fresh and self.samples >= self.warmup_samples and not self.reason
        self.was_ready |= ready
        return {'ready': bool(ready), 'state': 'FAILED' if self.reason else 'READY' if ready else 'INITIALIZING',
                'reason': self.reason, 'samples': self.samples,
                'source_age_s': None if not self.pose else clock-self.pose[0]}
