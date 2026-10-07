"""Strict continuous LIO observation conversion at the sole PX4 input boundary."""
import numpy as np
from scipy.spatial.transform import Rotation

from .coordinate import enu_to_ned, ros_to_px4_quaternion


def convert_external(sample, clock, now_ros_ns):
    if sample['frame'] != 'lio_odom' or sample['child_frame'] != 'lio_base_link':
        raise ValueError('external odometry must be continuous lio_odom/FLU, never map')
    age = (now_ros_ns-int(sample['source_ns']))/1e9
    if not 0 <= age <= .5:
        raise ValueError('external odometry source stale or from future')
    p, q, v, w = [np.asarray(sample[k], dtype=float) for k in
                   ('position', 'quaternion', 'body_velocity', 'body_angular_velocity')]
    if p.shape != (3,) or q.shape != (4,) or v.shape != (3,) or w.shape != (3,) or not all(np.isfinite(x).all() for x in (p,q,v,w)):
        raise ValueError('invalid external odometry values')
    if abs(np.linalg.norm(q)-1) > 1e-3:
        raise ValueError('invalid external quaternion')
    covariances = []
    for key in ('pose_covariance', 'twist_covariance'):
        c = np.asarray(sample[key]).reshape(6, 6)
        if not np.isfinite(c).all() or not np.allclose(c, c.T) or np.min(np.linalg.eigvalsh(c)) <= 0:
            raise ValueError('external odometry requires positive covariance/noise floors')
        covariances.append(c)
    pc, vc = covariances
    rotation = Rotation.from_quat(q).as_matrix()
    orientation = np.diag(rotation.T @ pc[3:, 3:] @ rotation)
    return {'timestamp': clock.timestamp(now_ros_ns), 'timestamp_sample': clock.sample_timestamp(sample['source_ns']),
            'pose_frame': 2, 'velocity_frame': 3,
            'position': list(enu_to_ned(p)), 'q': list(ros_to_px4_quaternion(q)),
            'velocity': [float(v[0]), -float(v[1]), -float(v[2])],
            'angular_velocity': [float(w[0]), -float(w[1]), -float(w[2])],
            'position_variance': [float(pc[1,1]), float(pc[0,0]), float(pc[2,2])],
            'orientation_variance': orientation.tolist(), 'velocity_variance': np.diag(vc[:3,:3]).tolist(),
            'reset_counter': 0, 'quality': 100}


class ExternalOdometryGate:
    def __init__(self):
        self.quality_at = None
        self.sample_at = None
        self.sample = None
        self.was_ready = False
        self.failed = ''

    def observe_quality(self, ready, now):
        if self.was_ready and not ready:
            self.failed = self.failed or 'localization quality failed'
        self.quality_at = now if ready else None

    def observe_sample(self, sample, now):
        if self.sample and sample['source_ns'] <= self.sample['source_ns']:
            self.failed = self.failed or 'external odometry source clock regression'
        self.sample, self.sample_at = sample, now

    def ready(self, now):
        fresh = (self.quality_at is not None and self.sample_at is not None and
                 now-self.quality_at <= .6 and now-self.sample_at <= .8)
        if self.was_ready and not fresh:
            self.failed = self.failed or 'external localization lost; restart required'
        ready = fresh and not self.failed
        self.was_ready |= ready
        return bool(ready)
