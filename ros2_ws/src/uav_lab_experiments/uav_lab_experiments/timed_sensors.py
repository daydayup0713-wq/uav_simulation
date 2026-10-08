"""Measurement-time ray acquisition for static box scenes.

Physics poses are private inputs to the simulator, never estimator inputs. The
200 Hz pose interpolation approximates motion between physics samples. Livox
directions approximate coverage; they do not reproduce proprietary optics.
"""
from collections import deque
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class PoseBuffer:
    def __init__(self, capacity=4096):
        self.samples = deque(maxlen=capacity)

    def add(self, time, position, quaternion):
        values = np.r_[time, position, quaternion]
        if not np.isfinite(values).all() or abs(np.linalg.norm(quaternion)-1) > .001:
            raise ValueError('finite pose and unit quaternion required')
        if self.samples and time <= self.samples[-1][0]:
            raise ValueError('pose timestamps must be monotonic')
        self.samples.append(values)

    def at_many(self, times):
        data = np.asarray(self.samples)
        times = np.asarray(times)
        if len(data) < 2 or not np.isfinite(times).all() or times.min() < data[0, 0] or times.max() > data[-1, 0]:
            raise ValueError('measurement times require bracketing physics poses')
        indices = np.clip(np.searchsorted(data[:, 0], times), 1, len(data)-1)
        gaps = data[indices, 0] - data[indices-1, 0]
        if np.max(gaps) > .11:
            raise ValueError('physics pose bracket gap exceeds 110ms')
        position = np.column_stack([np.interp(times, data[:, 0], data[:, axis]) for axis in (1, 2, 3)])
        rotation = Slerp(data[:, 0], Rotation.from_quat(data[:, 4:]))(times).as_matrix()
        return position, rotation


def measure_rays(poses, times, directions, boxes, extrinsic, minimum, maximum, noise_stddev, rng):
    """Return raw, uncorrected XYZ in the sensor frame at each beam time."""
    position, rotation = poses.at_many(times)
    directions = np.asarray(directions, dtype=float)
    sensor_rotation = rotation @ np.asarray(extrinsic)[:3, :3]
    origin = position + np.einsum('nij,j->ni', rotation, np.asarray(extrinsic)[:3, 3])
    world_directions = np.einsum('nij,nj->ni', sensor_rotation, directions)
    ranges = np.full(len(times), np.inf)
    # Slab intersection. Handle parallel rays explicitly, including origins on
    # a slab boundary, rather than relying on zero multiplied by infinity.
    parallel = np.abs(world_directions) < 1e-12
    for lower, upper in np.asarray(boxes):
        with np.errstate(divide='ignore', invalid='ignore'):
            a = (lower-origin)/world_directions
            b = (upper-origin)/world_directions
        near = np.where(parallel, -np.inf, np.minimum(a, b))
        far = np.where(parallel, np.inf, np.maximum(a, b))
        entry, exit = near.max(axis=1), far.min(axis=1)
        invalid = np.any(parallel & ((origin < lower) | (origin > upper)), axis=1)
        hit = np.where(entry >= minimum, entry, exit)
        hit = np.where(~invalid & (exit >= np.maximum(entry, minimum)) & (hit <= maximum), hit, np.inf)
        ranges = np.minimum(ranges, hit)
    valid = np.isfinite(ranges)
    ranges[valid] += rng.normal(0, noise_stddev, valid.sum())
    valid &= (ranges >= minimum) & (ranges <= maximum)
    xyz = directions*ranges[:, None]
    xyz[~valid] = np.nan
    return xyz


def scan_pattern(config, start):
    columns, channels = int(config['horizontal_samples']), int(config['vertical_samples'])
    count = columns*channels
    offsets = np.arange(count, dtype=float)/(count*config['hz'])
    line = np.tile(np.arange(channels, dtype=np.uint16), columns)
    if config['kind'] == 'mechanical':
        azimuth = np.repeat(np.arange(columns)*2*np.pi/columns, channels)
        elevation = np.linspace(-config['vertical_fov_rad']/2, config['vertical_fov_rad']/2, channels)[line]
    elif config['kind'] == 'livox':
        if channels != 4:
            raise ValueError('Livox approximation uses four declared optical channels')
        time = start+offsets
        azimuth = (2*np.pi*7.231*time + .23*np.sin(2*np.pi*17.719*time + line*np.pi/2)) % (2*np.pi)
        elevation = np.deg2rad(22.5 + 29.5*np.sin(2*np.pi*13.137*time + line*np.pi/2))
    else:
        raise ValueError('unknown scanner pattern')
    directions = np.column_stack([np.cos(elevation)*np.cos(azimuth), np.cos(elevation)*np.sin(azimuth), np.sin(elevation)])
    return start+offsets, directions, line


def point_records(xyz, relative_times, channels, kind):
    channel_name = 'ring' if kind == 'mechanical' else 'line'
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                      ('time', '<f4'), (channel_name, '<u2'), ('padding', '<u2')])
    result = np.zeros(len(xyz), dtype=dtype)
    for axis, name in enumerate('xyz'):
        result[name] = xyz[:, axis]
    # Deterministic range-dependent synthetic reflectivity, not a material model.
    result['intensity'] = np.clip(100/(1+.05*np.linalg.norm(xyz, axis=1)), 0, 100)
    result['time'], result[channel_name] = relative_times, channels
    return result


def noisy_attitude(quaternion, stddev, rng):
    if stddev <= 0:
        raise ValueError('attitude noise must be positive')
    result = (Rotation.from_quat(quaternion)*Rotation.from_rotvec(rng.normal(0, stddev, 3))).as_quat()
    return result, np.eye(3)*stddev**2


def rtk_measurement(config, source_time, position, rng):
    mode, event = 'fixed', {}
    for candidate in config['events']:
        if candidate['start_s'] <= source_time < candidate['end_s']:
            mode, event = candidate['mode'], candidate
            break
    if mode == 'lost':
        return {'stamp': source_time, 'mode': mode, 'status': -1, 'lla': [math.nan]*3,
                'enu': [math.nan]*3, 'covariance': [0.]*9}
    stddev = config.get('float_stddev_m', .5) if mode == 'float' else config['fixed_stddev_m']
    enu = np.asarray(position)+rng.normal(0, stddev, 3)+np.asarray(event.get('offset_m', [0, 0, 0]))
    lat, lon, height = config['origin']
    # Local WGS84 curvature, sufficient for the declared <=1km lab extent.
    radius, eccentricity = 6378137., .00669437999014
    latitude = math.radians(lat)
    prime = radius/math.sqrt(1-eccentricity*math.sin(latitude)**2)
    meridian = radius*(1-eccentricity)/(1-eccentricity*math.sin(latitude)**2)**1.5
    lla = [lat+math.degrees(enu[1]/(meridian+height)),
           lon+math.degrees(enu[0]/((prime+height)*math.cos(latitude))), height+enu[2]]
    return {'stamp': source_time, 'mode': mode, 'status': 0 if mode == 'float' else 2,
            'lla': lla, 'enu': enu.tolist(), 'covariance': (np.eye(3)*stddev**2).ravel().tolist()}
