"""Observation boundary shared by live LIO and immutable dataset replay."""
import numpy as np


class RejectedScan(ValueError):
    """Unavailable geometry; drop this scan and let the freshness watchdog decide."""


def cloud_xyz(message):
    fields = {f.name: f for f in message.fields}
    if any(n not in fields or fields[n].datatype != 7 or fields[n].count != 1 for n in 'xyz'):
        raise ValueError('expected FLOAT32 xyz point fields')
    if message.row_step < message.width * message.point_step or len(message.data) != message.height * message.row_step:
        raise ValueError('invalid point cloud layout')
    dtype = np.dtype({'names': list('xyz'), 'formats': [('>' if message.is_bigendian else '<')+'f4']*3,
                      'offsets': [fields[n].offset for n in 'xyz'], 'itemsize': message.point_step})
    values = np.ndarray((message.height, message.width), dtype=dtype, buffer=bytes(message.data),
                        strides=(message.row_step, message.point_step))
    points = np.column_stack([values[n].ravel() for n in 'xyz']).astype(float)
    return points[np.isfinite(points).all(axis=1)]


def validated_scan(message):
    if message.header.frame_id != 'lidar_link':
        raise ValueError('unexpected lidar frame')
    points = cloud_xyz(message)
    distance = np.linalg.norm(points, axis=1)
    points = points[(distance >= .35) & (distance <= 20)]
    if len(points) < 500 or len(np.unique(np.floor(points/.15), axis=0)) < 100:
        raise RejectedScan('insufficient distinct in-range lidar observations')
    return points


def validate_imu(message):
    values = [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
              message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z]
    if message.header.frame_id != 'imu_link' or message.orientation_covariance[0] != -1 or not np.isfinite(values).all():
        raise ValueError('invalid IMU contract')
