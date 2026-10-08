"""Typed ROS boundary for normalized-time polynomial trajectories."""
import numpy as np
from uav_lab_interfaces.action import ExecuteTrajectory
from uav_lab_interfaces.msg import TrajectorySegment
from .continuous_trajectory import Trajectory, Segment, Limits


def trajectory_goal(trajectory, identifier, starts_at_ns=0, replaces='', epoch=None):
    request = ExecuteTrajectory.Goal()
    request.header.frame_id = 'odom'
    request.header.stamp.sec, request.header.stamp.nanosec = divmod(int(starts_at_ns), 10**9)
    request.trajectory_id, request.replaces_id = identifier, replaces
    request.navigation = epoch is not None
    request.navigation_epoch = epoch if epoch is not None else 0
    request.segments = [TrajectorySegment(duration=segment.duration,
                        coefficients=segment.coefficients.ravel().tolist(),
                        yaw_coefficients=segment.yaw_coefficients.tolist()) for segment in trajectory.segments]
    return request


def trajectory_from_goal(request):
    # Client never supplies the execution limits: fixed platform defaults enforce them.
    if request.header.frame_id != 'odom' or not 1 <= len(request.segments) <= 128:
        raise ValueError('odom frame and one to 128 segments required')
    payload = {'schema': 1, 'frame': 'odom', 'limits': {'speed': .5, 'acceleration': .5, 'jerk': 1.},
               'segments': [{'duration': segment.duration,
                    'coefficients': np.asarray(segment.coefficients).reshape(3, 6).tolist(),
                    'yaw_coefficients': list(segment.yaw_coefficients)} for segment in request.segments]}
    return Trajectory.from_dict(payload)
