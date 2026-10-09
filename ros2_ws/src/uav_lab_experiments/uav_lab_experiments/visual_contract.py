"""Metric eligibility is separate from visually tracking an arbitrary scale."""


def metric_tracking_ready(backend, state, inertial_initialized):
    if backend == 'orb_slam3':
        return state == 2 and bool(inertial_initialized)
    if backend == 'vins_fusion':
        return state == 1 and bool(inertial_initialized)
    raise ValueError('unknown visual backend')
