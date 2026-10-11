"""Independent observed continuous-flight metrics; never used for control."""
import numpy as np


def evaluate_continuous(references, actual, heartbeats, setpoints):
    errors, derivatives, windows = [], [], []
    monotonic = True
    for identifier in sorted({r['id'] for r in references}):
        frames = [r for r in references if r['id'] == identifier]
        times = np.array([r['stamp'] for r in frames])
        points = np.array([r['position'] for r in frames])
        if not np.isfinite(times).all() or not np.isfinite(points).all() or np.any(np.diff(times) < 0):
            monotonic = False
            continue
        windows.append((frames[0]['wall'], frames[-1]['wall']))
        for sample in actual:
            if times[0] <= sample['stamp'] <= times[-1]:
                expected = np.array([np.interp(sample['stamp'], times, points[:, i]) for i in range(3)])
                errors.append(float(np.linalg.norm(np.array(sample['position']) - expected)))
        for a, b in zip(frames, frames[1:]):
            dt = b['stamp'] - a['stamp']
            if 0 < dt < .1:
                derivatives.append(float(np.linalg.norm(np.array(b['acceleration']) - a['acceleration']) / dt))
    duration = sum(max(0., upper - lower) for lower, upper in windows)
    def rate(values):
        count = sum(any(lower <= value <= upper for lower, upper in windows) for value in values)
        return count / duration if duration else 0.
    ref_rate = len(references) / duration if duration else 0.
    p95 = float(np.percentile(errors, 95)) if errors else None
    speed = max((float(np.linalg.norm(r['velocity'])) for r in references), default=0.)
    acceleration = max((float(np.linalg.norm(r['acceleration'])) for r in references), default=0.)
    jerk = max(derivatives, default=0.)
    heartbeat_rate, setpoint_rate = rate(heartbeats), rate(setpoints)
    checks = {'samples': len(errors) >= 20, 'tracking': p95 is not None and p95 <= .3,
              'reference': 40 <= ref_rate <= 65, 'heartbeat': 16 <= heartbeat_rate <= 24,
              'setpoints': 40 <= setpoint_rate <= 65, 'speed': speed <= .50001,
              'acceleration': acceleration <= .50001, 'jerk': jerk <= 1.01,
              'reference_time': monotonic}
    return {'passed': all(checks.values()), 'failed_checks': [k for k, passed in checks.items() if not passed],
        'tracking_p95_m': p95, 'tracking_maximum_m': max(errors) if errors else None,
        'tracking_samples': len(errors), 'reference_samples': len(references), 'reference_rate_hz': ref_rate,
        'reference_max_speed_mps': speed, 'reference_max_acceleration_mps2': acceleration,
        'reference_max_finite_difference_jerk_mps3': jerk, 'heartbeat_samples': len(heartbeats),
        'setpoint_samples': len(setpoints), 'heartbeat_rate_hz': heartbeat_rate,
        'setpoint_rate_hz': setpoint_rate, 'rate_scope': 'only within each trajectory observation window',
        'tracking_source': 'PX4 normalized odometry versus time-aligned commanded reference'}
