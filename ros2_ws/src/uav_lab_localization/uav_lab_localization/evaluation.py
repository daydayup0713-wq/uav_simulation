"""Independent timestamp-associated trajectory evaluation, without scale fitting."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def validate_trajectory(trajectory):
    data = np.asarray(trajectory, dtype=float)
    if data.ndim != 2 or data.shape[1] != 8 or len(data) < 3 or not np.isfinite(data).all():
        raise ValueError('trajectory must contain finite TUM rows')
    if data[0, 0] <= 0 or np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError('trajectory timestamps must be positive and strictly increasing')
    if not np.allclose(np.linalg.norm(data[:, 4:], axis=1), 1, atol=1e-3):
        raise ValueError('trajectory quaternion must have unit norm')
    return data


def associate(estimate, truth, tolerance=0.06):
    estimate, truth = validate_trajectory(estimate), validate_trajectory(truth)
    stamps = estimate[:, 0]
    i = np.clip(np.searchsorted(truth[:, 0], stamps), 1, len(truth)-1)
    gap = np.minimum(abs(truth[i, 0]-stamps), abs(truth[i-1, 0]-stamps))
    if np.any(gap > tolerance) or stamps[0] < truth[0, 0] or stamps[-1] > truth[-1, 0]:
        raise ValueError('truth timestamp association unavailable')
    xyz = np.column_stack([np.interp(stamps, truth[:, 0], truth[:, j]) for j in range(1, 4)])
    quat = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:]))(stamps).as_quat()
    return np.column_stack([stamps, xyz, quat])


def evaluate(estimate, truth, expected_stamps=None):
    estimate = validate_trajectory(estimate)
    reference = associate(estimate, truth)
    x, y = estimate[:, 1:4], reference[:, 1:4]
    xc, yc = x-x.mean(axis=0), y-y.mean(axis=0)
    u, singular, vt = np.linalg.svd(xc.T @ yc)
    if singular[1] < 1e-8 or np.max(np.linalg.norm(x-x[0], axis=1)) < 0.5:
        rotation = (Rotation.from_quat(reference[0, 4:]) *
                    Rotation.from_quat(estimate[0, 4:]).inv()).as_matrix()
    else:
        sign = np.diag([1., 1., np.linalg.det(vt.T @ u.T)])
        rotation = vt.T @ sign @ u.T
    translation = y.mean(axis=0) - rotation @ x.mean(axis=0)
    aligned = x @ rotation.T + translation
    error = np.linalg.norm(aligned-y, axis=1)
    aligned_rotation = Rotation.from_matrix(rotation) * Rotation.from_quat(estimate[:, 4:])
    truth_rotation = Rotation.from_quat(reference[:, 4:])
    attitude = np.rad2deg((truth_rotation.inv() * aligned_rotation).magnitude())
    # Relative SE(3) translation expressed in each trajectory's own body frame.
    j = np.searchsorted(estimate[:, 0], estimate[:, 0] + 1.0)
    i = np.flatnonzero(j < len(estimate))
    i = i[abs(estimate[j[i], 0]-estimate[i, 0]-1.0) <= 0.15]
    j = j[i]
    de = Rotation.from_quat(estimate[i, 4:]).inv().apply(x[j]-x[i])
    dt = truth_rotation[i].inv().apply(y[j]-y[i])
    rpe = np.linalg.norm(de-dt, axis=1)
    expected = estimate[:, 0] if expected_stamps is None else np.asarray(expected_stamps)
    nearest = np.min(abs(expected[:, None] - estimate[None, :, 0]), axis=1)
    return {
        'samples': len(estimate), 'coverage': float(np.mean(nearest < 0.051)),
        'duration_s': float(estimate[-1, 0]-estimate[0, 0]),
        'max_output_gap_s': float(np.max(np.diff(estimate[:, 0]))),
        'raw_position_rmse_m': float(np.sqrt(np.mean(np.sum((x-y)**2, axis=1)))),
        'ate_rmse_m': float(np.sqrt(np.mean(error**2))),
        'ate_p95_m': float(np.quantile(error, 0.95)), 'ate_max_m': float(error.max()),
        'attitude_rmse_deg': float(np.sqrt(np.mean(attitude**2))),
        'rpe_translation_rmse_m': float(np.sqrt(np.mean(rpe**2))) if len(rpe) else None,
        'alignment_se3': np.block([[rotation, translation[:, None]], [np.array([[0, 0, 0, 1]])]]).tolist(),
        'alignment_scale': 1.0,
    }
