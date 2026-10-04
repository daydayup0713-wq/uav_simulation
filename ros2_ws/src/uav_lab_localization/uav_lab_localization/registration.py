"""Small CPU point-to-plane ICP for learning and coarse-prior relocalization.

This is geometric registration, not an IMU estimator. Transform maps source to target.
"""
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


class RegistrationError(ValueError):
    pass


@dataclass
class RegistrationResult:
    transform: np.ndarray
    inlier_fraction: float
    rmse: float
    observable: bool
    information_eigenvalues: np.ndarray
    iterations: int


def validate_cloud(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 100:
        raise ValueError('point cloud must contain at least 100 xyz points')
    if not np.isfinite(points).all():
        raise ValueError('point cloud must be finite')
    return points


def voxel_downsample(points, resolution=0.15):
    points = validate_cloud(points)
    _, index = np.unique(np.floor(points / resolution).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(index)]


def register(source, target, initial=None, max_distance=0.6, max_iterations=35,
             min_inlier_fraction=0.65, max_rmse=0.10):
    source, target = validate_cloud(source), validate_cloud(target)
    transform = np.eye(4) if initial is None else np.asarray(initial, dtype=float).copy()
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError('initial pose must be a finite 4x4 transform')
    if not np.allclose(transform[3], [0, 0, 0, 1]) or not np.allclose(
            transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5
    ) or np.linalg.det(transform[:3, :3]) < 0.99:
        raise ValueError('initial pose must be SE(3)')
    tree = cKDTree(target)
    _, neighbors = tree.query(target, k=20)
    neighborhood = target[neighbors]
    centered = neighborhood - neighborhood.mean(axis=1, keepdims=True)
    _, eigenvectors = np.linalg.eigh(np.einsum('nki,nkj->nij', centered, centered))
    normals = eigenvectors[:, :, 0]
    for iteration in range(max_iterations):
        transformed = source @ transform[:3, :3].T + transform[:3, 3]
        distance, index = tree.query(transformed)
        keep = distance < max_distance
        if keep.sum() < max(100, min_inlier_fraction * len(source)):
            raise RegistrationError('insufficient overlap/correspondences')
        p, q, n = transformed[keep], target[index[keep]], normals[index[keep]]
        residual = np.einsum('ij,ij->i', n, p-q)
        jacobian = np.column_stack([n, np.cross(p, n)])
        information = jacobian.T @ jacobian / len(jacobian)
        values = np.linalg.eigvalsh(information)
        if values[0] < 1e-4 * values[-1]:
            raise RegistrationError('degenerate geometry: pose is not six-DOF observable')
        weights = np.minimum(1.0, 0.05 / np.maximum(abs(residual), 1e-9))
        delta = np.linalg.lstsq(jacobian * np.sqrt(weights[:, None]),
                                -residual * np.sqrt(weights), rcond=None)[0]
        update = np.eye(4)
        update[:3, :3] = Rotation.from_rotvec(delta[3:]).as_matrix()
        update[:3, 3] = delta[:3]
        transform = update @ transform
        if np.linalg.norm(delta) < 1e-6:
            break
    transformed = source @ transform[:3, :3].T + transform[:3, 3]
    distance, index = tree.query(transformed)
    keep = distance < max_distance
    fraction = float(keep.mean())
    rmse = float(np.sqrt(np.mean(distance[keep] ** 2))) if keep.any() else float('inf')
    if fraction < min_inlier_fraction or rmse > max_rmse:
        raise RegistrationError(f'registration rejected: overlap={fraction:.3f}, rmse={rmse:.3f}')
    return RegistrationResult(transform, fraction, rmse, True, values, iteration + 1)
