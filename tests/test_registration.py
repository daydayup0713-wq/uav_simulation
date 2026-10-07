import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from uav_lab_localization.registration import register, RegistrationError


def corner(seed=1):
    rng = np.random.default_rng(seed)
    p = rng.uniform(-2, 2, (2400, 3))
    for i in range(3):
        p[800*i:800*(i+1), i] = -2
    return p


def test_recovers_six_dof_with_partial_overlap():
    target = corner()
    rotation = Rotation.from_euler('xyz', [0.07, -0.06, 0.08])
    translation = np.array([0.18, -0.13, 0.12])
    source = rotation.inv().apply(target[:1900] - translation)
    result = register(source, target)
    assert result.inlier_fraction > 0.9
    assert result.rmse < 0.01
    assert np.linalg.norm(result.transform[:3, 3] - translation) < 0.01
    assert (Rotation.from_matrix(result.transform[:3, :3]).inv() * rotation).magnitude() < 0.005
    assert result.observable


def test_no_overlap_is_rejected():
    with pytest.raises(RegistrationError, match='correspondences|overlap'):
        register(corner() + 100, corner())


def test_plane_is_not_six_dof_observable():
    p = corner()[:800]
    with pytest.raises(RegistrationError, match='degenerate'):
        register(p, p)


def test_non_finite_cloud_rejected():
    p = corner()
    p[0, 0] = np.nan
    with pytest.raises(ValueError, match='finite'):
        register(p, corner())


def test_observability_and_solution_are_invariant_to_map_origin():
    reference=register(corner(),corner())
    for offset in ([20,0,0],[100,-60,50]):
        p=corner()+offset
        result=register(p,p)
        assert result.observable and result.rmse<.001
        assert np.allclose(result.information_eigenvalues,reference.information_eigenvalues,rtol=1e-5,atol=1e-8)
