import numpy as np
from simulator.frames import euler_to_LBE, kinematic_rates


def test_LBE_identity_at_zero_angles():
    L = euler_to_LBE(0.0, 0.0, 0.0)
    assert np.allclose(L, np.eye(3), atol=1e-10)


def test_LBE_is_orthonormal():
    L = euler_to_LBE(0.3, 0.5, -0.7)
    assert np.allclose(L @ L.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(L), 1.0, atol=1e-9)


def test_kinematic_rates_zero_when_body_rates_zero():
    phi_dot, theta_dot, psi_dot = kinematic_rates(0.2, 0.3, 0.0, 0.0, 0.0)
    assert phi_dot == 0.0 and theta_dot == 0.0 and psi_dot == 0.0


def test_kinematic_rates_matches_body_rates_at_zero_attitude():
    phi_dot, theta_dot, psi_dot = kinematic_rates(0.0, 0.0, 1.0, 2.0, 3.0)
    assert np.isclose(phi_dot, 1.0)
    assert np.isclose(theta_dot, 2.0)
    assert np.isclose(psi_dot, 3.0)
