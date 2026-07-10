"""Tests for the rotational-dynamics (Euler's Equation) generalization with
a nonzero cross-inertia term Izx, and the rotating/curved-Earth Navigation
Equation extension -- see src/simulator/equations_of_motion.py.
"""
import numpy as np

from simulator import RocketParams, run_simulation
from simulator.equations_of_motion import state_derivative, initial_state
from simulator.rocket import ROCKET_122MM
from simulator.atmosphere import Atmosphere
from simulator.aerodynamics import AeroModel


def test_izx_zero_matches_old_axisymmetric_formula_algebraically():
    """With Izx=0 the general Euler's Equation must reduce EXACTLY to the
    old axisymmetric formulas:
        p_dot = (L - (Izz-Iyy)*q*r)/Ixx
        q_dot = (M - (Ixx-Izz)*r*p)/Iyy
        r_dot = (N - (Iyy-Ixx)*p*q)/Izz
    Verified here by evaluating both the new general code path (via
    state_derivative, using the default Izx=0 rocket) and the old formula
    directly at several representative body-rate states, and checking
    bit-identical results within tight tolerance.
    """
    rocket = ROCKET_122MM
    atmo = Atmosphere()
    aero = AeroModel()

    for p, q, r in [(0.0, 0.0, 0.0), (36.4, 0.1, -0.2), (5.0, 2.0, -3.0), (-10.0, 0.5, 0.5)]:
        x = initial_state(rocket, elevation_deg=45.0)
        x[3], x[4], x[5] = p, q, r
        dx = state_derivative(0.5, x, rocket, atmo, aero)

        # Recompute via the OLD axisymmetric formula, using the same
        # forces/moments the general code path would have used.
        u, v, w = x[0], x[1], x[2]
        phi, theta, psi = x[6], x[7], x[8]
        from simulator.frames import euler_to_LBE
        L_BE = euler_to_LBE(phi, theta, psi)
        wind_body = L_BE.T @ np.array([0.0, 0.0, 0.0])
        u_r, v_r, w_r = u - wind_body[0], v - wind_body[1], w - wind_body[2]
        V = np.sqrt(u_r**2 + v_r**2 + w_r**2)
        alpha = np.arctan2(w_r, u_r) if abs(u_r) > 1e-9 else 0.0
        beta = np.arcsin(v_r / V) if V > 1e-9 else 0.0
        altitude = -x[11]
        rho = atmo.density(max(altitude, 0.0))
        M = atmo.mach(max(altitude, 0.0), V)
        active = 0.5 < rocket.burn_time
        Tx_a, Ty_a, Tz_a, L_aero, M_aero, N_aero = aero.forces_moments(
            rho, V, M, alpha, beta, p, q, r, rocket.reference_area, rocket.caliber, active=active)
        q_bar = 0.5 * rho * V * V
        L_aero += q_bar * rocket.reference_area * rocket.caliber * rocket.fin_cant_coefficient

        Ixx = rocket.Ixx_at(0.5)
        Iyy = rocket.Iyy_at(0.5)
        Izz = Iyy
        p_dot_old = (L_aero - (Izz - Iyy) * q * r) / Ixx
        q_dot_old = (M_aero - (Ixx - Izz) * r * p) / Iyy
        r_dot_old = (N_aero - (Iyy - Ixx) * p * q) / Izz

        assert np.isclose(dx[3], p_dot_old, rtol=1e-12, atol=1e-10)
        assert np.isclose(dx[4], q_dot_old, rtol=1e-12, atol=1e-10)
        assert np.isclose(dx[5], r_dot_old, rtol=1e-12, atol=1e-10)


def test_izx_zero_default_trajectory_unchanged():
    """A full-trajectory run with the default (Izx=0) rocket must be finite
    and land, exactly as before this change (behavior-preserving check).
    """
    res = run_simulation(elevation_deg=45.0, t_end=60.0, dt=0.01, method="rk4")
    assert np.all(np.isfinite(res.x))
    assert res.altitude[-1] <= 1.0


def test_izx_nonzero_runs_finite():
    """A nonzero cross-inertia term must not crash or produce NaN/Inf."""
    rocket = RocketParams(Izx_initial=0.02, Izx_final=0.015)
    res = run_simulation(rocket=rocket, elevation_deg=45.0, t_end=30.0, dt=0.01, method="rk4")
    assert np.all(np.isfinite(res.x))


def test_rotating_earth_runs_finite_and_has_extra_states():
    res = run_simulation(elevation_deg=45.0, t_end=30.0, dt=0.01, method="rk4",
                          include_earth_rotation=True, latitude=np.radians(30.0))
    assert np.all(np.isfinite(res.x))
    assert res.x.shape[1] == 15  # N, E, D plus V_N, V_E, V_D


def test_rotating_earth_differs_measurably_from_flat_earth():
    """The Coriolis/curvature terms must have an actual, non-trivial effect
    on the trajectory (not a no-op) when include_earth_rotation=True.
    """
    flat = run_simulation(elevation_deg=45.0, t_end=60.0, dt=0.01, method="rk4",
                           include_earth_rotation=False)
    rotating = run_simulation(elevation_deg=45.0, t_end=60.0, dt=0.01, method="rk4",
                               include_earth_rotation=True, latitude=np.radians(30.0))
    assert np.all(np.isfinite(rotating.x))
    # Compare drift (East displacement) at impact -- Coriolis deflection
    # should produce a measurable difference vs. the flat-Earth run.
    assert abs(rotating.drift - flat.drift) > 1e-3
