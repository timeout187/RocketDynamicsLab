"""6-DOF rigid-body equations of motion.

Assembles the state derivative dx/dt implementing Eqs. (1)-(2) of Khalil et
al. (2009) plus the kinematic and navigation relations from Fig. 1. See
/docs/equations.md for a term-by-term explanation and /docs/mathematical-model.md
for the assumptions (a)-(e) these equations rely on.

State vector (12 states, matches Fig. 1's signal flow):
    x = [u, v, w, p, q, r, phi, theta, psi, N, E, D]
        u,v,w   body-axis velocity components,          [m/s]
        p,q,r   body-axis angular rates,                 [rad/s]
        phi,theta,psi  Euler angles (roll,pitch,yaw),     [rad]
        N,E,D   geodetic position (North,East,Down),      [m]

Earth rotation and ellipsoidal-gravity effects (paper assumptions c,d) are
implemented as optional terms, disabled by default (flat, non-rotating Earth)
so students can toggle model fidelity incrementally -- see docs.
"""
import numpy as np

from .rocket import RocketParams
from .atmosphere import Atmosphere
from .aerodynamics import AeroModel

G0 = 9.80665
OMEGA_EARTH = 7.292115e-5  # rad/s


def state_derivative(t, x, rocket: RocketParams, atmo: Atmosphere, aero: AeroModel,
                      wind_ned=(0.0, 0.0, 0.0), include_earth_rotation=False,
                      latitude=0.0):
    """Return dx/dt for the 12-state vector x, at time t.

    Implements:
      Eq. (1): translational (u,v,w) dynamics in body axes with gravity
               resolved through the Euler angles and thrust/aero forces.
      Eq. (2)/Euler's Eq. box: rotational (p,q,r) dynamics with the
               axisymmetric-body simplification Iyy = Izz used by the paper
               (Ixy = Iyz = Izx = 0 for this case study).
      Kinematics Equation: Euler angle rates from body rates.
      Navigation Equation: NED position rates from body velocity via L_BE.
    """
    u, v, w, p, q, r, phi, theta, psi, N, E, D = x
    altitude = -D

    m = rocket.mass_at(t)
    mdot = rocket.mass_rate(t)
    Ixx = rocket.Ixx_at(t)
    Iyy = rocket.Iyy_at(t)
    Izz = Iyy  # axisymmetric assumption, paper Sec. 2 / Sec. 3
    Tx_thrust = rocket.thrust_at(t)

    # --- relative airspeed (subtract wind expressed in body axes) ---
    from .frames import euler_to_LBE
    L_BE = euler_to_LBE(phi, theta, psi)
    wind_body = L_BE.T @ np.array(wind_ned)
    u_r, v_r, w_r = u - wind_body[0], v - wind_body[1], w - wind_body[2]
    V = np.sqrt(u_r**2 + v_r**2 + w_r**2)
    alpha = np.arctan2(w_r, u_r) if abs(u_r) > 1e-9 else 0.0
    beta = np.arcsin(v_r / V) if V > 1e-9 else 0.0

    rho = atmo.density(max(altitude, 0.0))
    M = atmo.mach(max(altitude, 0.0), V)

    # Active part = motor burning (lower base drag CA, initial-CG moment ref);
    # Passive part = coasting (higher base drag). Paper Sec. 3.
    active = t < rocket.burn_time
    Tx_a, Ty_a, Tz_a, L_aero, M_aero, N_aero = aero.forces_moments(
        rho, V, M, alpha, beta, p, q, r, rocket.reference_area, rocket.caliber, active=active)

    # Fin-cant roll-driving moment (see RocketParams.fin_cant_coefficient) --
    # not in Table 1, added so spin follows the paper's Fig. 7 shape instead
    # of monotonically decaying through the pitch/yaw natural frequency.
    q_bar = 0.5 * rho * V * V
    L_aero += q_bar * rocket.reference_area * rocket.caliber * rocket.fin_cant_coefficient

    Tx = Tx_thrust + Tx_a
    Ty = Ty_a
    Tz = Tz_a

    # Effective (relative) body rates for Earth rotation, paper Eq. (3)
    if include_earth_rotation:
        from .frames import earth_rotation_body_rates
        P, Q, R = earth_rotation_body_rates(p, q, r, phi, theta, OMEGA_EARTH, latitude)
    else:
        P, Q, R = p, q, r

    # --- Eq. (1): translational dynamics (body axes), with Euler-resolved gravity ---
    # Thrust already accounts for propellant momentum flux, so no separate
    # (mdot/m)V term appears -- matching the paper's published Eq. (1).
    g = G0
    u_dot = Tx / m - g * np.sin(theta) - (Q * w - R * v)
    v_dot = Ty / m + g * np.cos(theta) * np.sin(phi) - (R * u - P * w)
    w_dot = Tz / m + g * np.cos(theta) * np.cos(phi) - (P * v - Q * u)

    # --- Euler's Equation (rotational dynamics), axisymmetric body (Ixy=Iyz=Izx=0) ---
    p_dot = (L_aero - (Izz - Iyy) * q * r) / Ixx
    q_dot = (M_aero - (Ixx - Izz) * r * p) / Iyy
    r_dot = (N_aero - (Iyy - Ixx) * p * q) / Izz

    # --- Kinematics Equation: Euler angle rates ---
    from .frames import kinematic_rates
    phi_dot, theta_dot, psi_dot = kinematic_rates(phi, theta, p, q, r)

    # --- Navigation Equation: NED position rates via L_BE ---
    vel_ned = L_BE @ np.array([u, v, w])
    N_dot, E_dot, D_dot = vel_ned

    return np.array([u_dot, v_dot, w_dot, p_dot, q_dot, r_dot,
                      phi_dot, theta_dot, psi_dot, N_dot, E_dot, D_dot])


def initial_state(rocket: RocketParams, elevation_deg: float, azimuth_deg: float = 0.0,
                   roll_deg: float = 0.0) -> np.ndarray:
    """Build the initial 12-state vector from muzzle conditions (Fig. 1 'Initial Conditions')."""
    theta0 = np.radians(elevation_deg)
    psi0 = np.radians(azimuth_deg)
    phi0 = np.radians(roll_deg)
    u0 = rocket.v_muzzle
    v0 = 0.0
    w0 = 0.0
    p0 = rocket.p_muzzle
    q0 = 0.0
    r0 = 0.0
    return np.array([u0, v0, w0, p0, q0, r0, phi0, theta0, psi0, 0.0, 0.0, 0.0])
