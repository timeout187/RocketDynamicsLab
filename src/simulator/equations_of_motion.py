"""6-DOF rigid-body equations of motion.

Assembles the state derivative dx/dt implementing Eqs. (1)-(2) of Khalil et
al. (2009) plus the kinematic and navigation relations from Fig. 1. See
/docs/equations.md for a term-by-term explanation and /docs/mathematical-model.md
for the assumptions (a)-(e) these equations rely on.

State vector (12 states, matches Fig. 1's signal flow; grows to 15 states
when include_earth_rotation=True -- see "Navigation Equation" below):
    x = [u, v, w, p, q, r, phi, theta, psi, N, E, D, (V_N, V_E, V_D)]
        u,v,w   body-axis velocity components,          [m/s]
        p,q,r   body-axis angular rates,                 [rad/s]
        phi,theta,psi  Euler angles (roll,pitch,yaw),     [rad]
        N,E,D   geodetic position (North,East,Down),      [m]
        V_N,V_E,V_D  geodetic-frame velocity (only present when
                     include_earth_rotation=True),         [m/s]

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

# Mean Earth radius, [m] (WGS84 semi-major axis). The paper's Navigation
# Equation is left unspecified for the curved/rotating-Earth case (only the
# flat-Earth L_BE @ [u,v,w] form and the Eq. (3) body-rate Coriolis term are
# given explicitly), so this module uses the standard spherical-Earth
# approximation R_meridian = R_normal = R_EARTH -- adequate for the paper's
# short-range (tens of km) 122mm rocket case study and simple enough for
# classroom use. A full WGS84 ellipsoidal model (distinct, latitude-dependent
# R_meridian/R_normal) is left as a documented simplification -- see
# docs/coordinate-systems.md.
R_EARTH = 6378137.0  # [m]


def state_derivative(t, x, rocket: RocketParams, atmo: Atmosphere, aero: AeroModel,
                      wind_ned=(0.0, 0.0, 0.0), include_earth_rotation=False,
                      latitude=0.0):
    """Return dx/dt for the state vector x, at time t (12 states, or 15 when
    include_earth_rotation=True -- see module docstring).

    Implements:
      Eq. (1): translational (u,v,w) dynamics in body axes with gravity
               resolved through the Euler angles and thrust/aero forces.
      Euler's Equation box: rotational (p,q,r) dynamics using the paper's
               general (non-axisymmetric) form with cross-inertia term Izx.
               This collapses to the paper's Ixy=Iyz=Izx=0 case-study
               simplification when Izx=0 (RocketParams' default).
      Kinematics Equation: Euler angle rates from body rates.
      Navigation Equation: NED position rates from body velocity via L_BE
               (flat, non-rotating Earth, default), OR -- when
               include_earth_rotation=True -- the full rotating/curved-Earth
               mechanization with Coriolis-coupled geodetic velocity states
               V_N, V_E, V_D and latitude-dependent transport rates.
    """
    u, v, w, p, q, r, phi, theta, psi, N, E, D = x[:12]
    altitude = -D

    m = rocket.mass_at(t)
    mdot = rocket.mass_rate(t)
    Ixx = rocket.Ixx_at(t)
    Iyy = rocket.Iyy_at(t)
    Izz = Iyy  # axisymmetric assumption, paper Sec. 2 / Sec. 3 (Iyy = Izz)
    Izx = rocket.Izx_at(t)  # cross-inertia term; 0.0 by default (case study)
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

    # --- Euler's Equation (rotational dynamics), general form with cross-
    # inertia term Izx (paper Fig. 1 boxed "Euler's Equation", fully resolved
    # form used in the block diagram -- NOT the earlier axisymmetric Eq. (2)
    # simplification). Strictly generalizes the old axisymmetric-only code:
    # with Izx=0 (RocketParams' default, matching the paper's case study)
    # the denominator Ixx*Izz - Izx^2 -> Ixx*Izz and every Izx-bearing term
    # vanishes, reducing algebraically to the previous
    #   p_dot = (L - (Izz-Iyy)*q*r)/Ixx
    #   q_dot = (M - (Ixx-Izz)*r*p)/Iyy
    #   r_dot = (N - (Iyy-Ixx)*p*q)/Izz
    # expressions exactly.
    denom_pr = Ixx * Izz - Izx ** 2
    p_dot = (Izz * L_aero + Izx * N_aero
             + Izz * Izx * (1.0 + (Ixx - Iyy) / Izz) * p * q
             + (Iyy * Izz - Izz ** 2 - Izx ** 2) * q * r) / denom_pr
    q_dot = M_aero / Iyy + (Izx * (r ** 2 - p ** 2) + (Izz - Ixx) * r * p) / Iyy
    r_dot = (Izx * L_aero + Ixx * N_aero
             + (Iyy - Ixx - Izz) * Izx * r * q
             + (Ixx ** 2 - Ixx * Iyy + Izx ** 2) * q * p) / denom_pr

    # --- Kinematics Equation: Euler angle rates ---
    from .frames import kinematic_rates
    phi_dot, theta_dot, psi_dot = kinematic_rates(phi, theta, p, q, r)

    if not include_earth_rotation:
        # --- Navigation Equation (default): flat, non-rotating Earth ---
        # NED position rates are simply body velocity rotated into the
        # geodetic frame via L_BE. This is the paper's short-range, flat-
        # Earth navigation relation and is left byte-for-byte unchanged.
        vel_ned = L_BE @ np.array([u, v, w])
        N_dot, E_dot, D_dot = vel_ned
        return np.array([u_dot, v_dot, w_dot, p_dot, q_dot, r_dot,
                          phi_dot, theta_dot, psi_dot, N_dot, E_dot, D_dot])

    # --- Navigation Equation (full rotating/curved Earth) ---
    # Extra states V_N, V_E, V_D (geodetic-frame velocity) are carried
    # alongside N, E, D so the Coriolis/transport-rate coupled velocity ODE
    # below can be integrated directly -- see module docstring. N, E, D
    # remain plain integrated NED distances (Ndot=V_N etc.), and we recover
    # the current geodetic latitude from the North distance travelled so
    # far via the spherical-Earth approximation (see R_EARTH above):
    #   lambda = latitude0 + N / R_EARTH
    # This spherical approximation (R_meridian = R_normal = R_EARTH) is the
    # documented simplification; a full WGS84 ellipsoidal model would make
    # R_meridian/R_normal latitude-dependent.
    V_N, V_E, V_D = x[12:15]
    H_G = altitude
    lam = latitude + N / R_EARTH
    cos_lam = np.cos(lam) if abs(np.cos(lam)) > 1e-9 else 1e-9
    R_meridian = R_EARTH
    R_normal = R_EARTH

    lambda_dot = V_N / (R_meridian + H_G)
    mu_dot = V_E / ((R_normal + H_G) * cos_lam)

    # Specific-force / acceleration terms in the geodetic frame, obtained by
    # rotating the body-axis velocity via L_BE (reusing what the flat-Earth
    # path calls vel_ned -- see task spec / docs/coordinate-systems.md).
    a_N, a_E, a_D = L_BE @ np.array([u, v, w])

    omega = OMEGA_EARTH
    V_N_dot = (mu_dot + 2.0 * omega) * np.sin(lam) * V_E - lambda_dot * V_D + a_N
    V_E_dot = -(mu_dot + 2.0 * omega) * np.sin(lam) * V_N - (mu_dot + 2.0 * omega) * np.cos(lam) * V_D + a_E
    V_D_dot = (mu_dot + 2.0 * omega) * np.cos(lam) * V_E + lambda_dot * V_N + a_D

    N_dot, E_dot, D_dot = V_N, V_E, V_D

    return np.array([u_dot, v_dot, w_dot, p_dot, q_dot, r_dot,
                      phi_dot, theta_dot, psi_dot, N_dot, E_dot, D_dot,
                      V_N_dot, V_E_dot, V_D_dot])


def initial_state(rocket: RocketParams, elevation_deg: float, azimuth_deg: float = 0.0,
                   roll_deg: float = 0.0, include_earth_rotation: bool = False) -> np.ndarray:
    """Build the initial state vector from muzzle conditions (Fig. 1 'Initial
    Conditions'). Returns 12 states normally, or 15 (appending V_N, V_E, V_D)
    when include_earth_rotation=True -- see equations_of_motion module
    docstring.
    """
    theta0 = np.radians(elevation_deg)
    psi0 = np.radians(azimuth_deg)
    phi0 = np.radians(roll_deg)
    u0 = rocket.v_muzzle
    v0 = 0.0
    w0 = 0.0
    p0 = rocket.p_muzzle
    q0 = 0.0
    r0 = 0.0
    state12 = [u0, v0, w0, p0, q0, r0, phi0, theta0, psi0, 0.0, 0.0, 0.0]
    if not include_earth_rotation:
        return np.array(state12)

    from .frames import euler_to_LBE
    L_BE0 = euler_to_LBE(phi0, theta0, psi0)
    V_N0, V_E0, V_D0 = L_BE0 @ np.array([u0, v0, w0])
    return np.array(state12 + [V_N0, V_E0, V_D0])
