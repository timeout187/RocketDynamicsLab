"""Coordinate frame transformations.

Implements Eq. (4) (body-to-geodetic direction cosine matrix L_BE) and the
Euler kinematic equation (the un-numbered "Kinematics Equation" box in
Fig. 1) of Khalil et al. (2009). See /docs/coordinate-systems.md for the
full derivation, sign conventions and singularity discussion.

Frames used (matching the paper's nomenclature):
  F_B : body-fixed frame, x forward along the axis of symmetry, y right, z down.
  F_E : local geodetic ("NED") frame, x North, y East, z Down.
Euler angles (3-2-1 / yaw-pitch-roll): psi (yaw/azimuth), theta (pitch/inclination),
phi (roll/bank).
"""
import numpy as np


def euler_to_LBE(phi: float, theta: float, psi: float) -> np.ndarray:
    """Direction cosine matrix transforming a vector from body frame F_B to
    the geodetic frame F_E, i.e. v_E = L_BE @ v_B (paper's Eq. 4).

    phi, theta, psi in radians.
    """
    sphi, cphi = np.sin(phi), np.cos(phi)
    sth, cth = np.sin(theta), np.cos(theta)
    spsi, cpsi = np.sin(psi), np.cos(psi)

    L_BE = np.array([
        [cth * cpsi, sphi * sth * cpsi - cphi * spsi, cphi * sth * cpsi + sphi * spsi],
        [cth * spsi, sphi * sth * spsi + cphi * cpsi, cphi * sth * spsi - sphi * cpsi],
        [-sth,       sphi * cth,                       cphi * cth],
    ])
    return L_BE


def kinematic_rates(phi: float, theta: float, P: float, Q: float, R: float):
    """Euler angle rates (phi_dot, theta_dot, psi_dot) from body rates [P Q R].

    This is the paper's "Kinematics Equation":
        [phi_dot; theta_dot; psi_dot] =
            [[1, sin(phi)tan(theta), cos(phi)tan(theta)],
             [0, cos(phi),          -sin(phi)],
             [0, sin(phi)sec(theta), cos(phi)sec(theta)]] @ [P; Q; R]

    Singular at theta = +-90 deg (gimbal lock) -- see docs for discussion of
    quaternion alternatives.
    """
    sphi, cphi = np.sin(phi), np.cos(phi)
    tth = np.tan(theta)
    sec_th = 1.0 / np.cos(theta)

    phi_dot = P + sphi * tth * Q + cphi * tth * R
    theta_dot = cphi * Q - sphi * R
    psi_dot = sphi * sec_th * Q + cphi * sec_th * R
    return phi_dot, theta_dot, psi_dot


def earth_rotation_body_rates(p: float, q: float, r: float, phi: float,
                                theta: float, omega_earth: float, latitude: float):
    """Eq. (3): total body rates [P Q R] including Earth's rotation, obtained
    by transforming the Earth's angular velocity vector into body axes and
    adding it to the relative body rates [p q r].
    """
    L_BE = euler_to_LBE(phi, theta, 0.0)
    omega_e_E = np.array([omega_earth * np.cos(latitude), 0.0, -omega_earth * np.sin(latitude)])
    omega_e_B = L_BE.T @ omega_e_E
    P = p + omega_e_B[0]
    Q = q + omega_e_B[1]
    R = r + omega_e_B[2]
    return P, Q, R
