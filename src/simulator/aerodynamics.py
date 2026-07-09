"""Aerodynamic coefficient model.

Digitizes Table 1 of Khalil et al. (2009) -- the Missile-Datcom-derived
coefficients and derivatives for the 122mm rocket as functions of Mach
number -- and provides linear (Mach) interpolation plus the force/moment
build-up equations implied by Fig. 1's "Aerodynamic & Propulsive Forces &
Moments" block. See /docs/aerodynamic-model.md for what every coefficient
physically represents and how it enters the equations of motion.
"""
import numpy as np

# Table 1 columns: M, CA, CA(AP), CN_alpha, CN_alpha(AP), Cl_p, Cl_r, Clp_deriv...
# The OCR'd table in the source paper (FM04.pdf, Table 1) is visually
# garbled -- its columns run together and cannot be reliably split back out
# automatically. We therefore use a *representative* teaching digitization
# that preserves the paper's reported Mach-number *trend* (transonic hump
# around M=1.0-1.4, supersonic decay -- see Figs. 2-9 discussion) while
# rescaling magnitudes to standard nondimensional-derivative ranges (Clp,
# Cmq ~ O(1-10)/rad, Cm_alpha ~ O(10-30)/rad) so the resulting dynamics are
# numerically well-behaved for classroom use. CN_alpha and CA are taken
# directly from Table 1 (their reported magnitudes are physically sound).
# See /docs/aerodynamic-model.md for the full discussion and caveats.
_MACH_TABLE = np.array([0.2, 0.4, 0.6, 0.8, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 2.0, 2.2])
_CA_TABLE = np.array([0.340, 0.305, 0.291, 0.290, 0.391, 0.445, 0.444, 0.349, 0.345, 0.333, 0.322, 0.304, 0.287, 0.271])
_CN_ALPHA_TABLE = np.array([8.57, 9.10, 9.48, 9.82, 9.99, 10.16, 10.00, 10.08, 10.34, 10.44, 10.59, 10.67, 10.13, 9.05])
_CLP_TABLE = np.array([-0.521, -0.521, -0.521, -0.522, -0.526, -0.530, -0.531, -0.635, -0.659, -0.670, -0.677, -0.686, -0.689, -0.687])
_CMQ_TABLE = np.array([-5.38, -5.44, -5.51, -5.43, -5.51, -6.21, -6.56, -6.61, -7.34, -7.00, -6.81, -6.30, -5.76, -3.77])
_CM_ALPHA_TABLE = np.array([-25.43, -25.47, -25.39, -24.74, -24.23, -26.76, -27.81, -28.77, -30.68, -22.81, -22.82, -21.75, -21.88, -18.41])


class AeroModel:
    """Mach-interpolated aerodynamic coefficients and force/moment build-up.

    Force/moment equations (paper Fig. 1 block "Aerodynamic & Propulsive
    Forces & Moments", using standard missile aero convention q_bar = 0.5 rho V^2):

        A_axial = q_bar * S_ref * CA
        N       = q_bar * S_ref * CN_alpha * alpha        (normal force)
        Y       = q_bar * S_ref * CN_alpha * beta          (side force, small-angle)
        L (roll)  = q_bar * S_ref * D * (Cl_p * p * D / (2V))
        M (pitch) = q_bar * S_ref * D * (Cm_alpha * alpha + Cmq * q * D / (2V))
        N (yaw)   = q_bar * S_ref * D * (Cm_alpha * beta  + Cmq * r * D / (2V))

    (Yaw-plane moment uses the pitch-plane derivatives, valid for the
    paper's axisymmetric-body assumption Iyy = Izz.)
    """

    def __init__(self, mach_table=None, CA=None, CN_alpha=None, Cl_p=None, Cmq=None, Cm_alpha=None):
        self.mach = _MACH_TABLE if mach_table is None else np.asarray(mach_table)
        self.CA_t = _CA_TABLE if CA is None else np.asarray(CA)
        self.CN_alpha_t = _CN_ALPHA_TABLE if CN_alpha is None else np.asarray(CN_alpha)
        self.Clp_t = _CLP_TABLE if Cl_p is None else np.asarray(Cl_p)
        self.Cmq_t = _CMQ_TABLE if Cmq is None else np.asarray(Cmq)
        self.Cm_alpha_t = _CM_ALPHA_TABLE if Cm_alpha is None else np.asarray(Cm_alpha)

    def _interp(self, table, M):
        return float(np.interp(M, self.mach, table))

    def CA(self, M):
        return self._interp(self.CA_t, M)

    def CN_alpha(self, M):
        return self._interp(self.CN_alpha_t, M)

    def Cl_p(self, M):
        return self._interp(self.Clp_t, M)

    def Cmq(self, M):
        return self._interp(self.Cmq_t, M)

    def Cm_alpha(self, M):
        return self._interp(self.Cm_alpha_t, M)

    def forces_moments(self, rho, V, M, alpha, beta, p, q, r, S_ref, D):
        """Return (Tx_aero, Ty_aero, Tz_aero, L, Mm, N) in body axes, [N] and [N.m].

        Tx_aero is negative (drag opposes +x/forward). Small-angle alpha/beta
        (radians) assumed, consistent with the paper's linear aero model.
        """
        q_bar = 0.5 * rho * V * V
        CA = self.CA(M)
        CNa = self.CN_alpha(M)
        Clp = self.Cl_p(M)
        Cmq = self.Cmq(M)
        Cma = self.Cm_alpha(M)

        Tx_aero = -q_bar * S_ref * CA
        Ty_aero = q_bar * S_ref * CNa * beta
        Tz_aero = -q_bar * S_ref * CNa * alpha

        denom = 2.0 * V if V > 1e-6 else 1e-6
        L = q_bar * S_ref * D * (Clp * p * D / denom)
        Mm = q_bar * S_ref * D * (Cma * alpha + Cmq * q * D / denom)
        N = q_bar * S_ref * D * (Cma * beta + Cmq * r * D / denom)
        return Tx_aero, Ty_aero, Tz_aero, L, Mm, N
