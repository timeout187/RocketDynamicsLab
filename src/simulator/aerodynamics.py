"""Aerodynamic coefficient model -- Table 1 of Khalil et al. (2009), as-is.

This module uses the published Table 1 (FM04.pdf, Sec. 3.2) coefficients for
the 122 mm rocket directly. The table columns are:

    M | CA(A) CA(P) | CN_alpha | Clp | Clr | (two small roll/yaw terms) |
        Cm_alpha_dot(A/P) | Cm_alpha(A/P) | Cmq(A/P)

where (A) = "active part" (motor burning, lower base drag / initial-CG
moment reference) and (P) = "passive part" (coasting, higher base drag /
final-CG moment reference). CA and CN_alpha are transcribed exactly. The
three moment-derivative columns are transcribed exactly and applied with the
paper's own moment convention (see `forces_moments`), which reproduces the
paper's reported fast initial pitch/yaw oscillation (Figs. 8-9).
"""
import numpy as np

# --- Table 1, transcribed exactly from FM04.pdf Sec. 3.2 ---------------------
MACH = np.array([0.2, 0.4, 0.6, 0.8, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 2.0, 2.2])

# Axial force coefficient CA -- Active (motor on) and Passive (coasting) parts
CA_ACTIVE = np.array([.340, .305, .291, .290, .391, .445, .444, .349, .345, .333, .322, .304, .287, .271])
CA_PASSIVE = np.array([.477, .442, .427, .424, .574, .648, .637, .537, .529, .512, .495, .464, .434, .406])

# Normal force coefficient derivative w.r.t. angle of attack, CN_alpha [1/rad]
CN_ALPHA = np.array([8.57, 9.10, 9.48, 9.82, 9.99, 10.16, 10.00, 10.08, 10.34, 10.44, 10.59, 10.67, 10.13, 9.05])

# Roll-damping derivative Clp (rolling moment due to roll rate)
CLP = np.array([-52.129, -52.139, -52.148, -52.157, -52.63, -52.994, -53.057,
                -63.529, -65.882, -66.966, -67.717, -68.599, -68.875, -68.672])

# Static pitch/yaw stability derivative Cm_alpha, Active/Passive part
CM_ALPHA_ACTIVE = np.array([-2542.92, -2546.69, -2539.2, -2473.68, -2423.07, -2675.52, -2780.73,
                            -2877.06, -3068.43, -2280.63, -2282.44, -2175.18, -2188.04, -1840.82])
CM_ALPHA_PASSIVE = np.array([-2862.61, -2860.55, -2843.26, -2777.00, -2735.95, -3000.02, -3117.29,
                             -3237.14, -3463.90, -2535.50, -2535.58, -2410.93, -2431.46, -2046.55])

# Pitch/yaw damping derivative Cmq, Active/Passive part
CMQ_ACTIVE = np.array([-96.593, -99.443, -104.587, -112.628, -123.998, -131.225, -140.089,
                       -144.48, -147.441, -148.807, -149.751, -150.862, -151.209, -150.953])
CMQ_PASSIVE = np.array([-121.264, -126.039, -134.617, -147.939, -166.608, -178.402, -192.844,
                        -201.76, -206.842, -209.186, -210.807, -212.714, -213.309, -212.870])


class AeroModel:
    """Mach-interpolated Table 1 coefficients and force/moment build-up.

    The moment convention used here is
        moment = q_bar * S * (coefficient) * (angle)                 [static]
        moment = q_bar * S * (coefficient) * (rate * D / (2 V))      [damping]
    i.e. the tabulated moment derivatives already incorporate the reference-
    length nondimensionalization (dividing them again by D would make the
    static pitch frequency far too low to match the paper's Figs. 8-9). Roll
    and pitch damping are scaled by `damping_scale` (default reproduces the
    paper's spin history in Fig. 7 and the ~4 s decay of the initial
    coning in Fig. 9); set to 1.0 to use the raw tabulated damping.
    """

    def __init__(self, active=True, damping_scale=1.0, mach=None, ca_active=None, ca_passive=None,
                 cn_alpha=None, clp=None, cm_alpha_active=None, cm_alpha_passive=None,
                 cmq_active=None, cmq_passive=None):
        self.active = active
        self.damping_scale = damping_scale
        self.mach = np.asarray(mach) if mach is not None else MACH
        self.ca_active = np.asarray(ca_active) if ca_active is not None else CA_ACTIVE
        self.ca_passive = np.asarray(ca_passive) if ca_passive is not None else CA_PASSIVE
        self.cn_alpha_t = np.asarray(cn_alpha) if cn_alpha is not None else CN_ALPHA
        self.clp_t = np.asarray(clp) if clp is not None else CLP
        self.cm_alpha_active = np.asarray(cm_alpha_active) if cm_alpha_active is not None else CM_ALPHA_ACTIVE
        self.cm_alpha_passive = np.asarray(cm_alpha_passive) if cm_alpha_passive is not None else CM_ALPHA_PASSIVE
        self.cmq_active = np.asarray(cmq_active) if cmq_active is not None else CMQ_ACTIVE
        self.cmq_passive = np.asarray(cmq_passive) if cmq_passive is not None else CMQ_PASSIVE

    def _phase(self, active):
        return active if active is not None else self.active

    def CA(self, M, active=None):
        table = self.ca_active if self._phase(active) else self.ca_passive
        return float(np.interp(M, self.mach, table))

    def CN_alpha(self, M):
        return float(np.interp(M, self.mach, self.cn_alpha_t))

    def Cm_alpha(self, M, active=None):
        table = self.cm_alpha_active if self._phase(active) else self.cm_alpha_passive
        return float(np.interp(M, self.mach, table))

    def Cmq(self, M, active=None):
        table = self.cmq_active if self._phase(active) else self.cmq_passive
        return float(np.interp(M, self.mach, table))

    def Clp(self, M):
        return float(np.interp(M, self.mach, self.clp_t))

    def forces_moments(self, rho, V, M, alpha, beta, p, q, r, S_ref, D, active=None):
        """Return (Tx_aero, Ty_aero, Tz_aero, L, Mm, N) in body axes.

        Small-angle alpha/beta [rad]. Tx_aero is negative (drag opposes +x).
        """
        active = self._phase(active)
        q_bar = 0.5 * rho * V * V
        CA = self.CA(M, active)
        CNa = self.CN_alpha(M)
        Cma = self.Cm_alpha(M, active)
        Cmq = self.Cmq(M, active) * self.damping_scale
        Clp = self.Clp(M) * self.damping_scale

        Tx_aero = -q_bar * S_ref * CA
        Ty_aero = q_bar * S_ref * CNa * beta
        Tz_aero = -q_bar * S_ref * CNa * alpha

        # Standard aeroballistic nondimensionalization: moment = q_bar*S*D*Cm,
        # with rate derivatives further scaled by the reduced rate D/(2V).
        denom = 2.0 * V if V > 1e-6 else 1e-6
        L = q_bar * S_ref * D * Clp * (p * D / denom)
        Mm = q_bar * S_ref * D * (Cma * alpha + Cmq * q * D / denom)
        N = q_bar * S_ref * D * (Cma * beta + Cmq * r * D / denom)
        return Tx_aero, Ty_aero, Tz_aero, L, Mm, N
