"""Rocket mass/geometry properties.

Defaults reproduce the 122mm unguided artillery rocket case study of
Khalil et al. (2009), Section 3.1 "Main data" (FM04.pdf, p.5/14).

Everything here is a *teaching dataset*: fictional/representative values
students can perturb freely. Do not treat as a real weapons-engineering
specification.
"""
from dataclasses import dataclass


@dataclass
class RocketParams:
    """Mass, geometry and inertia properties of the rigid body.

    During the "active" (boost) phase mass and axial/lateral inertia are
    linearly interpolated between the initial and final (burn-out) values
    over the burn time tk. After burn-out (the "passive"/free-flight phase)
    the rocket is treated as a fixed-mass projectile, per the paper's
    modeling assumption (Sec. 3, "Case Study").
    """

    caliber: float = 0.122          # D, [m]
    length: float = 2.870           # Lt, [m]
    mass_total: float = 66.0        # Mt, [kg] (includes propellant)
    mass_propellant: float = 20.5   # mp, [kg]
    burn_time: float = 1.67         # tk, [s]
    mean_thrust: float = 23600.0    # Tx, [N]

    cg_initial: float = 1.374       # C.G_xi from nose tip, [m]
    cg_final: float = 1.264         # C.G_xf from nose tip, [m]

    Ixx_initial: float = 0.1499     # [kg.m^2]
    Ixx_final: float = 0.1238       # [kg.m^2]
    Iyy_initial: float = 41.58      # = Izz_initial, [kg.m^2]
    Iyy_final: float = 33.83        # = Izz_final, [kg.m^2]

    v_muzzle: float = 26.7          # Vo, [m/s]
    p_muzzle: float = 36.4          # po (spin rate), [rad/s] (~5.8 rps)

    # Fin-cant roll-driving moment coefficient (dimensionless, lumped Cl_delta*delta).
    # NOT published in the paper's Table 1 -- the paper only tabulates roll
    # *damping* (Clp), but its Fig. 7 explicitly describes spin *increasing*
    # during boost "due to the inclination of the rocket fins," which requires
    # a roll-driving term Table 1 doesn't provide. This is an assumed value,
    # calibrated so the roll-rate history qualitatively matches Fig. 7's shape
    # (slight post-launch dip, rise through boost, decay after burnout) --
    # see docs/aerodynamic-model.md.
    fin_cant_coefficient: float = 2.0

    reference_area: float = None    # computed from caliber if None

    def __post_init__(self):
        if self.reference_area is None:
            self.reference_area = 3.141592653589793 * (self.caliber / 2) ** 2

    @property
    def mass_burnout(self) -> float:
        return self.mass_total - self.mass_propellant

    def mass_at(self, t: float) -> float:
        """Linearly-depleting mass model during boost, constant after."""
        if t >= self.burn_time:
            return self.mass_burnout
        frac = t / self.burn_time
        return self.mass_total - frac * self.mass_propellant

    def mass_rate(self, t: float) -> float:
        if t >= self.burn_time:
            return 0.0
        return -self.mass_propellant / self.burn_time

    def Ixx_at(self, t: float) -> float:
        if t >= self.burn_time:
            return self.Ixx_final
        frac = t / self.burn_time
        return self.Ixx_initial + frac * (self.Ixx_final - self.Ixx_initial)

    def Iyy_at(self, t: float) -> float:
        if t >= self.burn_time:
            return self.Iyy_final
        frac = t / self.burn_time
        return self.Iyy_initial + frac * (self.Iyy_final - self.Iyy_initial)

    def thrust_at(self, t: float) -> float:
        return self.mean_thrust if t < self.burn_time else 0.0


ROCKET_122MM = RocketParams()
