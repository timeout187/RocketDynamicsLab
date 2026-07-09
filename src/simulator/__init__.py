"""RocketDynamicsLab core simulator package.

Educational reimplementation of the 6-DOF rigid-body flight dynamics model
described in Khalil, Abdalla & Kamal, "Trajectory Prediction for a Typical
Fin Stabilized Artillery Rocket", ASAT-13-FM-04, 2009 (see /docs and the
source PDF FM04.pdf at the repo root).

This package is a teaching tool, not an operational engineering code. All
default numbers (mass, geometry, aero coefficients) are the paper's published
122mm case-study values, reused here purely as a validation/reference example.
"""

from .rocket import RocketParams, ROCKET_122MM
from .atmosphere import Atmosphere
from .aerodynamics import AeroModel
from .frames import euler_to_LBE, kinematic_rates
from .equations_of_motion import state_derivative
from .integrators import euler_step, rk4_step, integrate_fixed_step, integrate_solve_ivp
from .simulate import run_simulation
from .dispersion import UncertaintyParameter, DEFAULT_UNCERTAINTIES, monte_carlo_dispersion

__all__ = [
    "RocketParams",
    "ROCKET_122MM",
    "Atmosphere",
    "AeroModel",
    "euler_to_LBE",
    "kinematic_rates",
    "state_derivative",
    "euler_step",
    "rk4_step",
    "integrate_fixed_step",
    "integrate_solve_ivp",
    "run_simulation",
    "UncertaintyParameter",
    "DEFAULT_UNCERTAINTIES",
    "monte_carlo_dispersion",
]
