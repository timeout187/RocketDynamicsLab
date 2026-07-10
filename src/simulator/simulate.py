"""High-level simulation orchestration.

Wires together RocketParams, Atmosphere, AeroModel, the equations of motion,
and an integrator into a single "run_simulation" call producing a tidy
result object -- mirroring the full Fig. 1 block diagram of Khalil et al.
(2009): initial conditions -> forces/moments -> accelerations -> Euler
integration -> navigation -> stop-on-impact -> output parameters.
"""
from dataclasses import dataclass, field
import numpy as np

from .rocket import RocketParams, ROCKET_122MM
from .atmosphere import Atmosphere
from .aerodynamics import AeroModel
from .equations_of_motion import state_derivative, initial_state
from .integrators import integrate_fixed_step, integrate_solve_ivp


@dataclass
class SimulationResult:
    t: np.ndarray
    x: np.ndarray  # shape (n, 12)

    @property
    def u(self): return self.x[:, 0]
    @property
    def v(self): return self.x[:, 1]
    @property
    def w(self): return self.x[:, 2]
    @property
    def p(self): return self.x[:, 3]
    @property
    def q(self): return self.x[:, 4]
    @property
    def r(self): return self.x[:, 5]
    @property
    def phi(self): return self.x[:, 6]
    @property
    def theta(self): return self.x[:, 7]
    @property
    def psi(self): return self.x[:, 8]
    @property
    def north(self): return self.x[:, 9]
    @property
    def east(self): return self.x[:, 10]
    @property
    def down(self): return self.x[:, 11]
    @property
    def altitude(self): return -self.x[:, 11]
    @property
    def speed(self): return np.sqrt(self.u**2 + self.v**2 + self.w**2)

    @property
    def impact_range(self) -> float:
        """Horizontal distance (m) from launch to impact (last sample)."""
        return float(np.hypot(self.north[-1], self.east[-1]))

    @property
    def drift(self) -> float:
        """Cross-range (East) displacement at impact, [m]."""
        return float(self.east[-1])

    @property
    def time_of_flight(self) -> float:
        return float(self.t[-1])

    @property
    def summit_altitude(self) -> float:
        return float(np.max(self.altitude))

    @property
    def summit_time(self) -> float:
        return float(self.t[np.argmax(self.altitude)])


def _ground_impact_event(t, x):
    return (-x[11]) < 0.0 and t > 0.1


def run_simulation(rocket: RocketParams = None, atmo: Atmosphere = None, aero: AeroModel = None,
                    elevation_deg: float = 45.0, azimuth_deg: float = 0.0,
                    wind_ned=(0.0, 0.0, 0.0), t_end: float = 120.0, dt: float = 0.002,
                    method: str = "rk4", include_earth_rotation: bool = False,
                    latitude: float = 0.0) -> SimulationResult:
    """Run a full trajectory simulation, stopping at ground impact (altitude <= 0).

    Default dt=0.002s: this system's pitch/yaw dynamics are numerically stiff
    near launch (fast gyroscopic coning driven by the paper's own Table 1
    coefficients) -- a coarser fixed step can diverge. See
    docs/numerical-methods.md.

    method: "euler" | "rk4" (fixed step, size dt) | "solve_ivp" (adaptive RK45).
    include_earth_rotation: False (default) uses the flat, non-rotating-Earth
        Navigation Equation, matching the paper's case study exactly. True
        switches to the full rotating/curved-Earth mechanization (Coriolis +
        transport-rate terms, extra V_N/V_E/V_D states) -- see
        equations_of_motion.py and docs/coordinate-systems.md.
    latitude: launch-site geodetic latitude [rad], used only when
        include_earth_rotation=True.
    """
    rocket = rocket or ROCKET_122MM
    atmo = atmo or Atmosphere()
    aero = aero or AeroModel()

    x0 = initial_state(rocket, elevation_deg, azimuth_deg,
                        include_earth_rotation=include_earth_rotation)

    def f(t, x):
        return state_derivative(t, x, rocket, atmo, aero, wind_ned=wind_ned,
                                 include_earth_rotation=include_earth_rotation,
                                 latitude=latitude)

    if method == "solve_ivp":
        t, x = integrate_solve_ivp(f, x0, 0.0, t_end, max_step=dt, stop_event=_ground_impact_event)
    else:
        t, x = integrate_fixed_step(f, x0, 0.0, t_end, dt, method=method, stop_event=_ground_impact_event)

    return SimulationResult(t=t, x=x)
