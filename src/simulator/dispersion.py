"""Monte Carlo dispersion / uncertainty propagation.

Reproduces the sensitivity-analysis workflow of Sec. 3.4 "Dispersion
Analysis" and Table 2 of Khalil et al. (2009): each uncertain input
parameter is perturbed over its published range, the trajectory is
re-simulated, and the resulting scatter in impact range/drift/radial error
is reported. See /docs/uncertainty-analysis.md for the statistical
interpretation (this is a fictional teaching dataset re-using the paper's
published uncertainty *ranges*, not its confidential design data).
"""
from dataclasses import dataclass
from typing import Callable, List
import numpy as np

from .rocket import RocketParams, ROCKET_122MM
from .atmosphere import Atmosphere
from .aerodynamics import AeroModel
from .simulate import run_simulation


@dataclass
class UncertaintyParameter:
    name: str
    unit: str
    low: float   # lower bound of uncertainty range
    high: float  # upper bound of uncertainty range
    apply: Callable  # (rocket, atmo, value) -> (rocket, atmo) modified copies


def _pct(nominal, pct):
    return nominal * (1.0 + pct / 100.0)


def _apply_mass(rocket, atmo, pct):
    r = RocketParams(**{**rocket.__dict__, "mass_total": _pct(rocket.mass_total, pct)})
    return r, atmo


def _apply_propellant_mass(rocket, atmo, pct):
    r = RocketParams(**{**rocket.__dict__, "mass_propellant": _pct(rocket.mass_propellant, pct)})
    return r, atmo


def _apply_burn_time(rocket, atmo, delta_s):
    r = RocketParams(**{**rocket.__dict__, "burn_time": rocket.burn_time + delta_s})
    return r, atmo


def _apply_thrust(rocket, atmo, pct):
    r = RocketParams(**{**rocket.__dict__, "mean_thrust": _pct(rocket.mean_thrust, pct)})
    return r, atmo


def _apply_density(rocket, atmo, pct):
    a = Atmosphere(sea_level_pressure=_pct(atmo.sea_level_pressure, pct))
    return rocket, a


def _apply_Ixx(rocket, atmo, pct):
    r = RocketParams(**{**rocket.__dict__,
                         "Ixx_initial": _pct(rocket.Ixx_initial, pct),
                         "Ixx_final": _pct(rocket.Ixx_final, pct)})
    return r, atmo


def _apply_Iyy(rocket, atmo, pct):
    r = RocketParams(**{**rocket.__dict__,
                         "Iyy_initial": _pct(rocket.Iyy_initial, pct),
                         "Iyy_final": _pct(rocket.Iyy_final, pct)})
    return r, atmo


def _apply_v0(rocket, atmo, pct):
    r = RocketParams(**{**rocket.__dict__, "v_muzzle": _pct(rocket.v_muzzle, pct)})
    return r, atmo


def _apply_spin(rocket, atmo, pct):
    r = RocketParams(**{**rocket.__dict__, "p_muzzle": _pct(rocket.p_muzzle, pct)})
    return r, atmo


# Table 2 of the paper (ranges as published; launch-angle/wind-direction in
# degrees, mass/thrust/time/density in percent, wind speed in m/s).
DEFAULT_UNCERTAINTIES: List[UncertaintyParameter] = [
    UncertaintyParameter("Launching pitch angle", "deg", -2.0, 2.0, None),
    UncertaintyParameter("Rocket total mass", "%", -2.0, 2.0, _apply_mass),
    UncertaintyParameter("Propellant mass", "%", -2.0, 2.0, _apply_propellant_mass),
    UncertaintyParameter("Propellant burning time", "s", -0.1, 0.1, _apply_burn_time),
    UncertaintyParameter("Thrust mean value", "%", -2.0, 2.0, _apply_thrust),
    UncertaintyParameter("Air density", "%", -4.0, 4.0, _apply_density),
    UncertaintyParameter("Axial moment of inertia", "%", -1.0, 1.0, _apply_Ixx),
    UncertaintyParameter("Lateral moment of inertia", "%", -1.0, 1.0, _apply_Iyy),
    UncertaintyParameter("Rocket launch velocity", "%", -2.0, 2.0, _apply_v0),
    UncertaintyParameter("Rocket launch spin rate", "%", -2.0, 2.0, _apply_spin),
]


def monte_carlo_dispersion(param: UncertaintyParameter, n_samples: int = 30,
                            rocket: RocketParams = None, elevation_deg: float = 45.0,
                            seed: int = 0, distribution: str = "uniform"):
    """Sweep one uncertainty parameter over its range and return arrays of
    (values, range_error, drift_error, radial_error) relative to the nominal
    trajectory -- reproducing Figs. 10-21 of the paper.
    """
    rng = np.random.default_rng(seed)
    rocket = rocket or ROCKET_122MM
    atmo = Atmosphere()

    nominal = run_simulation(rocket, atmo, AeroModel(), elevation_deg=elevation_deg)
    R0, D0 = nominal.impact_range, nominal.drift

    if distribution == "uniform":
        values = np.linspace(param.low, param.high, n_samples)
    else:
        values = rng.normal((param.low + param.high) / 2, (param.high - param.low) / 6, n_samples)

    range_err, drift_err, radial_err = [], [], []
    for val in values:
        if param.name == "Launching pitch angle":
            res = run_simulation(rocket, atmo, AeroModel(), elevation_deg=elevation_deg + val)
        else:
            r_mod, a_mod = param.apply(rocket, atmo, val)
            res = run_simulation(r_mod, a_mod, AeroModel(), elevation_deg=elevation_deg)
        dR = res.impact_range - R0
        dD = res.drift - D0
        range_err.append(dR)
        drift_err.append(dD)
        radial_err.append(np.hypot(dR, dD))

    return values, np.array(range_err), np.array(drift_err), np.array(radial_err)


def run_joint_dispersion(rocket: RocketParams = None, aero: AeroModel = None,
                          elevation_deg: float = 50.0, n_samples: int = 200,
                          std_devs: dict = None, seed: int = 0, t_end: float = 150.0,
                          dt: float = 0.01, method: str = "rk4"):
    """Full joint Monte Carlo: draw ALL uncertainty parameters simultaneously
    from independent Gaussians (one std dev per Table 2 parameter) and
    re-simulate, returning per-sample impact range/drift/time-of-flight
    arrays -- the actual dispersion ellipse an accuracy-requirements
    engineer would compute (Exercise 8), rather than one-at-a-time sweeps.

    std_devs: dict with any of the keys below (falls back to Table 2's
    upper bound / 3 if omitted): elevation_deg, mass_pct, propellant_mass_pct,
    burn_time_s, thrust_pct, density_pct, ixx_pct, iyy_pct, v0_pct, spin_pct.
    """
    rng = np.random.default_rng(seed)
    rocket = rocket or ROCKET_122MM
    aero = aero or AeroModel()
    sd = {
        "elevation_deg": 2.0 / 3, "mass_pct": 2.0 / 3, "propellant_mass_pct": 2.0 / 3,
        "burn_time_s": 0.1 / 3, "thrust_pct": 2.0 / 3, "density_pct": 4.0 / 3,
        "ixx_pct": 1.0 / 3, "iyy_pct": 1.0 / 3, "v0_pct": 2.0 / 3, "spin_pct": 2.0 / 3,
    }
    if std_devs:
        sd.update(std_devs)

    ranges, drifts, tofs = [], [], []
    for _ in range(n_samples):
        elev = elevation_deg + rng.normal(0, sd["elevation_deg"])
        r = RocketParams(**{
            **rocket.__dict__,
            "mass_total": _pct(rocket.mass_total, rng.normal(0, sd["mass_pct"])),
            "mass_propellant": _pct(rocket.mass_propellant, rng.normal(0, sd["propellant_mass_pct"])),
            "burn_time": rocket.burn_time + rng.normal(0, sd["burn_time_s"]),
            "mean_thrust": _pct(rocket.mean_thrust, rng.normal(0, sd["thrust_pct"])),
            "Ixx_initial": _pct(rocket.Ixx_initial, rng.normal(0, sd["ixx_pct"])),
            "Ixx_final": _pct(rocket.Ixx_final, rng.normal(0, sd["ixx_pct"])),
            "Iyy_initial": _pct(rocket.Iyy_initial, rng.normal(0, sd["iyy_pct"])),
            "Iyy_final": _pct(rocket.Iyy_final, rng.normal(0, sd["iyy_pct"])),
            "v_muzzle": _pct(rocket.v_muzzle, rng.normal(0, sd["v0_pct"])),
            "p_muzzle": _pct(rocket.p_muzzle, rng.normal(0, sd["spin_pct"])),
        })
        a = Atmosphere(sea_level_pressure=_pct(101325.0, rng.normal(0, sd["density_pct"])))
        res = run_simulation(r, a, aero, elevation_deg=elev, t_end=t_end, dt=dt, method=method)
        ranges.append(res.impact_range)
        drifts.append(res.drift)
        tofs.append(res.time_of_flight)

    return np.array(ranges), np.array(drifts), np.array(tofs)
