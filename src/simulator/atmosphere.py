"""Simple layered atmosphere model.

The paper (Sec. 2, assumption (e)) states: "The atmospheric model is
included where the temperature, sonic speed, and air density are varying
with the body altitude." It does not publish the exact tables used, so this
module implements the standard 1976 US Standard Atmosphere troposphere/lower
stratosphere layers -- a physically faithful, commonly-used stand-in that
lets students reproduce the qualitative behaviour (density decreasing with
altitude, Mach number computed from local sonic speed) discussed in the
paper. See /docs/atmosphere-model.md for the full derivation and the exact
equations used here.
"""
from dataclasses import dataclass

R_AIR = 287.05287        # specific gas constant for dry air, [J/(kg.K)]
GAMMA = 1.4               # specific heat ratio (isentropic flow), from paper nomenclature
G0 = 9.80665              # standard gravity, [m/s^2]

T0 = 288.15               # sea-level temperature, [K]
P0 = 101325.0             # sea-level pressure, [Pa]
LAPSE_TROPO = -0.0065     # [K/m], 0-11000 m
H_TROPO = 11000.0
T_TROPO = T0 + LAPSE_TROPO * H_TROPO   # 216.65 K, isothermal above this to 20000 m
H_STRATO1 = 20000.0


@dataclass
class Atmosphere:
    """Altitude-dependent temperature, pressure, density and sonic speed."""

    sea_level_temp: float = T0
    sea_level_pressure: float = P0

    def temperature(self, h: float) -> float:
        h = max(h, 0.0)
        if h <= H_TROPO:
            return self.sea_level_temp + LAPSE_TROPO * h
        return self.sea_level_temp + LAPSE_TROPO * H_TROPO  # isothermal layer

    def pressure(self, h: float) -> float:
        h = max(h, 0.0)
        if h <= H_TROPO:
            T = self.temperature(h)
            return self.sea_level_pressure * (T / self.sea_level_temp) ** (-G0 / (LAPSE_TROPO * R_AIR))
        p_tropo = self.pressure(H_TROPO)
        return p_tropo * pow(2.718281828459045, -G0 * (h - H_TROPO) / (R_AIR * T_TROPO))

    def density(self, h: float) -> float:
        p = self.pressure(h)
        T = self.temperature(h)
        return p / (R_AIR * T)

    def sonic_speed(self, h: float) -> float:
        T = self.temperature(h)
        return (GAMMA * R_AIR * T) ** 0.5

    def mach(self, h: float, v: float) -> float:
        a = self.sonic_speed(h)
        return v / a if a > 0 else 0.0
