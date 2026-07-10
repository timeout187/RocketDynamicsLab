"""Exercise 3 starter: compare Euler vs RK4 impact range across timesteps.

Usage:
    python examples/timestep_sensitivity.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from simulator import run_simulation

# This system's pitch/yaw dynamics are numerically stiff near launch (fast
# gyroscopic coning driven by the paper's own Table 1 coefficients) -- both
# methods diverge at dt=0.01, RK4 alone stays stable down to dt=0.005-0.001.
DTS = [0.02, 0.01, 0.005, 0.002, 0.001, 0.0005]


def main():
    print(f"{'dt':>8} | {'Euler range [m]':>16} | {'RK4 range [m]':>14}")
    print("-" * 44)
    for dt in DTS:
        e = run_simulation(elevation_deg=45.0, t_end=120.0, dt=dt, method="euler")
        e_range = f"{e.impact_range:.1f}" if abs(e.impact_range) < 1e6 else "diverged"
        r = run_simulation(elevation_deg=45.0, t_end=120.0, dt=dt, method="rk4")
        r_range = f"{r.impact_range:.1f}" if abs(r.impact_range) < 1e6 else "diverged"
        print(f"{dt:>8} | {e_range:>16} | {r_range:>14}")


if __name__ == "__main__":
    main()
