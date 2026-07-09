"""Exercise 3 starter: compare Euler vs RK4 impact range across timesteps.

Usage:
    python examples/timestep_sensitivity.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from simulator import run_simulation

DTS = [0.1, 0.05, 0.02, 0.01, 0.005, 0.001]


def main():
    print(f"{'dt':>8} | {'Euler range [m]':>16} | {'RK4 range [m]':>14}")
    print("-" * 44)
    for dt in DTS:
        try:
            e = run_simulation(elevation_deg=45.0, t_end=120.0, dt=dt, method="euler")
            e_range = f"{e.impact_range:.1f}"
        except Exception:
            e_range = "diverged"
        r = run_simulation(elevation_deg=45.0, t_end=120.0, dt=dt, method="rk4")
        print(f"{dt:>8} | {e_range:>16} | {r.impact_range:>14.1f}")


if __name__ == "__main__":
    main()
