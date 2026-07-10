"""Exercise 2 starter: compare fixed-step RK4 against adaptive solve_ivp (RK45).

Usage:
    python examples/rk4_vs_solve_ivp.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from simulator import run_simulation


def main():
    rk4 = run_simulation(elevation_deg=50.0, t_end=120.0, dt=0.002, method="rk4")
    ivp = run_simulation(elevation_deg=50.0, t_end=120.0, dt=0.01, method="solve_ivp")

    print("RK4 (fixed dt=0.002):")
    print(f"  range={rk4.impact_range:.2f} m, ToF={rk4.time_of_flight:.2f} s, steps={len(rk4.t)}")
    print("solve_ivp (adaptive RK45):")
    print(f"  range={ivp.impact_range:.2f} m, ToF={ivp.time_of_flight:.2f} s, steps={len(ivp.t)}")
    print(f"Range discrepancy: {abs(rk4.impact_range - ivp.impact_range):.3f} m")


if __name__ == "__main__":
    main()
