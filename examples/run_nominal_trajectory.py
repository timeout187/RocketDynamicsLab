"""Run the nominal 122mm rocket trajectory and print summary output parameters.

Usage:
    python examples/run_nominal_trajectory.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from simulator import run_simulation


def main():
    res = run_simulation(elevation_deg=50.0, t_end=120.0, dt=0.002, method="rk4")
    print(f"Time of flight:     {res.time_of_flight:8.2f} s")
    print(f"Summit altitude:    {res.summit_altitude:8.1f} m  at t={res.summit_time:.2f} s")
    print(f"Impact range:       {res.impact_range:8.1f} m")
    print(f"Drift:              {res.drift:8.2f} m")
    print(f"Max speed:          {res.speed.max():8.1f} m/s")


if __name__ == "__main__":
    main()
