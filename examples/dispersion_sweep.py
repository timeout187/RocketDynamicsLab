"""Exercise 8 starter: run a one-parameter-at-a-time dispersion sweep and
print the results (reproduces the paper's Figs. 10-21 workflow numerically).

Usage:
    python examples/dispersion_sweep.py "Air density"
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from simulator.dispersion import DEFAULT_UNCERTAINTIES, monte_carlo_dispersion


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "Air density"
    param = next((p for p in DEFAULT_UNCERTAINTIES if p.name == name), None)
    if param is None:
        print("Available parameters:")
        for p in DEFAULT_UNCERTAINTIES:
            print(f"  - {p.name}")
        return

    values, range_err, drift_err, radial_err = monte_carlo_dispersion(param, n_samples=11)
    print(f"{param.name} sweep ({param.low} to {param.high} {param.unit}):")
    for v, dr, dd, rad in zip(values, range_err, drift_err, radial_err):
        print(f"  {v:8.3f} {param.unit:>4} -> range_err={dr:8.2f} m, drift_err={dd:7.2f} m, radial={rad:7.2f} m")


if __name__ == "__main__":
    main()
