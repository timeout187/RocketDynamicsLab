import numpy as np
from simulator import run_simulation


def test_nominal_trajectory_runs_and_lands():
    res = run_simulation(elevation_deg=45.0, t_end=120.0, dt=0.01, method="rk4")
    assert np.isfinite(res.x).all()
    assert res.altitude[-1] <= 1.0  # landed (altitude ~0)
    assert res.time_of_flight > 10.0
    assert res.impact_range > 0.0


def test_summit_altitude_positive_and_before_impact():
    res = run_simulation(elevation_deg=45.0, t_end=120.0, dt=0.01, method="rk4")
    assert res.summit_altitude > 0.0
    assert res.summit_time < res.time_of_flight


def test_higher_elevation_generally_higher_summit():
    low = run_simulation(elevation_deg=20.0, t_end=120.0, dt=0.02, method="rk4")
    high = run_simulation(elevation_deg=60.0, t_end=120.0, dt=0.02, method="rk4")
    assert high.summit_altitude > low.summit_altitude


def test_euler_diverges_at_large_dt_rk4_stable():
    euler = run_simulation(elevation_deg=45.0, t_end=20.0, dt=0.05, method="euler")
    rk4 = run_simulation(elevation_deg=45.0, t_end=20.0, dt=0.05, method="rk4")
    assert np.isfinite(rk4.x).all()
    # Euler at this timestep should show much larger magnitude excursions than RK4
    assert np.max(np.abs(euler.x[:, :6])) >= np.max(np.abs(rk4.x[:, :6]))
