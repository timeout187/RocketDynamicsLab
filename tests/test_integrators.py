import numpy as np
from simulator.integrators import euler_step, rk4_step, integrate_fixed_step, integrate_solve_ivp


def _harmonic(t, x, omega=2.0):
    """dx/dt for x = [pos, vel] of a simple harmonic oscillator, known analytical solution."""
    return np.array([x[1], -omega**2 * x[0]])


def test_rk4_more_accurate_than_euler_on_harmonic_oscillator():
    x0 = np.array([1.0, 0.0])
    t_end = 2 * np.pi  # one full period at omega=2 -> pos should return to x0 after 2*pi/omega*... just check magnitude bound
    dt = 0.05

    _, xs_euler = integrate_fixed_step(_harmonic, x0, 0.0, t_end, dt, method="euler")
    _, xs_rk4 = integrate_fixed_step(_harmonic, x0, 0.0, t_end, dt, method="rk4")

    # energy should be conserved: 0.5*v^2 + 0.5*omega^2*x^2 = const = 0.5*omega^2 (since x0=[1,0])
    omega = 2.0
    energy0 = 0.5 * omega**2 * x0[0]**2
    energy_euler = 0.5 * xs_euler[-1, 1]**2 + 0.5 * omega**2 * xs_euler[-1, 0]**2
    energy_rk4 = 0.5 * xs_rk4[-1, 1]**2 + 0.5 * omega**2 * xs_rk4[-1, 0]**2

    err_euler = abs(energy_euler - energy0)
    err_rk4 = abs(energy_rk4 - energy0)
    assert err_rk4 < err_euler


def test_rk4_convergence_order_on_harmonic_oscillator():
    x0 = np.array([1.0, 0.0])
    t_end = 1.0
    errors = []
    dts = [0.1, 0.05, 0.025]
    for dt in dts:
        _, xs = integrate_fixed_step(_harmonic, x0, 0.0, t_end, dt, method="rk4")
        analytical = np.array([np.cos(2 * t_end), -2 * np.sin(2 * t_end)])
        errors.append(np.linalg.norm(xs[-1] - analytical))

    # halving dt should shrink RK4 error by roughly 2^4=16x (allow generous margin)
    ratio1 = errors[0] / errors[1]
    ratio2 = errors[1] / errors[2]
    assert ratio1 > 5
    assert ratio2 > 5


def test_integrate_solve_ivp_matches_rk4_reasonably():
    x0 = np.array([1.0, 0.0])
    _, xs_rk4 = integrate_fixed_step(_harmonic, x0, 0.0, 5.0, 0.001, method="rk4")
    _, xs_ivp = integrate_solve_ivp(_harmonic, x0, 0.0, 5.0, max_step=0.01)
    assert np.allclose(xs_rk4[-1], xs_ivp[-1], atol=1e-2)
