"""Numerical integration schemes for the 6-DOF ODE system.

Provides a fixed-step explicit Euler method, a fixed-step classical RK4
method (implemented from scratch for teaching purposes), and a wrapper
around SciPy's adaptive-step `solve_ivp` for comparison. See
/docs/numerical-methods.md for truncation-error analysis, stability
discussion, and the RK4-vs-solve_ivp assignment.
"""
import numpy as np
from scipy.integrate import solve_ivp


def euler_step(f, t, x, dt, *args, **kwargs):
    """Forward Euler: x_{n+1} = x_n + dt * f(t_n, x_n). Local error O(dt^2),
    global error O(dt) -- first-order accurate.
    """
    return x + dt * f(t, x, *args, **kwargs)


def rk4_step(f, t, x, dt, *args, **kwargs):
    """Classical 4th-order Runge-Kutta step.

    k1 = f(t, x)
    k2 = f(t + dt/2, x + dt/2 * k1)
    k3 = f(t + dt/2, x + dt/2 * k2)
    k4 = f(t + dt,   x + dt   * k3)
    x_{n+1} = x_n + dt/6 * (k1 + 2 k2 + 2 k3 + k4)

    Local truncation error O(dt^5), global error O(dt^4).
    """
    k1 = f(t, x, *args, **kwargs)
    k2 = f(t + dt / 2, x + dt / 2 * k1, *args, **kwargs)
    k3 = f(t + dt / 2, x + dt / 2 * k2, *args, **kwargs)
    k4 = f(t + dt, x + dt * k3, *args, **kwargs)
    return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def integrate_fixed_step(f, x0, t0, t_end, dt, method="rk4", stop_event=None, *args, **kwargs):
    """Fixed-step integration loop, returning (t_array, x_array).

    method: "euler" or "rk4".
    stop_event: optional callable(t, x) -> bool; integration stops the step
                *after* it becomes True (used e.g. for altitude <= 0, the
                paper's "Stop Simulation / Altitude <= 0" test in Fig. 1).
    """
    step_fn = euler_step if method == "euler" else rk4_step
    ts = [t0]
    xs = [np.array(x0, dtype=float)]
    t, x = t0, np.array(x0, dtype=float)
    while t < t_end:
        step = min(dt, t_end - t)
        x = step_fn(f, t, x, step, *args, **kwargs)
        t = t + step
        ts.append(t)
        xs.append(x.copy())
        if stop_event is not None and stop_event(t, x):
            break
    return np.array(ts), np.array(xs)


def integrate_solve_ivp(f, x0, t0, t_end, max_step=0.05, rtol=1e-6, atol=1e-6,
                          stop_event=None, *args, **kwargs):
    """Adaptive-step reference integration via scipy.integrate.solve_ivp
    (Dormand-Prince RK45 by default), for comparison against the fixed-step
    RK4 implementation.
    """
    def rhs(t, x):
        return f(t, x, *args, **kwargs)

    events = None
    if stop_event is not None:
        def ev(t, x):
            return -1.0 if stop_event(t, x) else 1.0
        ev.terminal = True
        ev.direction = -1
        events = ev

    sol = solve_ivp(rhs, (t0, t_end), np.array(x0, dtype=float), method="RK45",
                     max_step=max_step, rtol=rtol, atol=atol, dense_output=False,
                     events=events)
    return sol.t, sol.y.T
