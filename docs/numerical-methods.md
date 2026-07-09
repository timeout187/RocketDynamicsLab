# Numerical Integration

*Code: `src/simulator/integrators.py`.*

The 6-DOF equations of motion (`equations.md`) have no closed-form solution
— they are a coupled, nonlinear, non-autonomous (time-varying mass/thrust)
system of 12 first-order ODEs:

```
dx/dt = f(t, x)
```

We must integrate this numerically. This lab implements three schemes.

## 1. Forward (explicit) Euler

```
x_{n+1} = x_n + Δt · f(t_n, x_n)
```

The simplest possible method: take the current slope, step forward
linearly. **Local truncation error is O(Δt²)**; accumulated over `N = T/Δt`
steps, **global error is O(Δt)** — halving the timestep roughly halves the
total error. Cheap (one function evaluation per step) but requires small
`Δt` for accuracy, and can be numerically **unstable** for stiff/oscillatory
systems (see below) if `Δt` isn't small enough relative to the system's
fastest timescale.

## 2. Classical 4th-order Runge-Kutta (RK4)

```
k1 = f(t, x)
k2 = f(t+Δt/2, x+Δt/2·k1)
k3 = f(t+Δt/2, x+Δt/2·k2)
k4 = f(t+Δt,   x+Δt·k3)
x_{n+1} = x_n + (Δt/6)(k1 + 2k2 + 2k3 + k4)
```

Evaluates the slope at four points per step (start, two midpoint estimates,
end) and combines them with weights matched to a Taylor-series expansion.
**Local error O(Δt⁵), global error O(Δt⁴)** — four evaluations per step buys
you *four extra orders* of accuracy versus Euler's one evaluation. This is
why RK4 is the classical workhorse for this kind of problem, and why the
paper's simulator (and this lab's default) uses it.

Implemented from scratch in `integrators.rk4_step()` — no external ODE
library — specifically so students can read every line.

## 3. Adaptive-step reference: `scipy.integrate.solve_ivp`

`integrate_solve_ivp()` wraps SciPy's `solve_ivp` (default method: Dormand-
Prince RK45), which automatically adjusts its step size to hit a requested
error tolerance (`rtol`, `atol`), taking large steps where the solution is
smooth and small steps where it's rapidly changing (e.g. near launch, where
gyroscopic coning is fast — see `equations.md`). This is the standard
production-grade choice and serves as a **ground truth** to validate the
fixed-step RK4 implementation against (Exercise 2).

## Why timestep matters here specifically

This system has two very different timescales:
- The **trajectory** timescale (seconds to ~100 s: rising, arcing, falling).
- The **coning/nutation** timescale near launch, driven by gyroscopic
  coupling at frequency ~`(Izz-Ixx)·p/Iyy` (see `equations.md`) — this can
  be several rad/s to tens of rad/s, i.e. periods of a fraction of a second.

A fixed-step integrator must resolve the *fastest* relevant timescale even
though we only care about the *slow* trajectory outcome — this is a classic
"stiff-adjacent" numerical methods lesson. Try `dt=0.05` vs `dt=0.001` on
the **Numerical Integrator** GUI page and watch Euler diverge while RK4
stays stable, then push Euler's `dt` down until it also becomes stable — a
direct, hands-on illustration of conditional numerical stability
(Exercise 3).

## Stopping condition (event detection)

The paper's Fig. 1 stops the simulation when altitude ≤ 0 (ground impact).
`integrate_fixed_step()` checks this *after* every step (`stop_event`
callback); `integrate_solve_ivp()` uses SciPy's built-in `events=` mechanism,
which locates the crossing more precisely via root-finding rather than just
checking after a discrete step — another accuracy/engineering trade-off
worth discussing (why does the impact-point estimate change slightly
depending on which method you use to detect the event?).

## Professor Notes

> A satisfying live demo: run the same trajectory with Euler at `dt=0.1,
> 0.01, 0.001` and RK4 at the same three timesteps; plot impact range vs.
> `dt` for both. Euler's error should shrink linearly with `dt`; RK4's
> should shrink roughly as `dt⁴` until roundoff/float64 precision floors it
> out — a genuine empirical order-of-convergence measurement.

## Student Exercises

See `assignments.md` Exercises 2 and 3.
