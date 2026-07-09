# The Mathematical Model

*Reference: FM04.pdf, Sec. 2 "Mathematical Model", Fig. 1.*

## 1. What problem is being solved?

Given a rocket's mass properties, its aerodynamic coefficients, its
propulsion profile, and the atmosphere it flies through, predict its
**trajectory**: position, velocity, attitude and angular rates as functions
of time, from launch to ground impact.

This is an **initial value problem (IVP)**: a system of coupled nonlinear
ordinary differential equations (ODEs), integrated forward in time from a
known state at t=0 (the "Initial Conditions" box in Fig. 1) until a stopping
condition is reached (altitude ≤ 0, the "Stop Simulation" box).

## 2. The five modeling assumptions (paper Sec. 2, a–e)

The paper's 6-DOF model rests on five explicit assumptions. Each has a
teaching point:

| # | Assumption | Why it matters |
|---|---|---|
| a | The flying body is **rigid**. | No structural bending/flexing modes — 6 DOF (3 translation + 3 rotation) fully describe the motion. A flexible-body model would need many more states. |
| b | All equations are referred to a **body-fixed frame**. | Moments of inertia are constant in this frame (they'd be time-varying in an inertial frame as the body rotates) — this is *why* body axes are the natural choice for rotational dynamics. |
| c | Aerodynamic coefficients are computed in the **body-fixed frame**. | Forces/moments from tables (Table 1) apply directly to body-axis states without extra rotation. |
| d | The **Earth model** includes ellipsoidal shape, rotation, gravity variation. | For short-range artillery trajectories (tens of km, under two minutes of flight) Earth curvature/rotation effects are small but not always negligible — see `equations_of_motion.py`'s optional `include_earth_rotation` flag and the flat-Earth vs rotating-Earth assignment in `assignments.md`. |
| e | The **atmosphere** varies with altitude (temperature, sonic speed, density). | Aerodynamic forces scale with dynamic pressure `q̄ = ½ρV²` and Mach number `M = V/a`; both change substantially over a multi-km-altitude trajectory, so a constant-atmosphere assumption would be a poor approximation. |

## 3. The state vector

This lab's implementation (`src/simulator/equations_of_motion.py`) uses a
**12-state** vector, matching every quantity that flows through Fig. 1's
signal-flow diagram:

```
x = [u, v, w,        body-axis velocity components         [m/s]
     p, q, r,        body-axis angular rates                [rad/s]
     φ, θ, ψ,        Euler angles (roll, pitch, yaw)          [rad]
     N, E, D]        geodetic position (North/East/Down)      [m]
```

See [`state-variables` page](../src/gui) in the GUI and
[`equations.md`](equations.md) for what produces the *rate* of each of
these 12 quantities.

## 4. The four coupled equation groups (Fig. 1)

1. **Translational dynamics**, Eq. (1): `u̇, v̇, ẇ` from forces (thrust +
   aerodynamic + gravity) and Coriolis-like body-rate coupling terms.
2. **Rotational dynamics** (Euler's equations, the paper's unlabeled "Euler's
   Equation" box) : `ṗ, q̇, ṙ` from aerodynamic moments and gyroscopic
   coupling through the inertia tensor.
3. **Kinematics equation**: `φ̇, θ̇, ψ̇` from body rates — how attitude
   angles evolve given the angular velocity.
4. **Navigation equation**: `Ṅ, Ė, Ḋ` from body velocity rotated into the
   geodetic frame by the direction cosine matrix `L_BE`.

These four groups are *coupled*: forces/moments depend on velocity,
attitude and altitude; velocity and attitude evolve from those same
forces/moments. That coupling is exactly why this must be solved
numerically (Section [`numerical-methods.md`](numerical-methods.md)) rather
than in closed form.

## 5. From equations to code

```
state_derivative(t, x, rocket, atmo, aero, ...) -> dx/dt
```
implements all four groups in one function
(`src/simulator/equations_of_motion.py`), calling out to:

- `rocket.py` — time-varying mass/inertia (boost vs. free-flight phase),
- `atmosphere.py` — density, sonic speed vs. altitude,
- `aerodynamics.py` — Mach-interpolated coefficients → forces/moments,
- `frames.py` — the direction cosine matrix and Euler kinematics.

An integrator (`integrators.py`) then repeatedly evaluates this function to
advance the state in time — see `numerical-methods.md`.

## Professor Notes

> The block-diagram style of Fig. 1 is itself worth teaching: it is exactly
> how you would structure this code in *any* language — a small "physics
> kernel" function (forces/moments → accelerations) wrapped by an
> integration loop, with clearly separated sub-models (atmosphere,
> aerodynamics, mass properties) that could each be swapped out
> independently. This separation-of-concerns is why `src/simulator/` has
> one file per sub-model instead of one monolithic script.

## Student Exercises

See [`assignments.md`](assignments.md) Exercises 1, 6.
