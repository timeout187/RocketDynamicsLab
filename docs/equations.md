# The Equations, Term by Term

*Reference: FM04.pdf Eqs. (1)-(4), Fig. 1. Code: `src/simulator/equations_of_motion.py`.*

For every equation below: **math** → **engineering interpretation** →
**assumptions** → **implementation notes** → **numerical considerations**.

---

## Eq. (1): Translational dynamics

```
u̇ = Tx/m − g·sinθ − Qw + Rv
v̇ = Ty/m + g·cosθ·sinφ − Ru + Pw
ẇ = Tz/m + g·cosθ·cosφ − Pv + Qu
```

**Math.** Newton's second law, `F = m·a`, written in a *rotating* (body-fixed)
reference frame. The `-Qw+Rv`-type terms are **not** extra forces — they
appear because differentiating a vector expressed in a rotating frame
introduces `ω × v` (Coriolis-like) terms. `[Tx Ty Tz]` is the total external
force (thrust + aerodynamic) in body axes; `g·sinθ` etc. are gravity
resolved through the current attitude.

**Engineering interpretation.** `u` (axial velocity) is driven by thrust
minus axial drag minus a gravity component that depends on how nose-up the
rocket currently is. `v, w` (lateral/normal velocity) respond to side-force
and normal-force plus gravity components — this is what produces angle of
attack and sideslip.

**Assumptions.** Rigid body (no elastic deformation); mass is *time-varying*
during boost (see `rocket.mass_at(t)`) but this is **not** explicitly a term
of the paper's published Eq. (1) — we include an optional `-（ṁ/m)·u`-type
correction as a modeling note (see Numerical considerations below).

**Implementation notes.** `Tx, Ty, Tz` are assembled as
`thrust + aerodynamic force` inside `state_derivative()`; gravity uses a
constant `g = 9.80665 m/s²` (flat, non-rotating Earth by default — the full
ellipsoidal-gravity model of assumption (d) is a stretch exercise, see
`assignments.md`).

**Numerical considerations.** During boost, `m(t)` decreases roughly
linearly; a variable-mass rigid body technically requires a `ṁV/m`-type
correction to Newton's law (rocket equation logic) — we include it as an
optional term so students can compare with/without it (Exercise 4).

---

## Euler's Equation: Rotational dynamics

```
Ixx·ṗ = L − (Izz−Iyy)·q·r
Iyy·q̇ = M − (Ixx−Izz)·r·p
Izz·ṙ = N − (Iyy−Ixx)·p·q
```

(shown here for the paper's axisymmetric-body case, `Iyy = Izz`,
`Ixy = Iyz = Izx = 0` — the full paper equations retain a nonzero `Izx`
product of inertia for the general case; see FM04.pdf Fig.1's boxed
"Euler's Equation".)

**Math.** Euler's equations for a rotating rigid body: the rate of change of
angular momentum equals applied moment, again with `ω × Iω`-type gyroscopic
coupling terms arising from differentiating in a rotating frame.

**Engineering interpretation.** `L, M, N` (roll, pitch, yaw moments) come
from the fins/body aerodynamics (`aerodynamics.py`). The gyroscopic coupling
terms are why a *spinning* fin-stabilized rocket's pitch and yaw motions are
coupled — a purely pitching disturbance on a spinning body produces a
yawing response and vice versa (**epicyclic motion** / "coning"), a
textbook feature of spin-stabilized projectile dynamics.

**Assumptions.** Axisymmetric mass distribution (`Iyy=Izz`, cross-products
of inertia zero) — valid for a rocket manufactured to be rotationally
symmetric about its axis, which is the paper's Sec. 3 case-study assumption.

**Implementation notes.** `Ixx(t), Iyy(t)` vary linearly during boost (see
`rocket.py`); `Izz` is set equal to `Iyy` per the axisymmetric assumption.

**Numerical considerations.** The gyroscopic terms scale with `p` (spin
rate), which starts large (tens of rad/s) at launch and decays under roll
damping (`Cl_p`). This creates a genuine **fast-slow** system: pitch/yaw
"coning" oscillates at a frequency related to `(Izz−Ixx)·p/Iyy`, which can
be much faster than the trajectory's overall timescale — this is *why* a
small integration timestep is required near launch (see
`numerical-methods.md`, and Exercise 3 — "timestep sensitivity").

---

## Kinematics equation: Euler angle rates

See `coordinate-systems.md` for the full matrix and the gimbal-lock
discussion. Physically: converts *body-frame* angular velocity into the
*rate of change of the attitude description itself*.

---

## Navigation equation: position rates

```
[Ṅ Ė Ḋ]ᵀ = L_BE · [u v w]ᵀ
```

**Math.** Simple rotation of the body-axis velocity vector into the local
geodetic frame.

**Engineering interpretation.** This is literally "where is the rocket
going" — integrate this to get the trajectory (range, drift, altitude) that
Figs. 2-3 of the paper plot.

**Numerical considerations.** `Ḋ` (down-rate) is what the "altitude ≤ 0"
stopping condition (Fig. 1, "Stop Simulation" box) monitors — the impact
event that ends the simulation.

---

## Eq. (3): Earth's rotation effect (optional)

```
[P Q R]ᵀ = [p q r]ᵀ + L_BE⁻¹·[ (ωE+ψ̇)cosλ, 0, -(ωE+ψ̇)sinλ ]ᵀ
```

Adds the Earth's angular velocity (resolved into body axes) to the
*relative* body rates `[p q r]` to get the *total* (inertial) body rates
`[P Q R]` used in the Coriolis/gyroscopic terms above. Disabled by default
(`include_earth_rotation=False`) — this is a stretch-goal fidelity toggle,
see Exercise 6.

## Professor Notes

> A good in-class demo: set `p_muzzle=0` in the GUI's Rocket parameters and
> compare the pitch/yaw trace to the default (spin ≈ 36 rad/s). The
> "coning"/epicyclic wobble should visibly appear only when spin is nonzero
> — a direct, visual demonstration of gyroscopic coupling.

## Student Exercises

See `assignments.md` Exercises 1, 3, 4, 6.
