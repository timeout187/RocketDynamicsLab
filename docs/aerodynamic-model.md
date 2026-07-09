# Aerodynamic Model

*Reference: FM04.pdf Sec. 3.2 and Table 1. Code: `src/simulator/aerodynamics.py`.*

## Where the coefficients come from

The paper computes its aerodynamic coefficients and derivatives using
**Missile Datcom**, a widely-used semi-empirical aerodynamic prediction code
for missile/rocket-shaped bodies (fin/body/nose geometry in, aerodynamic
coefficients vs. Mach and angle of attack out). This lab does not run
Datcom — it uses a **digitized, representative version of Table 1** as a
static lookup table, interpolated linearly in Mach number.

> **Important caveat.** The text layer of the source PDF (`FM04.pdf`,
> Table 1) is corrupted — its columns of numbers are interleaved/garbled by
> the OCR/extraction process and cannot be reliably separated back into
> the original per-Mach coefficient rows. `aerodynamics.py` therefore uses a
> *reconstruction* that preserves the reported trend (values rising through
> transonic Mach numbers around 1.0–1.4, then decaying supersonically) and
> is rescaled to standard nondimensional-derivative magnitudes so the
> resulting dynamics are numerically well-posed. **Treat every coefficient
> value in this repository as a fictional teaching dataset**, not the
> paper's actual proprietary design data.

## What each coefficient means, physically

| Symbol | Name | Physical meaning |
|---|---|---|
| `CA` | Axial force coefficient | Drag-like force along the body's own x-axis (thrust axis); combines skin friction, wave drag (rises sharply near M=1), and base drag. |
| `CN_alpha` | Normal force curve slope | How strongly a normal (body z-axis) force builds up per radian of angle of attack — analogous to a wing's lift-curve slope `Cl_α`. |
| `Cl_p` | Roll-damping derivative | Restoring roll moment per unit roll rate — always negative (opposes spin), it's what makes rocket spin decay over time absent a spin-inducing fin cant. |
| `Cmq` | Pitch/yaw damping derivative | Restoring moment per unit pitch (or yaw) rate — analogous to an aircraft's pitch damping derivative; negative means stabilizing. |
| `Cm_alpha` | Pitching moment curve slope | How strongly a restoring (or destabilizing) pitching moment builds per radian of angle of attack. **This is the single most important stability parameter** for a fin-stabilized body: a large negative `Cm_alpha` means the nose "weathercocks" strongly back into the relative wind — the entire reason fins are on the back of the rocket rather than the front. |

## From coefficients to forces and moments

With dynamic pressure `q̄ = ½ρV²`, reference area `S` (`π·D²/4`), and
reference length `D` (caliber):

```
Axial force   Tx_aero = -q̄·S·CA
Side force    Ty_aero =  q̄·S·CN_alpha·β
Normal force  Tz_aero = -q̄·S·CN_alpha·α
Roll moment   L  = q̄·S·D · [Cl_p · p·D/(2V)]
Pitch moment  M  = q̄·S·D · [Cm_alpha·α + Cmq·q·D/(2V)]
Yaw moment    N  = q̄·S·D · [Cm_alpha·β + Cmq·r·D/(2V)]
```

The `·D/(2V)` factors are the standard **nondimensionalization** of angular
rate derivatives in aerodynamics — they convert a dimensional rate (rad/s)
into the dimensionless "reduced rate" the coefficient was measured against,
so `Cl_p`, `Cmq` can be tabulated independent of the specific vehicle's
size or speed.

## Small-angle assumption

Forces are linear in `α, β` (`sin α ≈ α`) — valid for the paper's reported
regime (total angle of attack under ~1° after the initial launch transient,
Fig. 9). This breaks down for high-AoA maneuvering flight, which this linear
model cannot represent (see Exercise 7's assumptions discussion).

## Units and Mach interpolation

Mach number `M = V/a(h)` is dimensionless; coefficients are dimensionless
per-radian derivatives (except `CA`, dimensionless force coefficient).
`AeroModel` linearly interpolates each coefficient against the tabulated
Mach breakpoints (`numpy.interp`) — a simple, transparent choice students
can replace with a spline (Exercise: compare interpolation schemes).

## Professor Notes

> On the **Aerodynamics** GUI page, have students sweep Mach from 0.2 to
> 2.2 and watch `Cm_alpha` and `CN_alpha` trace the transonic hump —
> connect this to the physical explanation (shock formation, center-of-
> pressure shift) even though this lab's tables are only representative.

## Student Exercises

See `assignments.md` Exercises 6 and 7.
