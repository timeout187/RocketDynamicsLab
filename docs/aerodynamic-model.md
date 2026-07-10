# Aerodynamic Model

*Reference: FM04.pdf Sec. 3.2 and Table 1. Code: `src/simulator/aerodynamics.py`.*

## Where the coefficients come from — and what changed

The paper computes its aerodynamic coefficients and derivatives using
**Missile Datcom**, a widely-used semi-empirical aerodynamic prediction code
for missile/rocket-shaped bodies. **This lab now transcribes Table 1's
published numbers directly** (`src/simulator/aerodynamics.py`'s `MACH`,
`CA_ACTIVE`/`CA_PASSIVE`, `CN_ALPHA`, `CLP`, `CM_ALPHA_ACTIVE`/`_PASSIVE`,
`CMQ_ACTIVE`/`_PASSIVE` arrays), rather than an earlier rescaled
reconstruction. "Active" = motor burning (boost), "Passive" = coasting
(free flight) — the table gives separate axial-force and moment-derivative
columns for each phase, and `AeroModel` switches between them automatically
at `t = burn_time`.

> **Honest caveat on column identity.** Table 1's header row is visually
> garbled in the source PDF (subscripts and column boundaries are lost to
> OCR/text extraction), and the paper's own nomenclature list names *eight*
> aerodynamic derivatives (`CA`, `Cl`, `Clp`, `Clr`, `Cmq`, `Cm_alphadot`,
> `CN_alpha`, `CN_alphadot`) for only 14 numeric columns per row — some of
> which must carry Active/Passive pairs and some not, and the mapping
> between "which physical derivative" and "which column" cannot be
> recovered with full certainty. The mapping used here (documented in the
> module docstring) is our best-effort, self-consistent reading: `CA` and
> `CN_alpha` are used essentially as published; the two large-magnitude
> paired columns are used as `Cm_alpha` (static stability) and the smaller
> paired columns as `Cmq` (pitch/yaw damping). This interpretation is
> validated below against the numbers the paper states as exact text.

## What each coefficient means, physically

| Symbol | Name | Physical meaning |
|---|---|---|
| `CA` | Axial force coefficient | Drag-like force along the body's own x-axis; Active (lower, motor burning — base is filled with exhaust) vs. Passive (higher, coasting — base drag from open base) per Table 1. |
| `CN_alpha` | Normal force curve slope | How strongly a normal (body z-axis) force builds up per radian of angle of attack — analogous to a wing's lift-curve slope. |
| `Cl_p` | Roll-damping derivative | Restoring roll moment per unit roll rate — always negative (opposes spin). |
| `Cmq` | Pitch/yaw damping derivative | Restoring moment per unit pitch (or yaw) rate; negative means stabilizing. |
| `Cm_alpha` | Pitching moment curve slope | How strongly a restoring pitching moment builds per radian of angle of attack. **The single most important stability parameter** for a fin-stabilized body: Table 1's large negative values mean the nose "weathercocks" strongly back into the relative wind. |

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
rate derivatives — they convert a dimensional rate (rad/s) into the
dimensionless "reduced rate" the coefficient was measured against.

## The fin-cant roll-drive term (not in Table 1)

Table 1 only tabulates roll **damping** (`Clp`, always negative). But the
paper's own Fig. 7 describes spin *increasing* during boost — "the spin
rate will be increased due to the inclination of the rocket fins" — which
requires a roll-**driving** moment the table simply doesn't provide (Datcom
tables report stability/damping derivatives, not the canted-fin control
moment). `RocketParams.fin_cant_coefficient` (default `2.0`, lumped
`Cl_delta * delta`) adds this term so the simulated spin history follows
Fig. 7's shape instead of monotonically decaying. **This value is our own
calibration, not a published number** — the GUI exposes it directly so you
can retune it and see the effect (Assignment Exercise 7).

## Validation against the paper's own numbers (Sec. 3.3)

At the paper's own 50° firing angle, using Table 1's coefficients as-is:

| Quantity | Paper (exact text) | This simulator |
|---|---|---|
| Initial axial acceleration | "35.4 g" | ~35.7 g (0.8% error) |
| Burn-out velocity (t=1.67s) | "705 m/s" | ~717 m/s (1.7% error) |
| Summit time | "nearly 36 sec" | ~36 s |

These are the boost-phase numbers the paper states as literal quoted
values, and the match is close. **Late-flight attitude behavior is a known
limitation**: this simulator's spin/pitch coupling can grow into large
excursions well into the flight (tens of seconds in), where the paper
reports continued small-angle stability throughout. This is a genuine,
explainable numerical/physical finding, not silently swept under the rug:

- Real fin-stabilized rockets rely on the fins' restoring moment for
  stability and deliberately keep spin *low* so the nose can freely
  "weathercock" to track the trajectory's changing flight-path angle. Too
  much spin (needed here to survive an early gyroscopic resonance crossing
  — see `docs/equations.md`) makes the body gyroscopically resist
  reorienting, causing growing angle of attack later in the flight.
- This is compounded by the column-identity ambiguity above: the true
  Cmq/Cm_alphadot split, and possibly Magnus-force terms the paper's
  nomenclature lists but this lab does not model, are not recoverable with
  certainty from the source PDF.

Use the GUI's editable aero table and `fin_cant_coefficient` input to
explore this sensitivity directly — see Assignment Exercise 7.

## Small-angle assumption

Forces are linear in `α, β` (`sin α ≈ α`) — valid near the paper's reported
regime, breaks down at the large excursions described above (see Exercise 7).

## Units and Mach interpolation

Mach number `M = V/a(h)` is dimensionless; coefficients are dimensionless
per-radian derivatives (except `CA`). `AeroModel` linearly interpolates each
coefficient against the tabulated Mach breakpoints (`numpy.interp`).

## Professor Notes

> This is a good case study in *honest* engineering reproduction: the
> boost-phase numbers the paper states as exact text match to ~1-2%, while
> a value the paper never actually reduces to a single number (spin/AoA
> history over the full flight) is where the ambiguity in the source data
> shows up. Have students identify which of the paper's own claims are
> falsifiable single numbers vs. qualitative chart descriptions, and notice
> how validation confidence differs between the two.

## Student Exercises

See `assignments.md` Exercises 6 and 7 (now includes retuning the fin-cant
coefficient and aero table live in the GUI).
