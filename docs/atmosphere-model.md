# Atmosphere Model

*Reference: FM04.pdf Sec. 2 assumption (e), Fig. 1 "ATMOSPHERIC MODEL" box.
Code: `src/simulator/atmosphere.py`.*

## What the paper says

> "The atmospheric model is included where the temperature, sonic speed, and
> air density are varying with the body altitude."

The paper does not publish its exact atmosphere tables/equations. This lab
uses the standard **1976 U.S. Standard Atmosphere** troposphere and lower
stratosphere layers — a well-documented, physically faithful, widely-used
model that reproduces the qualitative behavior the paper describes (and is
the de-facto standard reference atmosphere used across aerospace
engineering).

## The equations

Troposphere (0 – 11,000 m), constant lapse rate `L = -0.0065 K/m`:

```
T(h) = T0 + L·h
p(h) = p0 · (T(h)/T0)^(-g0/(L·R))
ρ(h) = p(h) / (R·T(h))
a(h) = sqrt(γ·R·T(h))          [sonic speed]
```

Lower stratosphere (11,000 – 20,000 m), isothermal (`T` constant at 216.65 K):

```
p(h) = p(11000)·exp(-g0·(h-11000) / (R·T(11000)))
```

with `T0 = 288.15 K`, `p0 = 101325 Pa`, `R = 287.05 J/(kg·K)` (specific gas
constant for dry air), `γ = 1.4` (specific heat ratio — this is exactly the
paper's nomenclature entry "Specific heat ratio (γ=1.4 for isentropic
flow)"), `g0 = 9.80665 m/s²`.

## Why each output matters to the 6-DOF model

- **Density `ρ(h)`** feeds directly into dynamic pressure
  `q̄ = ½ρV²`, which scales *every* aerodynamic force and moment
  (`aerodynamic-model.md`). A rocket climbing several km loses substantial
  aerodynamic authority as `ρ` drops — this is part of why angle-of-attack
  behavior differs between the boost phase (low altitude, dense air) and
  near the summit (higher altitude, thinner air).
- **Sonic speed `a(h)`** sets the local **Mach number** `M = V/a`, which is
  the lookup key for every aerodynamic coefficient in Table 1 — the
  transonic "hump" the paper's coefficients show around `M ≈ 1.0-1.4` is a
  real compressibility effect, not a modeling artifact.
- **Temperature `T(h)`** underlies both of the above.

## Units

| Quantity | Symbol | Unit |
|---|---|---|
| Temperature | T | K |
| Pressure | p | Pa |
| Density | ρ | kg/m³ |
| Sonic speed | a | m/s |
| Altitude | h | m |

## Implementation notes

`Atmosphere` is a small stateless class — `temperature(h)`, `pressure(h)`,
`density(h)`, `sonic_speed(h)`, `mach(h, v)`. Altitude is clamped to `≥ 0`
(no below-ground evaluation). The **Air density** uncertainty parameter in
the dispersion study (`uncertainty-analysis.md`) is implemented by
perturbing `sea_level_pressure`, which propagates through to density at
every altitude — a simple, physically direct way to inject a density bias.

## Professor Notes

> Have students plot `ρ(h)` and `a(h)` from 0–8000 m (the paper's Fig. 3
> summit altitude is ~8000 m in the higher-elevation case) on the
> **Atmosphere** GUI page, then overlay the trajectory's actual altitude
> profile to see what density/Mach range the rocket actually experiences.

## Student Exercises

See `assignments.md` Exercise 7 (model assumptions discussion).
