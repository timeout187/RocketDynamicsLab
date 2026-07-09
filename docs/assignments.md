# Assignments

Graduate-level (MSc/PhD) exercises built around `FM04.pdf` and this
repository's implementation. Each exercise names the files you'll touch.

## Exercise 1 — Derive the equations

Starting from `F = ma` and `dH/dt = M` (Newton's second law, translational
and rotational forms) in an inertial frame, derive:

(a) the Coriolis-coupling terms in Eq. (1) (`equations.md`) by expressing
the time derivative of a vector in a rotating body frame as
`(dv/dt)_inertial = (dv/dt)_body + ω × v`;

(b) Euler's equations (the paper's rotational-dynamics box) from
`dH/dt = M` with `H = Iω`, showing where the gyroscopic `ω×Iω` terms come
from;

(c) the `L_BE` direction cosine matrix (Eq. 4) as the product of three
elementary rotation matrices in the 3-2-1 (yaw-pitch-roll) sequence.

Show your work; compare your derived matrix element-by-element against
`frames.euler_to_LBE()`.

## Exercise 2 — Implement RK4 from scratch, then compare to `solve_ivp`

Without looking at `integrators.py`, implement your own `rk4_step()`
function. Validate it against a problem with a known analytical solution
(e.g. simple harmonic oscillator, or two-body Kepler orbit) before applying
it to the 6-DOF system. Then:

- Run the full trajectory with your RK4 at `dt = 0.01 s` and with
  `integrate_solve_ivp()` (adaptive RK45) at tight tolerance.
- Plot impact range, drift, and time-of-flight for both.
- Quantify the discrepancy. Is it within the tolerance you'd expect given
  RK4's fixed step vs. RK45's adaptive error control?

## Exercise 3 — Timestep sensitivity and numerical instability

Using `examples/timestep_sensitivity.py` as a starting point:

(a) Run the trajectory with **forward Euler** at `dt ∈ {0.1, 0.05, 0.01,
0.005, 0.001, 0.0005}` s. Plot impact range vs. `dt`. At what `dt` does the
solution visibly diverge (blow up)? Explain *why*, referencing the
gyroscopic coning frequency discussed in `equations.md` and
`numerical-methods.md`.

(b) Repeat with **RK4**. At what `dt` does RK4 diverge? How much larger can
RK4's stable `dt` be than Euler's, for comparable accuracy?

(c) Empirically estimate each method's **order of convergence**: compute
the impact-range error (vs. a very-fine-`dt` reference solution) at several
`dt` values, plot `log(error)` vs. `log(dt)`, and fit a line. Does the slope
match the expected O(dt) for Euler and O(dt⁴) for RK4?

## Exercise 4 — Variable-mass correction

Eq. (1) as published does not show an explicit `ṁ`-dependent correction
term, yet the rocket loses ~30% of its mass during boost. Derive whether a
`-(ṁ/m)·V`-type term *should* appear in a rigorous variable-mass Newton's
law (hint: consider whether the propellant exhaust's momentum is already
fully accounted for by the thrust force `Tx`). Implement both versions in
`equations_of_motion.py` (behind a flag) and compare the resulting
burn-out velocity against the paper's reported 705 m/s at the burn-out
point (Sec. 3.3).

## Exercise 5 — Gimbal lock

Show analytically that the kinematic-equation matrix in
`coordinate-systems.md` is singular at `θ = ±90°`. Construct a launch
scenario (very steep elevation angle) that drives `θ` close to 90° and
observe the numerical behavior of `kinematic_rates()` there. Research and
briefly describe (2-3 paragraphs) how a quaternion attitude representation
avoids this singularity, and what it would take to refactor this codebase
to use one.

## Exercise 6 — Model assumptions and Earth-rotation fidelity

Enable `include_earth_rotation=True` in `state_derivative()` and compare
trajectories with and without it, at different launch latitudes. Given the
rocket's ~1-2 minute time of flight and tens-of-km range, is the
Earth-rotation (Coriolis) effect significant here? Contrast with a
long-range ballistic missile (~30 min flight, thousands of km) where this
effect is well known to matter (e.g. WWII long-range artillery lore).
Discuss which of the paper's five modeling assumptions (`mathematical-model.md`)
you would relax first if asked to build a higher-fidelity model, and why.

## Exercise 7 — Reproduce and discuss the paper's figures

Using the default `ROCKET_122MM` parameters at 45° elevation (the paper
uses 50°), reproduce and compare against the paper:

- Fig. 2/3 — 3D trajectory and altitude vs. time.
- Fig. 4 — velocity magnitude vs. time.
- Fig. 5/6 — axial and normal acceleration vs. time.
- Fig. 7/8 — spin rate and pitch angle vs. time.
- Fig. 9 — aerodynamic angles (α, β, total AoA) vs. time.

For each, note qualitative agreement (shape, key transition points like
burn-out) and explain any quantitative discrepancy, referencing the
aerodynamic-coefficient caveat in `aerodynamic-model.md` (this lab's
coefficients are a representative reconstruction, not the paper's exact
Datcom output).

## Exercise 8 — Full-parameter Monte Carlo dispersion

Extend `monte_carlo_dispersion()` (currently one-parameter-at-a-time) to
perturb **all ten** `DEFAULT_UNCERTAINTIES` parameters simultaneously,
drawing each from an appropriate distribution over its Table-2 range, for
N=200-500 trajectories. Plot the resulting (range, drift) impact-point
scatter. Fit an ellipse (or compute a covariance matrix) and estimate the
50% Circular/Elliptical Error Probable. Compare the *combined* dispersion
to a naive sum of the individual one-at-a-time sensitivities from Exercise
review of Figs. 10-21 — are they consistent? Why or why not?

## Submission format

For each exercise: a short write-up (derivation/discussion, as applicable)
plus a runnable script under `examples/` or a notebook, and any figures
saved to your own scratch directory (do not commit generated figures into
`assets/`, which is reserved for hand-authored teaching diagrams).
