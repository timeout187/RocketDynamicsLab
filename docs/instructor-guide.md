# Instructor Guide

## Course fit

This lab targets a graduate flight dynamics / aerospace systems course
(MSc core or PhD elective). Prerequisites: rigid-body dynamics (Euler's
equations), an ODE course covering numerical integration basics, and
familiarity with Python/NumPy. No prior exposure to missile/rocket
aerodynamics is assumed — `aerodynamic-model.md` and `atmosphere-model.md`
are written to stand alone.

## Suggested schedule (7 sessions, ~2h each)

1. **Mathematical model & assumptions.** Read `mathematical-model.md` and
   FM04.pdf Secs. 1-2 together. Discuss why each of the five modeling
   assumptions is reasonable for this problem and where it would break.
2. **Coordinate frames and Euler angles.** `coordinate-systems.md` +
   Exercise 1(c). Whiteboard the 3-rotation derivation of `L_BE` live.
3. **Equations of motion, term by term.** `equations.md`. Walk through
   `equations_of_motion.py` line-by-line, matching each Python line to a
   term in Eq. (1)/Euler's equation.
4. **Numerical integration.** `numerical-methods.md` + Exercises 2-3 as an
   in-class coding session (pair programming: implement Euler, then RK4).
5. **Atmosphere and aerodynamics.** `atmosphere-model.md` +
   `aerodynamic-model.md`. Use the GUI's **Aerodynamics** page live to
   sweep Mach and discuss the transonic hump.
6. **Sensitivity/dispersion analysis.** `uncertainty-analysis.md` +
   Exercise 8 assigned as a take-home. Discuss OAT vs. global sensitivity
   methods.
7. **Synthesis / figure reproduction.** Exercise 7 presentations — each
   student/group presents their reproduction of one paper figure and
   discusses discrepancies.

## Grading rubric guidance (suggested weights)

- Correctness of derivations (Exercise 1): 15%
- Working RK4 implementation + validation (Exercise 2): 20%
- Timestep/stability analysis with genuine empirical convergence-order
  measurement (Exercise 3): 20%
- Model-assumptions discussion depth (Exercises 4, 6): 15%
- Figure reproduction + discrepancy analysis (Exercise 7): 15%
- Dispersion/Monte Carlo extension (Exercise 8): 15%

Emphasize **numerical rigor** over cosmetic plot-matching: a student who
gets a *different* impact range than the paper but correctly explains *why*
(digitized-coefficient caveat, different elevation angle used in the
worked example, etc.) should score higher than one who does not
acknowledge the discrepancy at all.

## Known pitfalls to warn students about

- **Table 1 digitization caveat.** Make sure students read the caveat in
  `aerodynamic-model.md` before spending hours trying to exactly match the
  paper's numeric figures — the source PDF's coefficient table is
  corrupted by OCR/extraction and this repo's values are a stand-in.
- **Elevation angle mismatch.** The paper's worked trajectory example
  (Figs. 2-9) uses a 50° firing angle; this repo's defaults use 45° unless
  changed. Range/time-of-flight will differ for that reason alone.
- **Gimbal lock at steep launch angles.** Very high elevation angles
  (near 90°) will show numerical artifacts from the Euler-angle kinematic
  singularity (Exercise 5) — this is expected and pedagogically useful,
  not a bug to "fix" by brute force.

## Extending the lab

The codebase is intentionally modular (`rocket.py`, `atmosphere.py`,
`aerodynamics.py`, `frames.py`, `equations_of_motion.py`, `integrators.py`,
`simulate.py`, `dispersion.py`) so instructors can swap in:
- a different rocket's mass/geometry (new `RocketParams`),
- a real Missile Datcom output table (replace `aerodynamics.py`'s tables),
- a higher-fidelity atmosphere (e.g. full 1976 USSA up to 86 km),
- a quaternion-based kinematics module (Exercise 5's stretch goal),

without touching the rest of the pipeline.
