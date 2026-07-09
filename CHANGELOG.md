# Changelog

All notable changes to this project are documented here.
Format loosely follows [Keep a Changelog](https://keepachangelog.com/).

## [0.1.0] — 2026-07-09

### Added
- Initial release of the 6-DOF simulator (`src/simulator/`): rocket mass
  properties, layered atmosphere model, Mach-interpolated aerodynamic
  coefficients, translational/rotational equations of motion, coordinate
  frame utilities, three integrators (Euler, RK4, `solve_ivp`), and the
  Table-2 Monte Carlo dispersion module.
- Ten-page Streamlit lab GUI (`src/gui/`) covering coordinate frames, state
  variables, forces and moments, equations of motion, the numerical
  integrator, atmosphere, aerodynamics, sensitivity analysis, visualization,
  and simulation results.
- Full documentation set in `docs/`: course notes, mathematical model,
  equations, coordinate systems, numerical methods, atmosphere and
  aerodynamic models, uncertainty analysis, nine graduate assignments, and
  an instructor guide.
- `examples/` standalone scripts and an 18-test `pytest` suite.
- Project licensing (MIT), `CREDITS.md`, and this changelog.

### Known limitations
- The source paper's Table 1 aerodynamic-coefficient data is unrecoverable
  from the source PDF's corrupted text layer; this release uses a
  representative reconstruction (see `docs/aerodynamic-model.md`).
