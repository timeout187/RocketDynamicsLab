# Changelog

All notable changes to this project are documented here.
Format loosely follows [Keep a Changelog](https://keepachangelog.com/).

## [0.3.0] — 2026-07-10

### Added
- **General (non-axisymmetric) Euler's Equation** with the Izx cross-inertia
  term (`src/simulator/equations_of_motion.py`, `RocketParams.Izx_initial`/
  `Izx_final`), matching the paper's Fig. 1 "Euler's Equation" box.
  `Izx=0` (the shipped 122mm case study's default) reproduces the prior
  axisymmetric trajectory exactly — verified algebraically and with a
  bit-for-bit regression test.
- **Rotating/curved-Earth Navigation equation** (Coriolis terms,
  R_meridian/R_normal, spherical-Earth simplification) behind the existing
  `include_earth_rotation` flag, replacing the previous flat-NED shortcut
  for that mode.
- Restored pedagogical background sections (coordinate frames, equations of
  motion, forces & moments, numerical integrator comparison) into the
  single-page GUI as expanders.
- `tests/test_equations_of_motion.py` — 5 new tests covering both additions.
- GitHub Pages course site now live at
  `https://timeout187.github.io/RocketDynamicsLab/`, cross-linked with the
  live Streamlit app.

### Fixed
- The RK4-vs-Euler stiffness test and the new full-trajectory tests used a
  step size (`dt=0.01`) that the exact Table 1 coefficients make unstable
  even for RK4 over tens of seconds; switched to `dt=0.002`/`0.003`
  matching `run_simulation()`'s own default.

## [0.2.0] — 2026-07-10

### Changed
- **Rebuilt the GUI as a single-page dashboard** (`src/gui/app.py`), matching
  a sidebar (rocket properties, initial conditions, atmosphere/wind, solver
  settings, dispersion parameters) + editable Table 1 aero data grid
  (upload/reset/download CSV) + Run button + 3D trajectory + 7 time-history
  plots + CSV/JSON export + joint Monte Carlo dispersion sweep. Replaces the
  previous 10-page multipage layout.
- **Aerodynamics now use Table 1's published coefficients directly**
  (`src/simulator/aerodynamics.py`): CA active/passive, CN_alpha, Clp,
  Cm_alpha active/passive, Cmq active/passive, transcribed from the source
  PDF rather than a rescaled reconstruction.
- Added a fin-cant roll-drive coefficient (`RocketParams.fin_cant_coefficient`)
  reproducing the paper's Fig. 7 spin-up behavior during boost — not
  published in Table 1, calibrated and clearly documented as such.
- Default integration timestep reduced to `dt=0.002s` (`run_simulation`,
  examples, tests) — the real Table 1 coefficients make the pitch/yaw
  dynamics numerically stiffer near launch than the earlier rescaled model.

### Validated
- Initial axial acceleration: paper "35.4 g" vs. simulator ~35.7 g (0.8% error).
- Burn-out velocity at t=1.67s: paper "705 m/s" vs. simulator ~717 m/s (1.7% error).
- Summit time: paper "nearly 36 sec" vs. simulator ~36 s.
- See `docs/aerodynamic-model.md` for the full precision-validation writeup,
  including the documented late-flight attitude-behavior limitation.

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
