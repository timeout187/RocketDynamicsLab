# RocketDynamicsLab

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/tests-pytest-blue.svg)](tests/)
[![Made with Streamlit](https://img.shields.io/badge/GUI-Streamlit-ff4b4b.svg)](src/gui/app.py)
[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-blue.svg)](requirements.txt)
[![Live App](https://static.streamlit.io/badges/streamlit_badge_black_white.svg)](https://rocketdynamicslab.streamlit.app/)

**A graduate-level, MIT/OpenCourseWare-style interactive laboratory for
six-degree-of-freedom (6-DOF) rigid-body flight dynamics** — built around a
single required reading on trajectory prediction and dispersion analysis for
an unguided fin-stabilized artillery rocket.

> 🚀 **[Try the live lab now →](https://rocketdynamicslab.streamlit.app/)**
> No install required — runs the full interactive GUI in your browser.

> ⚠️ **This is a teaching tool, not an operational engineering application.**
> Every numeric default in this repository is a fictional teaching dataset —
> see [Caveats](#caveats-and-honesty-about-the-data) below.

---

## Table of contents

- [What is this?](#what-is-this)
- [Key features](#key-features)
- [Architecture](#architecture)
- [Physics model](#physics-model)
  - [State vector](#state-vector)
  - [Equation groups](#equation-groups)
- [Quick start](#quick-start)
- [Repository layout](#repository-layout)
- [Documentation](#documentation)
- [Caveats and honesty about the data](#caveats-and-honesty-about-the-data)
- [Contributing](#contributing)
- [License](#license)
- [Credits](#credits)

---

## What is this?

`RocketDynamicsLab` turns a dense 14-page conference paper —

> M. Khalil, H. Abdalla, O. Kamal, *"Trajectory Prediction for a Typical Fin
> Stabilized Artillery Rocket"*, ASAT-13-FM-04, 13th International Conference
> on Aerospace Sciences & Aviation Technology, Military Technical College,
> Cairo, 2009 (`FM04.pdf`, repo root)

— into a full teaching laboratory: readable docs explaining every equation,
a from-scratch Python simulator implementing them, and an interactive
Streamlit GUI so students can *see* what changing a coefficient, a timestep,
or a wind speed actually does to a trajectory.

It exists to teach **methodology** — the math, the numerical integration, the
software architecture behind a 6-DOF flight simulator — not to reproduce a
real-world engineering case study. See [`docs/course-notes.md`](docs/course-notes.md)
for the full framing.

## Key features

- 📖 **Ten markdown course documents** (`docs/`) covering the mathematical
  model, coordinate frames, every term of every equation, numerical methods,
  the atmosphere and aerodynamic models, uncertainty analysis, graduate
  assignments, and an instructor's guide.
- 🧮 **A from-scratch 6-DOF simulator** (`src/simulator/`): translational and
  rotational dynamics, Euler kinematics, navigation equations, a layered
  atmosphere model, and a Mach-interpolated aerodynamic-coefficient model.
- 🔢 **Three numerical integrators** implemented for direct comparison:
  forward Euler, hand-written classical RK4, and adaptive `scipy.solve_ivp`
  (RK45) — built so students can see exactly how each one works.
- 🎲 **A Monte-Carlo/sensitivity dispersion module** reproducing the paper's
  12-parameter uncertainty study (its Table 2, Figs. 10–21).
- 🖥️ **A 10-page interactive Streamlit lab notebook** — Coordinate Frames,
  State Variables, Forces and Moments, Equations of Motion, Numerical
  Integrator, Atmosphere, Aerodynamics, Sensitivity Analysis, Visualization,
  Simulation Results — each page with live widgets, "Professor Notes," and
  "Student Exercises."
- ✅ **A pytest suite** validating frame transforms, atmosphere physics,
  integrator convergence order, and full-trajectory behavior.
- 📝 **Nine graduate-level assignments**, from deriving the equations by hand
  to reproducing the paper's own figures and extending its dispersion study.

## Architecture

```
FM04.pdf                     required reading (source paper)
│
├── docs/                    theory, math, numerics, assignments, instructor guide
│
├── src/
│   ├── simulator/           the physics kernel (no plotting/UI dependencies)
│   │   ├── rocket.py            mass/inertia properties, boost vs. free-flight
│   │   ├── atmosphere.py        density, sonic speed, Mach vs. altitude
│   │   ├── aerodynamics.py      Mach-interpolated coefficients -> forces/moments
│   │   ├── frames.py            Euler angles, L_BE direction cosine matrix
│   │   ├── equations_of_motion.py   dx/dt for the 12-state vector
│   │   ├── integrators.py       Euler, RK4, solve_ivp wrapper
│   │   ├── simulate.py          orchestration: run_simulation() -> SimulationResult
│   │   └── dispersion.py        Table-2 Monte Carlo sensitivity sweeps
│   │
│   ├── visualization/        reusable Plotly figure builders (no simulator dependency)
│   │
│   └── gui/                  Streamlit multipage lab
│       ├── app.py                landing page
│       └── pages/                one file per lab topic
│
├── examples/                 standalone scripts, no Streamlit required
├── tests/                    pytest unit/regression tests
└── assets/                   static, hand-authored diagrams
```

**Design principle:** the physics kernel (`src/simulator/`) has zero
dependency on plotting or UI code, and one file per sub-model (mass
properties, atmosphere, aerodynamics, kinematics, integration). This mirrors
the paper's own Fig. 1 block-diagram structure and lets an instructor swap in
a different rocket, atmosphere, or aerodynamic table without touching the
rest of the pipeline — see `docs/mathematical-model.md`'s Professor Notes.

## Physics model

### State vector

A 12-element state vector drives the whole simulation:

```
x = [u, v, w,       body-axis velocity components        [m/s]
     p, q, r,       body-axis angular rates                [rad/s]
     φ, θ, ψ,       Euler angles (roll, pitch, yaw)          [rad]
     N, E, D]       geodetic position (North/East/Down)      [m]
```

### Equation groups

Four coupled equation groups (matching the paper's Fig. 1 signal-flow
diagram) produce `dx/dt`:

1. **Translational dynamics** (Eq. 1) — `u̇, v̇, ẇ` from thrust, aerodynamic
   forces, and gravity resolved through the current attitude.
2. **Rotational dynamics** (Euler's equations) — `ṗ, q̇, ṙ` from aerodynamic
   moments and gyroscopic coupling through the inertia tensor.
3. **Kinematics equation** — `φ̇, θ̇, ψ̇` (Euler-angle rates) from body rates.
4. **Navigation equation** — `Ṅ, Ė, Ḋ` from body velocity rotated into the
   geodetic frame.

Full derivations, assumptions, and implementation notes for every term live
in [`docs/equations.md`](docs/equations.md) and [`docs/mathematical-model.md`](docs/mathematical-model.md).

## Quick start

**Fastest way in:** open the hosted lab at
**[rocketdynamicslab.streamlit.app](https://rocketdynamicslab.streamlit.app/)**
— nothing to install.

To run it locally instead:

```bash
git clone https://github.com/timeout187/RocketDynamicsLab.git
cd RocketDynamicsLab
pip install -r requirements.txt

# launch the interactive lab
streamlit run src/gui/app.py

# or run a standalone script
python examples/run_nominal_trajectory.py
python examples/timestep_sensitivity.py
python examples/rk4_vs_solve_ivp.py
python examples/dispersion_sweep.py "Air density"

# run the test suite
pytest tests/ -q
```

## Repository layout

See [Architecture](#architecture) above for the annotated tree.

## Documentation

| Doc | Covers |
|---|---|
| [`docs/course-notes.md`](docs/course-notes.md) | Syllabus, reading order, how the repo fits together |
| [`docs/mathematical-model.md`](docs/mathematical-model.md) | The five modeling assumptions, state vector, equation groups |
| [`docs/coordinate-systems.md`](docs/coordinate-systems.md) | Body vs. geodetic frames, Euler angles, gimbal lock |
| [`docs/equations.md`](docs/equations.md) | Every term of every equation, explained |
| [`docs/numerical-methods.md`](docs/numerical-methods.md) | Euler vs. RK4 vs. `solve_ivp`, convergence order, stability |
| [`docs/atmosphere-model.md`](docs/atmosphere-model.md) | Standard atmosphere layers, density/Mach vs. altitude |
| [`docs/aerodynamic-model.md`](docs/aerodynamic-model.md) | What each aerodynamic coefficient means physically |
| [`docs/uncertainty-analysis.md`](docs/uncertainty-analysis.md) | Table 2 dispersion methodology |
| [`docs/assignments.md`](docs/assignments.md) | Nine graduate-level exercises |
| [`docs/instructor-guide.md`](docs/instructor-guide.md) | 7-session schedule, grading rubric, known pitfalls |

Additional narrative documentation, a full user manual, and FAQ also live on
the [project wiki](https://github.com/timeout187/RocketDynamicsLab/wiki).

## Caveats and honesty about the data

The source PDF's Table 1 (Missile-Datcom-derived aerodynamic coefficients)
has a **corrupted text layer** — its columns are interleaved by OCR/text
extraction and cannot be reliably recovered. This lab uses a **representative
reconstruction** that preserves the paper's reported Mach-number trend
(transonic hump around M≈1.0–1.4, supersonic decay) but is rescaled to
numerically well-behaved, textbook-typical magnitudes. **Every coefficient
and case-study number in this repository should be treated as a fictional
teaching dataset**, not the paper's actual proprietary design data. Full
explanation in [`docs/aerodynamic-model.md`](docs/aerodynamic-model.md).

## Contributing

Issues and pull requests are welcome — bug fixes, additional assignments,
higher-fidelity sub-models (quaternion kinematics, a real Datcom table, a
full 1976 USSA atmosphere), or GUI polish are all in scope. Please keep the
"teaching tool, not operational tool" framing intact in anything you add.

## License

Released under the [MIT License](LICENSE).

## Credits

See [`CREDITS.md`](CREDITS.md) for the required-reading citation, referenced
works, and tooling acknowledgments.
