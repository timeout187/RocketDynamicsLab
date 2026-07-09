# RocketDynamicsLab

[![Tests](https://github.com/timeout187/RocketDynamicsLab/actions/workflows/tests.yml/badge.svg)](https://github.com/timeout187/RocketDynamicsLab/actions/workflows/tests.yml)
[![Python 3.11+](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Live App](https://static.streamlit.io/badges/streamlit_badge_black_white.svg)](https://rocketdynamicslab.streamlit.app/)

**A 6-DOF fin-stabilized artillery rocket flight-dynamics lab**, built as a
teaching companion to a published research paper's model and case study —
for numerical-methods and flight-dynamics education, not operational use.

> **Teaching tool only.** No target-coordinate input, aim correction,
> fire-control capability, or weapon-deployment advice of any kind. Not
> validated for real-world use, and several coefficient values are
> reconstructed teaching data rather than the source paper's exact
> published numbers — see [Assumptions and limitations](#assumptions-and-limitations).

**🚀 [Try the live lab — rocketdynamicslab.streamlit.app](https://rocketdynamicslab.streamlit.app/)**
— nothing to install.

---

## Table of contents

1. [What is this?](#what-is-this)
2. [Key features](#key-features)
3. [Architecture](#architecture)
4. [Physics model](#physics-model)
5. [Project structure](#project-structure)
6. [Installation](#installation)
7. [Usage](#usage)
8. [Validation against the published paper](#validation-against-the-published-paper)
9. [Assumptions and limitations](#assumptions-and-limitations)
10. [Learn more](#learn-more)
11. [References](#references)
12. [Credits](#credits)
13. [License](#license)

---

## What is this?

RocketDynamicsLab simulates the full 6-degree-of-freedom flight of a
122&nbsp;mm unguided, fin-stabilized artillery rocket — position, velocity,
spin, and pitch/yaw motion — from muzzle to ground impact, teaching the
model, case-study data, and dispersion methodology of a real, published
paper (see [Credits](#credits)). It ships with a browser-based GUI covering
ten separate lab topics, a full test suite, and nine graduate-level
assignments, and is explicitly designed to teach **methodology** (the math,
the numerical integration, the software architecture) rather than to
reproduce a real-world case study bit-for-bit.

## Key features

| Feature | Description |
|---|---|
| **Interactive GUI** | 10-page Streamlit lab: Coordinate Frames, State Variables, Forces & Moments, Equations of Motion, Numerical Integrator, Atmosphere, Aerodynamics, Sensitivity Analysis, Visualization, Simulation Results |
| **Real 6-DOF rigid-body dynamics** | Full translational + rotational equations of motion in body axes, not a simplified point-mass model |
| **Mach-indexed aerodynamics** | Axial force, normal force, roll damping, pitch/yaw damping, and pitching-moment slope, all interpolated from a Table-1-style Mach-indexed lookup |
| **US Standard Atmosphere 1976** | Altitude-varying temperature, pressure, density, and speed of sound (troposphere + lower stratosphere) |
| **Three integrators** | Hand-written forward Euler and classical RK4, plus adaptive `scipy.integrate.solve_ivp` (RK45), built for direct side-by-side comparison |
| **Monte Carlo dispersion analysis** | Sweeps the paper's Table 2 uncertainty parameters (launch angle, mass, inertia, thrust, burn time, air density, spin rate, wind...) and reports impact-point spread |
| **Professor Notes / Student Exercises** | Every lab page includes collapsible teaching notes and nine graduate-level assignments in `docs/assignments.md` |
| **CI-tested** | 18 pytest tests (frame transforms, atmosphere physics, integrator convergence order, full-trajectory behavior, dispersion) run on every push |

## Architecture

```mermaid
graph TD
    A["src/gui/app.py<br/>(Streamlit GUI)"] --> B["run_simulation()<br/>(simulate.py)"]
    B --> C["state_derivative()<br/>(equations_of_motion.py - the 6-DOF EOM)"]
    C --> D["Atmosphere.density/mach()<br/>(atmosphere.py)"]
    C --> E["AeroModel.forces_moments()<br/>(aerodynamics.py - Mach lookup)"]
    C --> F["euler_to_LBE() / kinematic_rates()<br/>(frames.py)"]
    B --> G["euler_step() / rk4_step() / solve_ivp<br/>(integrators.py)"]
    H["RocketParams<br/>(rocket.py)"] --> B
    I["monte_carlo_dispersion()<br/>(dispersion.py)"] --> B

    style A fill:#2a78d6,stroke:#184f95,color:#fff
    style B fill:#1baf7a,stroke:#0d5c3d,color:#fff
    style C fill:#e34948,stroke:#8f1e1d,color:#fff
    style E fill:#eda100,stroke:#8a5f00,color:#fff
    style H fill:#4a3aa7,stroke:#2a2060,color:#fff
```

**Data flow**: the GUI (or a script) builds a `RocketParams` instance from
defaults plus any edited inputs → `run_simulation()` dispatches to the
chosen integrator → at every step, `state_derivative()` queries the
atmosphere and aero model for the current altitude/Mach/angle-of-attack,
sums thrust, aerodynamic forces/moments, and gravity, resolves them through
the current attitude via `frames.py`, and returns the 12-element state
derivative → the integrator stops at ground impact (altitude ≤ 0) and
returns the full time history as a `SimulationResult`.

## Physics model

### State vector (12 elements)

| Index | Variable | Description | Frame |
|---|---|---|---|
| 0-2 | u, v, w | Velocity (axial, side, normal) | Body-fixed |
| 3-5 | p, q, r | Angular rates (roll, pitch, yaw) | Body-fixed |
| 6-8 | φ, θ, ψ | Euler angles (roll, pitch/inclination, yaw/azimuth) | — |
| 9-11 | N, E, D | Position (North, East, Down) | Local geodetic (NED) |

Body axes are used because the rocket's moments of inertia (`Ixx, Iyy,
Izz`) are constant in that frame — a rigid body's mass distribution doesn't
change relative to itself as it tumbles. See
[`docs/coordinate-systems.md`](docs/coordinate-systems.md) and
[`docs/equations.md`](docs/equations.md) for the full derivation.

### Translational dynamics (Eq. 1)

```
u_dot = Tx/m - g*sin(theta) - Q*w + R*v
v_dot = Ty/m + g*cos(theta)*sin(phi) - R*u + P*w
w_dot = Tz/m + g*cos(theta)*cos(phi) - P*v + Q*u
```

### Rotational dynamics (Euler's equations, axisymmetric body: Iyy = Izz)

```
p_dot = L / Ixx
q_dot = (M - (Ixx - Izz)*r*p) / Iyy
r_dot = (N - (Iyy - Ixx)*p*q) / Izz
```

The `(Ixx-Izz)*r*p` / `(Iyy-Ixx)*p*q` terms are the **gyroscopic coupling**:
for a spinning fin-stabilized body they couple pitch and yaw into a bounded
"coning"/epicyclic wobble instead of an independent runaway in each plane.
This coupling — and how fast a fixed-step integrator must resolve it near
launch, when spin rate is highest — is the crux of the timestep-sensitivity
assignment; see [`docs/equations.md`](docs/equations.md) and
[`docs/numerical-methods.md`](docs/numerical-methods.md).

### Aerodynamic forces and moments

Given dynamic pressure `q_bar = 0.5*rho*V^2`, reference area `S`, caliber
`D`, angle of attack `alpha`, and sideslip `beta`:

```
Axial force (drag)   = -q_bar * S * CA
Normal force          = -q_bar * S * CN_alpha * alpha
Side force            =  q_bar * S * CN_alpha * beta
Roll moment           =  q_bar * S * D * Cl_p * (p*D / 2V)
Pitch moment          =  q_bar * S * D * (Cm_alpha*alpha + Cmq*q*D/2V)
Yaw moment            =  q_bar * S * D * (Cm_alpha*beta  + Cmq*r*D/2V)
```

`CA`, `CN_alpha`, `Cl_p`, `Cmq`, and `Cm_alpha` are Mach-indexed
coefficients modeled on the source paper's Table 1 (Missile-Datcom-derived
data). **`Cm_alpha` is strongly negative** — the defining feature of a
fin-stabilized (as opposed to spin-stabilized) projectile: the fins create
a strong "weathercock" restoring moment independent of spin rate. See
[`docs/aerodynamic-model.md`](docs/aerodynamic-model.md) for what every
coefficient means physically, and the important caveat on where these
values diverge from the paper's own numbers.

### Atmosphere

| Layer | Altitude | Model |
|---|---|---|
| Troposphere | 0-11 km | Linear lapse rate, `T = 288.15 - 0.0065*h` |
| Lower stratosphere | 11-20 km | Isothermal, `T = 216.65 K` |

Per the **US Standard Atmosphere 1976**; density and speed of sound follow
from the ideal gas law and `a = sqrt(gamma * R * T)`. See
[`docs/atmosphere-model.md`](docs/atmosphere-model.md).

## Project structure

```
FM04.pdf              the required-reading source paper
src/simulator/
  rocket.py            mass/inertia properties, boost vs. free-flight phase
  atmosphere.py        US Standard Atmosphere 1976 (density, sonic speed, Mach)
  aerodynamics.py      Mach-indexed coefficients -> forces/moments
  frames.py            Euler angles, L_BE direction cosine matrix
  equations_of_motion.py   the 6-DOF equations of motion (the physics core)
  integrators.py       Euler, RK4, solve_ivp wrappers, ground-impact event
  simulate.py          orchestration: run_simulation() -> SimulationResult
  dispersion.py        Monte Carlo dispersion sweep (paper's Table 2)
src/visualization/     reusable Plotly figure builders
src/gui/
  app.py               the Streamlit GUI landing page
  pages/                one file per lab topic (10 pages)
docs/
  course-notes.md       syllabus and reading order
  mathematical-model.md the five modeling assumptions, state vector
  coordinate-systems.md  frames, Euler angles, gimbal lock
  equations.md           every term of every equation, explained
  numerical-methods.md   Euler vs. RK4 vs. solve_ivp, convergence, stability
  atmosphere-model.md    the atmosphere layers in full
  aerodynamic-model.md   what each coefficient means, and the data caveat
  uncertainty-analysis.md  Table 2 dispersion methodology
  assignments.md         nine graduate-level exercises
  instructor-guide.md    7-session schedule, rubric, known pitfalls
examples/              four ready-to-run standalone scripts
tests/                 18 tests: frames, atmosphere, integrators, simulate, dispersion
assets/                static, hand-authored diagrams
.github/workflows/     CI (pytest on every push)
```

## Installation

Requires **Python 3.11+**.

```bash
git clone https://github.com/timeout187/RocketDynamicsLab.git
cd RocketDynamicsLab
pip install -r requirements.txt
pytest tests/ -q   # optional: verify the install, ~25s
```

## Usage

### GUI (recommended)

```bash
streamlit run src/gui/app.py
```

Opens at `http://localhost:8501`. Navigate the ten lab pages from the
sidebar; pages 9-10 (Visualization, Simulation Results) share a sidebar
control panel for rocket parameters (elevation angle, mass, thrust, burn
time, muzzle velocity/spin, wind, integrator, timestep) and re-run the
simulation live as you change them. Or just use the hosted version:
**[rocketdynamicslab.streamlit.app](https://rocketdynamicslab.streamlit.app/)**.

### Command line

```bash
python examples/run_nominal_trajectory.py             # the default case, summary output
python examples/timestep_sensitivity.py                # Euler vs. RK4 across six timesteps
python examples/rk4_vs_solve_ivp.py                    # fixed-step RK4 vs. adaptive solve_ivp
python examples/dispersion_sweep.py "Air density"      # one Table-2 dispersion sweep
```

### Python API

```python
import sys; sys.path.insert(0, "src")
from simulator import run_simulation, RocketParams

rocket = RocketParams(mass_total=66.0, mean_thrust=23600.0)  # change anything
result = run_simulation(rocket=rocket, elevation_deg=45.0, dt=0.01, method="rk4")
print(f"time of flight: {result.time_of_flight:.1f} s, range: {result.impact_range:.0f} m")
```

### Tests

```bash
pytest tests/ -q
```

18 tests across coordinate frames, atmosphere physics, integrator
convergence order, full-trajectory behavior, and dispersion — runs in
about 25 seconds, and on every push via GitHub Actions.

## Validation against the published paper

The source paper's own worked example uses a 50° firing angle; running
this simulator at the same angle with its default 122&nbsp;mm case:

| Quantity | Paper (Sec. 3.3, at 50°) | This simulator (50°, reconstructed aero) |
|---|---|---|
| Total flight time | "79 sec" | ~73.4 s |
| Summit time | "nearly 36 sec" | ~22.3 s |
| Muzzle velocity | 26.7 m/s | 26.7 m/s (exact — input parameter) |
| Burn-out velocity | "705 m/s" | comparable order of magnitude (~870 m/s peak) |

**These numbers will not match closely, and that's by design, not a bug.**
The source PDF's Table 1 aerodynamic-coefficient data has a corrupted text
layer (OCR/extraction interleaved its columns beyond reliable recovery), so
this simulator uses a *representative reconstruction* that preserves the
reported Mach-number trend but not the exact values — see
[Assumptions and limitations](#assumptions-and-limitations) and
[`docs/aerodynamic-model.md`](docs/aerodynamic-model.md) for the full
explanation. Reproducing and discussing this exact discrepancy is
Assignment Exercise 7 in [`docs/assignments.md`](docs/assignments.md).

## Assumptions and limitations

**What this model includes:**

- Full 6-DOF rigid-body dynamics (not a point-mass approximation)
- Mach-indexed aerodynamic coefficients modeled on the paper's own Table 1 structure
- Gyroscopic coupling between pitch and yaw (the mechanism behind
  spin-induced "coning"/epicyclic motion)
- Altitude-varying atmosphere (US Standard Atmosphere 1976)
- Monte Carlo dispersion sensitivity analysis (the paper's own Table 2
  uncertainty parameters)
- Time-varying mass and axial/lateral inertia during the boost (propellant-burning) phase

**What this model deliberately omits** (physics simplifications, not
missing features to add later):

- Earth's rotation (Coriolis) and ellipsoidal-Earth geometry — implemented
  as an optional, disabled-by-default toggle (`include_earth_rotation`),
  since it's negligible for this rocket's ~1-2 minute flight time (see
  Assignment Exercise 6)
- Projectile structural flexibility
- Launcher-phase effects (tip-off, launcher deflection) discussed in the
  paper's introduction but not modeled numerically here

**What this project will never include, by design:**

- Target-coordinate input, aim correction, or fire-control solutions
- Live-fire recommendations or artillery firing-table generation
- Any real-world weapon-deployment or targeting capability

**Known data caveat:** every aerodynamic coefficient and case-study number
in this repository should be treated as a **fictional teaching dataset**,
not the paper's actual proprietary design data — see
[Validation](#validation-against-the-published-paper) above.

## Learn more

- **[Project Wiki](https://github.com/timeout187/RocketDynamicsLab/wiki)**
  — Home, Installation, full page-by-page User Manual, Model Overview,
  Equations, and FAQ.
- [`docs/course-notes.md`](docs/course-notes.md) — syllabus and suggested reading order.
- [`docs/mathematical-model.md`](docs/mathematical-model.md) and
  [`docs/equations.md`](docs/equations.md) — the full derivation, mapped to the code.
- [`docs/assignments.md`](docs/assignments.md) — nine graduate-level exercises.
- [`docs/instructor-guide.md`](docs/instructor-guide.md) — a 7-session schedule and grading rubric.

## References

1. Khalil, M., Abdalla, H., and Kamal, O., *"Trajectory Prediction for a
   Typical Fin Stabilized Artillery Rocket"*, 13th International Conference
   on Aerospace Sciences & Aviation Technology (ASAT-13), Paper
   ASAT-13-FM-04, Military Technical College, Cairo, Egypt, May 2009.
   **The paper this project is a teaching companion to.**
2. Etkin, B., *Dynamics of Atmospheric Flight*, John Wiley & Sons, 1972.
   (Cited by [1] as the source of its 6-DOF formulation.)
3. *U.S. Standard Atmosphere, 1976*, jointly published by NOAA/NASA/USAF —
   the standard model implemented in `src/simulator/atmosphere.py`. Not
   cited by [1], which does not publish its own atmospheric model.

Full reference list (all nine works cited by the source paper) in
[`CREDITS.md`](CREDITS.md).

## Credits

This project reimplements the 6-DOF equations of motion, 122&nbsp;mm
case-study data, and Table 1 aerodynamic-coefficient methodology of
reference [1] above as open-source, runnable teaching code with an
interactive GUI — full credit for the underlying research to **Mostafa
Khalil, H. Abdalla, and Osama Kamal** (Egyptian Armed Forces / Military
Technical College, Cairo, Egypt). See [`CREDITS.md`](CREDITS.md) for the
complete citation list and data-provenance notes.

**Built with:** [Streamlit](https://streamlit.io), [Plotly](https://plotly.com),
[SciPy](https://scipy.org), [NumPy](https://numpy.org), and [pandas](https://pandas.pydata.org).

**Project by [timeout187](https://github.com/timeout187)**, built with Claude.

## License

[MIT](LICENSE) — see the license file for the full text and copyright.
