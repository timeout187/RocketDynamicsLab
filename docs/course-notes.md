# Course Notes — RocketDynamicsLab

**A graduate lab in six-degree-of-freedom rigid-body flight dynamics**

## Purpose

This repository is a teaching laboratory built around one required reading:

> M. Khalil, H. Abdalla, O. Kamal, *"Trajectory Prediction for a Typical Fin
> Stabilized Artillery Rocket"*, ASAT-13-FM-04, 13th International Conference
> on Aerospace Sciences & Aviation Technology, Military Technical College,
> Cairo, 2009. (`FM04.pdf`, repo root.)

The paper is short (14 pages) but dense: in one block diagram (its Fig. 1) it
packs a full 6-DOF rigid-body simulation — translational and rotational
dynamics, Euler kinematics, navigation, an atmosphere model, an
aerodynamic-coefficient lookup, and a Monte-Carlo dispersion study. Reading
it once is not enough to *understand* it. This lab exists so you can:

1. Read each equation of the paper next to a plain-language explanation
   ([`equations.md`](equations.md)).
2. Run the exact same equations yourself, in Python, one Euler/RK4 step at a
   time ([`numerical-methods.md`](numerical-methods.md)).
3. Perturb the aerodynamic coefficients, the mass properties, or the wind,
   and *see* the trajectory change, using the Streamlit GUI
   (`src/gui/app.py`).
4. Reproduce the paper's sensitivity/dispersion study (its Table 2 and
   Figs. 10–21) yourself and discuss why each parameter matters
   ([`uncertainty-analysis.md`](uncertainty-analysis.md)).

## What this is *not*

This is **not** an operational trajectory/fire-control engineering tool, and
it is not a faithful bit-for-bit reproduction of the paper's numbers. Several
of the paper's tables (notably Table 1, the Datcom aerodynamic-coefficient
table) are corrupted in the source PDF's text layer — columns of numbers ran
together during OCR/text-extraction and cannot be reliably un-scrambled. We
digitized *representative* coefficients that preserve the reported trend
(the transonic hump around M≈1–1.4, the supersonic falloff) and are
rescaled to numerically well-behaved, textbook-typical magnitudes. Every
place this happens is flagged in the code and in
[`aerodynamic-model.md`](aerodynamic-model.md). Treat all numbers here as a
**fictional teaching dataset**, not design data.

## How the repository is organized

```
docs/                    <- you are here: theory, math, assignments
src/simulator/           <- the physics: frames, EOM, atmosphere, aero, integrators
src/visualization/       <- reusable Plotly figure builders
src/gui/                 <- the Streamlit multipage lab notebook
examples/                <- short standalone scripts (run without Streamlit)
tests/                   <- pytest unit/regression tests for the simulator
assets/                  <- static figures / diagrams used by the docs and GUI
```

## Suggested reading order

1. [`mathematical-model.md`](mathematical-model.md) — the assumptions and the big picture.
2. [`coordinate-systems.md`](coordinate-systems.md) — frames and Euler angles.
3. [`equations.md`](equations.md) — every term of Eqs. (1)–(4) explained.
4. [`numerical-methods.md`](numerical-methods.md) — how we turn ODEs into numbers.
5. [`atmosphere-model.md`](atmosphere-model.md) and [`aerodynamic-model.md`](aerodynamic-model.md) — the two "black boxes" that feed forces/moments into the equations of motion.
6. [`uncertainty-analysis.md`](uncertainty-analysis.md) — the dispersion study.
7. [`assignments.md`](assignments.md) — do the exercises.
8. [`instructor-guide.md`](instructor-guide.md) — if you are teaching this material.

## Running the lab

```bash
pip install -r requirements.txt
streamlit run src/gui/app.py
```

Or, without the GUI, run any of the `examples/*.py` scripts directly.
