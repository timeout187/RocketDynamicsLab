import streamlit as st
from common import page_header, professor_notes, student_exercises  # noqa: E402

st.set_page_config(page_title="RocketDynamicsLab", page_icon="🚀", layout="wide")

page_header("RocketDynamicsLab", paper_ref="Khalil, Abdalla & Kamal, ASAT-13-FM-04 (2009)")

st.markdown("""
A graduate-level interactive laboratory for **six-degree-of-freedom (6-DOF)
rigid-body flight dynamics**, built around a single required reading: a 2009
paper on trajectory prediction and dispersion analysis for an unguided
fin-stabilized artillery rocket.

This is a **teaching tool**, not an operational engineering application. Use
the sidebar to navigate between lab pages, each covering one piece of the
6-DOF model:
""")

cols = st.columns(2)
with cols[0]:
    st.markdown("""
- **Coordinate Frames** — body vs. geodetic axes, Euler angles
- **State Variables** — the 12-state vector, units, meaning
- **Forces and Moments** — thrust, aerodynamics, gravity
- **Equations of Motion** — Eq. (1)-(2), term by term
- **Numerical Integrator** — Euler vs. RK4 vs. `solve_ivp`
""")
with cols[1]:
    st.markdown("""
- **Atmosphere** — density, sonic speed, Mach number vs. altitude
- **Aerodynamics** — coefficients and what they mean
- **Sensitivity Analysis** — Table 2 dispersion study, reproduced
- **Visualization** — everything plotted at once
- **Simulation Results** — run and inspect a full trajectory
""")

st.divider()
st.subheader("Required reading")
st.markdown("""
`FM04.pdf` (repository root): M. Khalil, H. Abdalla, O. Kamal,
*"Trajectory Prediction for a Typical Fin Stabilized Artillery Rocket"*,
ASAT-13-FM-04, 13th International Conference on Aerospace Sciences &
Aviation Technology, Military Technical College, Cairo, 2009.

See `docs/course-notes.md` for the full syllabus and reading order.
""")

professor_notes("""
This landing page is deliberately light on equations — send students to
`docs/mathematical-model.md` for the full picture, then let them explore
page by page. Consider assigning `docs/assignments.md` Exercise 1 before
the first lab session so students arrive having attempted the derivations
by hand.
""")

student_exercises("""
Before proceeding: skim `FM04.pdf` Sections 1-2 and `docs/mathematical-model.md`.
Note the five modeling assumptions (a)-(e) — you'll be asked to critique them
in Exercise 6.
""")

st.info("⚠️ Some Table 1 aerodynamic-coefficient values could not be reliably "
        "recovered from the source PDF's corrupted text layer and were "
        "reconstructed for teaching purposes. See `docs/aerodynamic-model.md`.")
