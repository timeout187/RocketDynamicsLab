import streamlit as st
from common import page_header, professor_notes, student_exercises
from visualization.plots import aero_coefficient_plot

page_header("Aerodynamic Model", paper_ref="Sec. 3.2, Table 1")

st.warning("These coefficients are a **representative reconstruction** of the "
           "paper's Table 1 (its OCR text layer is corrupted). See "
           "`docs/aerodynamic-model.md` for the full caveat.")

from simulator.aerodynamics import AeroModel
aero = AeroModel()

coef = st.selectbox("Coefficient", ["CA", "CN_alpha", "Cl_p", "Cmq", "Cm_alpha"])
table_map = {
    "CA": (aero.CA_t, "Axial force coefficient — drag along body x-axis"),
    "CN_alpha": (aero.CN_alpha_t, "Normal force curve slope — per-radian normal force vs. angle of attack"),
    "Cl_p": (aero.Clp_t, "Roll damping derivative — restoring roll moment per unit roll rate"),
    "Cmq": (aero.Cmq_t, "Pitch/yaw damping derivative — restoring moment per unit pitch/yaw rate"),
    "Cm_alpha": (aero.Cm_alpha_t, "Pitching moment curve slope — static stability derivative"),
}
values, meaning = table_map[coef]
st.caption(meaning)
fig = aero_coefficient_plot(aero.mach, values, coef)
st.plotly_chart(fig, use_container_width=True)

st.subheader("Coefficient table")
import pandas as pd
df = pd.DataFrame({"Mach": aero.mach, "CA": aero.CA_t, "CN_alpha": aero.CN_alpha_t,
                    "Cl_p": aero.Clp_t, "Cmq": aero.Cmq_t, "Cm_alpha": aero.Cm_alpha_t})
st.dataframe(df, use_container_width=True)

st.subheader("What each coefficient means")
st.markdown("""
| Symbol | Meaning | Stability role |
|---|---|---|
| `CA` | Axial (drag-like) force coefficient | Determines deceleration in flight direction |
| `CN_alpha` | Normal force curve slope | Analogous to lift-curve slope; builds normal force with α |
| `Cl_p` | Roll-damping derivative | Always negative — decays spin over time |
| `Cmq` | Pitch/yaw damping derivative | Negative = stabilizing (opposes angular rate) |
| `Cm_alpha` | Pitching moment curve slope | **Key stability parameter** — large negative value = strong weathercock (fin) stability |
""")

professor_notes("""
Connect the transonic hump (M≈1.0-1.4) visible in every coefficient to the
physical phenomenon of shock formation and center-of-pressure shift —
even though these particular numbers are a reconstruction, the *shape* is
a genuine aerodynamic feature worth discussing.
""")
student_exercises("See `docs/assignments.md` Exercises 6-7 and `docs/aerodynamic-model.md`.")
