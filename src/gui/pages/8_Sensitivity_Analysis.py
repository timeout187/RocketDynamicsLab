import streamlit as st
from common import page_header, professor_notes, student_exercises
from visualization.plots import dispersion_scatter

page_header("Sensitivity Analysis", paper_ref="Sec. 3.4 Dispersion Analysis, Table 2, Figs. 10-21")

st.markdown("""
Reproduce the paper's one-parameter-at-a-time dispersion sweeps. Pick an
uncertain parameter from Table 2, choose a sample count, and see how the
impact point's range/drift/radial error respond.
""")

from simulator.dispersion import DEFAULT_UNCERTAINTIES, monte_carlo_dispersion

names = [p.name for p in DEFAULT_UNCERTAINTIES]
choice = st.selectbox("Uncertainty parameter (Table 2)", names)
param = next(p for p in DEFAULT_UNCERTAINTIES if p.name == choice)
st.caption(f"Nominal uncertainty range: [{param.low}, {param.high}] {param.unit}")

n_samples = st.slider("Number of samples", 5, 60, 20)
elevation = st.slider("Nominal elevation angle [deg]", 20.0, 70.0, 45.0)

if st.button("Run sensitivity sweep", type="primary"):
    with st.spinner("Running trajectory for each sample point..."):
        values, range_err, drift_err, radial_err = monte_carlo_dispersion(
            param, n_samples=n_samples, elevation_deg=elevation)
    fig = dispersion_scatter(values, range_err, drift_err, radial_err, param.name, param.unit)
    st.plotly_chart(fig, use_container_width=True)
    st.caption("Compare against the corresponding figure in FM04.pdf Sec. 3.4 (Figs. 10-21).")

st.subheader("Table 2 — all uncertainty parameters")
import pandas as pd
df = pd.DataFrame([{"Parameter": p.name, "Low": p.low, "High": p.high, "Unit": p.unit}
                    for p in DEFAULT_UNCERTAINTIES])
st.dataframe(df, use_container_width=True)

professor_notes("""
Run 2-3 parameters back to back (e.g. propellant burning time vs. air
density) and ask students to rank them by impact severity *before* running
— then compare their intuition to the actual sweep, echoing the paper's
Sec. 4 conclusion that motor parameters dominate range error.
""")
student_exercises("This page implements the core loop for `docs/assignments.md` Exercise 8 "
                   "(extend to a full multi-parameter Monte Carlo).")
