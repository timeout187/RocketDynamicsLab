import numpy as np
import streamlit as st
from common import page_header, professor_notes, student_exercises

page_header("Numerical Integrator", paper_ref="Fig. 1 signal-flow loop")

st.markdown("""
Compare **forward Euler**, **RK4** (implemented from scratch), and SciPy's
adaptive **`solve_ivp`** (RK45) on the full 6-DOF trajectory. This is a
direct, hands-on version of Assignment Exercises 2 and 3.
""")

c1, c2, c3 = st.columns(3)
dt = c1.select_slider("Fixed-step dt [s]", options=[0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001], value=0.01)
elevation = c2.slider("Elevation angle [deg]", 5.0, 85.0, 45.0)
t_end = c3.slider("Max sim time [s]", 20.0, 150.0, 100.0)

from simulator import run_simulation

with st.spinner("Running Euler, RK4, and solve_ivp..."):
    res_euler = run_simulation(elevation_deg=elevation, t_end=t_end, dt=dt, method="euler")
    res_rk4 = run_simulation(elevation_deg=elevation, t_end=t_end, dt=dt, method="rk4")
    res_ivp = run_simulation(elevation_deg=elevation, t_end=t_end, dt=min(dt, 0.05), method="solve_ivp")

from visualization.plots import integrator_comparison
fig = integrator_comparison({
    "Euler": (res_euler.t, res_euler.altitude),
    "RK4": (res_rk4.t, res_rk4.altitude),
    "solve_ivp (RK45)": (res_ivp.t, res_ivp.altitude),
}, ylabel="Altitude [m]", title=f"Altitude vs. time, dt={dt}s")
st.plotly_chart(fig, use_container_width=True)

cols = st.columns(3)
cols[0].metric("Euler impact range [m]", f"{res_euler.impact_range:.1f}" if len(res_euler.t) else "diverged")
cols[1].metric("RK4 impact range [m]", f"{res_rk4.impact_range:.1f}")
cols[2].metric("solve_ivp impact range [m]", f"{res_ivp.impact_range:.1f}")

if not np.isfinite(res_euler.altitude).all() or np.max(np.abs(res_euler.altitude)) > 1e6:
    st.error("Forward Euler has numerically diverged at this timestep! Try a smaller dt. "
             "This is exactly the instability discussed in docs/numerical-methods.md.")

st.subheader("Convergence check")
st.markdown("Impact range vs. dt, for both fixed-step methods (reference = smallest dt RK4):")
dts = [0.1, 0.05, 0.02, 0.01, 0.005]
if st.button("Run convergence sweep (may take ~10s)"):
    rows = []
    ref = run_simulation(elevation_deg=elevation, t_end=t_end, dt=0.001, method="rk4").impact_range
    for d in dts:
        re_ = run_simulation(elevation_deg=elevation, t_end=t_end, dt=d, method="euler").impact_range
        rr_ = run_simulation(elevation_deg=elevation, t_end=t_end, dt=d, method="rk4").impact_range
        rows.append({"dt": d, "Euler range [m]": re_, "Euler |err|": abs(re_-ref),
                     "RK4 range [m]": rr_, "RK4 |err|": abs(rr_-ref)})
    st.dataframe(rows)
    st.caption("Euler error should shrink roughly linearly with dt; RK4's should shrink much faster (~dt^4) "
               "until roundoff dominates.")

professor_notes("""
This page is designed for a live "break it" demo: push dt up until Euler
visibly diverges (usually well before RK4 does), then discuss why —
gyroscopic coning frequency near launch, per docs/numerical-methods.md.
""")
student_exercises("This page directly implements Exercises 2 and 3 from docs/assignments.md.")
