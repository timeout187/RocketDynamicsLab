import numpy as np
import streamlit as st
from common import page_header, professor_notes, student_exercises, default_rocket_sidebar
from visualization.plots import trajectory_3d, multi_time_series

page_header("Visualization", paper_ref="Figs. 2-9")

st.markdown("Every standard trajectory plot in one place. Adjust rocket parameters in the sidebar.")

t, x = default_rocket_sidebar()
u, v, w = x[:, 0], x[:, 1], x[:, 2]
p, q, r = x[:, 3], x[:, 4], x[:, 5]
phi, theta, psi = np.degrees(x[:, 6]), np.degrees(x[:, 7]), np.degrees(x[:, 8])
north, east, down = x[:, 9], x[:, 10], x[:, 11]
altitude = -down
speed = np.sqrt(u**2 + v**2 + w**2)
alpha = np.degrees(np.arctan2(w, np.maximum(np.abs(u), 1e-6)) * np.sign(u))
beta = np.degrees(np.arcsin(np.clip(v / np.maximum(speed, 1e-6), -1, 1)))
accel = np.gradient(speed, t)

st.plotly_chart(trajectory_3d(north, east, altitude), use_container_width=True)

tabs = st.tabs(["Altitude & Range", "Velocity", "Acceleration", "Angular rates",
                "Euler angles", "Aero angles"])
with tabs[0]:
    st.plotly_chart(multi_time_series(t, {"Altitude [m]": altitude}, "Altitude vs time"), use_container_width=True)
    st.plotly_chart(multi_time_series(t, {"North [m]": north, "East [m]": east}, "Range/drift vs time"),
                     use_container_width=True)
with tabs[1]:
    st.plotly_chart(multi_time_series(t, {"u": u, "v": v, "w": w, "Speed": speed}, "Velocity components vs time", "m/s"),
                     use_container_width=True)
with tabs[2]:
    st.plotly_chart(multi_time_series(t, {"Axial accel": accel}, "Acceleration vs time", "m/s^2"),
                     use_container_width=True)
with tabs[3]:
    st.plotly_chart(multi_time_series(t, {"p (roll)": p, "q (pitch)": q, "r (yaw)": r},
                     "Angular rates vs time", "rad/s"), use_container_width=True)
with tabs[4]:
    st.plotly_chart(multi_time_series(t, {"phi (roll)": phi, "theta (pitch)": theta, "psi (yaw)": psi},
                     "Euler angles vs time", "deg"), use_container_width=True)
with tabs[5]:
    st.plotly_chart(multi_time_series(t, {"alpha": alpha, "beta": beta}, "Aerodynamic angles vs time", "deg"),
                     use_container_width=True)

st.subheader("Dispersion concept")
st.markdown("""
A single trajectory is one draw from a distribution of possible outcomes.
The **Sensitivity Analysis** page shows how impact-point scatter (range,
drift) emerges from parameter uncertainty — conceptually, imagine many
overlaid trajectories like this one, each with slightly perturbed inputs,
landing at slightly different points.
""")

professor_notes("""
This page is the natural "put it all together" stop after students have
studied each individual equation/model page — use it for the Fig. 2-9
reproduction assignment (Exercise 7).
""")
student_exercises("See `docs/assignments.md` Exercise 7.")
