import numpy as np
import streamlit as st
from common import page_header, professor_notes, student_exercises

page_header("Atmosphere Model", paper_ref="Sec. 2 assumption (e), Fig. 1 atmosphere block")

st.markdown("""
Standard 1976 US Standard Atmosphere (troposphere + lower stratosphere) —
see `docs/atmosphere-model.md` for why this stands in for the paper's
unpublished atmosphere table.
""")

from simulator.atmosphere import Atmosphere
from visualization.plots import time_series

atmo = Atmosphere()
h = np.linspace(0, 15000, 300)
T = np.array([atmo.temperature(hh) for hh in h])
rho = np.array([atmo.density(hh) for hh in h])
a = np.array([atmo.sonic_speed(hh) for hh in h])

import plotly.graph_objects as go
c1, c2, c3 = st.columns(3)
with c1:
    fig = go.Figure(go.Scatter(x=T, y=h, mode="lines"))
    fig.update_layout(title="Temperature vs altitude", xaxis_title="T [K]", yaxis_title="Altitude [m]",
                       margin=dict(l=10, r=10, t=40, b=10))
    st.plotly_chart(fig, use_container_width=True)
with c2:
    fig = go.Figure(go.Scatter(x=rho, y=h, mode="lines"))
    fig.update_layout(title="Density vs altitude", xaxis_title="ρ [kg/m^3]", yaxis_title="Altitude [m]",
                       margin=dict(l=10, r=10, t=40, b=10))
    st.plotly_chart(fig, use_container_width=True)
with c3:
    fig = go.Figure(go.Scatter(x=a, y=h, mode="lines"))
    fig.update_layout(title="Sonic speed vs altitude", xaxis_title="a [m/s]", yaxis_title="Altitude [m]",
                       margin=dict(l=10, r=10, t=40, b=10))
    st.plotly_chart(fig, use_container_width=True)

st.subheader("Mach number calculator")
c1, c2 = st.columns(2)
alt = c1.slider("Altitude [m]", 0.0, 15000.0, 2000.0)
v = c2.slider("Velocity [m/s]", 0.0, 900.0, 300.0)
st.metric("Mach number", f"{atmo.mach(alt, v):.3f}")

professor_notes("""
Overlay a real trajectory's altitude profile (from the Simulation Results
page) onto these curves to show students what density/Mach range the
rocket actually experiences during its flight.
""")
student_exercises("See `docs/assignments.md` Exercise 7 discussion of model assumptions.")
