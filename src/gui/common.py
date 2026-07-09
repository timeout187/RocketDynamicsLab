"""Shared setup and small UI helpers for every Streamlit page."""
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_THIS_DIR)
_REPO_ROOT = os.path.dirname(_SRC_DIR)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import streamlit as st


def page_header(title: str, paper_ref: str = ""):
    st.title(title)
    if paper_ref:
        st.caption(f"📄 Paper reference: {paper_ref}")


def professor_notes(text: str):
    with st.expander("🎓 Professor Notes"):
        st.markdown(text)


def student_exercises(text: str):
    with st.expander("✏️ Student Exercises"):
        st.markdown(text)


@st.cache_data(show_spinner="Running 6-DOF simulation...")
def cached_run_simulation(elevation_deg, azimuth_deg, wind_n, wind_e, wind_d,
                           t_end, dt, method, mass_total, Ixx_i, Ixx_f, Iyy_i, Iyy_f,
                           mean_thrust, burn_time, v_muzzle, p_muzzle):
    from simulator import run_simulation, RocketParams, Atmosphere, AeroModel

    rocket = RocketParams(mass_total=mass_total, Ixx_initial=Ixx_i, Ixx_final=Ixx_f,
                           Iyy_initial=Iyy_i, Iyy_final=Iyy_f, mean_thrust=mean_thrust,
                           burn_time=burn_time, v_muzzle=v_muzzle, p_muzzle=p_muzzle)
    res = run_simulation(rocket=rocket, atmo=Atmosphere(), aero=AeroModel(),
                          elevation_deg=elevation_deg, azimuth_deg=azimuth_deg,
                          wind_ned=(wind_n, wind_e, wind_d), t_end=t_end, dt=dt, method=method)
    return res.t, res.x


def default_rocket_sidebar():
    from simulator import ROCKET_122MM
    st.sidebar.subheader("Rocket parameters")
    r = ROCKET_122MM
    elevation = st.sidebar.slider("Elevation angle [deg]", 5.0, 85.0, 45.0)
    mass_total = st.sidebar.number_input("Total mass [kg]", value=r.mass_total)
    mean_thrust = st.sidebar.number_input("Mean thrust [N]", value=r.mean_thrust)
    burn_time = st.sidebar.number_input("Burn time [s]", value=r.burn_time)
    v_muzzle = st.sidebar.number_input("Muzzle velocity [m/s]", value=r.v_muzzle)
    p_muzzle = st.sidebar.number_input("Muzzle spin rate [rad/s]", value=r.p_muzzle)
    wind_e = st.sidebar.slider("Cross wind (East) [m/s]", -20.0, 20.0, 0.0)
    method = st.sidebar.selectbox("Integrator", ["rk4", "euler", "solve_ivp"], index=0)
    dt = st.sidebar.select_slider("Timestep dt [s]", options=[0.1, 0.05, 0.02, 0.01, 0.005, 0.001], value=0.01)

    t, x = cached_run_simulation(elevation, 0.0, 0.0, wind_e, 0.0, 120.0, dt, method,
                                  mass_total, r.Ixx_initial, r.Ixx_final,
                                  r.Iyy_initial, r.Iyy_final, mean_thrust, burn_time,
                                  v_muzzle, p_muzzle)
    return t, x


def variable_card(symbol: str, meaning: str, source: str, unit: str, effect: str):
    with st.container(border=True):
        st.markdown(f"**{symbol}** — {meaning}")
        cols = st.columns(3)
        cols[0].markdown(f"**Unit**\n\n{unit}")
        cols[1].markdown(f"**Comes from**\n\n{source}")
        cols[2].markdown(f"**Effect on solution**\n\n{effect}")
