"""Streamlit GUI for the 6-DOF fin-stabilized artillery rocket simulator.

TEACHING TOOL - reproduces a published research paper's model and case
study for numerical-methods / flight-dynamics education only. NOT for
operational use. NOT validated for real-world fire-control. Contains no
target-coordinate input, aim correction, weapon-deployment advice, or
firing-table generation of any kind.
"""
from __future__ import annotations

import io
import json
import os
import sys

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st

_SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from simulator.rocket import RocketParams, ROCKET_122MM  # noqa: E402
from simulator.atmosphere import Atmosphere  # noqa: E402
from simulator.aerodynamics import AeroModel, MACH, CA_ACTIVE, CA_PASSIVE, CN_ALPHA, CLP, \
    CM_ALPHA_ACTIVE, CM_ALPHA_PASSIVE, CMQ_ACTIVE, CMQ_PASSIVE  # noqa: E402
from simulator.equations_of_motion import state_derivative, initial_state  # noqa: E402
from simulator.integrators import integrate_fixed_step, integrate_solve_ivp  # noqa: E402
from simulator.dispersion import run_joint_dispersion  # noqa: E402

st.set_page_config(page_title="RocketDynamicsLab - 6-DOF Artillery Rocket Simulator", layout="wide")

st.title("RocketDynamicsLab: 6-DOF Fin-Stabilized Artillery Rocket Simulator")
st.caption(
    "Teaching-lab reproduction of Khalil, Abdalla & Kamal (2009), ASAT-13-FM-04 - "
    "for numerical-methods and flight-dynamics education only. Not for operational use. "
    "Not validated for real-world fire-control. This tool has no target-coordinate input, "
    "aim-correction, or weapon-deployment capability of any kind."
)

with st.expander("Coordinate frames and the 12-state vector (background)"):
    st.markdown("""
    Two reference frames matter for this model:

    - **Body-fixed frame (F_B)** - origin at the C.G., x forward along the
      symmetry axis, y right, z down. Moments of inertia are constant here.
    - **Local geodetic / NED frame (F_E)** - origin at launch, x North, y East,
      z Down. Trajectory position and "range/drift" are naturally expressed here.

    Attitude is described by three Euler angles applied in the 3-2-1 (yaw,
    pitch, roll) sequence: **psi** (yaw/azimuth), **theta** (pitch/inclination),
    **phi** (roll/bank).

    The 12-state vector `x = [u,v,w, p,q,r, phi,theta,psi, N,E,D]` fully
    describes the rocket's instantaneous condition:

    | Symbol | Meaning | Unit | Comes from |
    |---|---|---|---|
    | u, v, w | Body-axis velocity (axial, side, normal) | m/s | Translational dynamics (Eq. 1) |
    | p, q, r | Body-axis angular rates (roll, pitch, yaw) | rad/s | Euler's rotational equations |
    | phi, theta, psi | Roll, pitch, yaw Euler angles | rad | Kinematic equation |
    | N, E, D | Geodetic position (North, East, Down) | m | Navigation equation, L_BE . [u v w] |

    Derived (not integrated) quantities: angle of attack `alpha = atan2(w_r, u_r)`,
    sideslip `beta = asin(v_r/V)`, and Mach number `M = V / a(h)` - these are
    recomputed every step from the state above and drive the aerodynamic lookup.

    **Kinematic equation (Euler angle rates):**
    """)
    st.latex(r"""
    \begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix} =
    \begin{bmatrix}1 & \sin\phi\tan\theta & \cos\phi\tan\theta\\
    0 & \cos\phi & -\sin\phi\\
    0 & \sin\phi\sec\theta & \cos\phi\sec\theta\end{bmatrix}
    \begin{bmatrix}P\\Q\\R\end{bmatrix}
    """)
    st.warning("Singular at theta = +-90 deg (gimbal lock) - sec(theta) -> infinity.")

with st.expander("Equations of motion (background)"):
    st.subheader("Eq. (1): Translational dynamics")
    st.latex(r"""
    \dot u = \frac{T_x}{m} - g\sin\theta - Qw + Rv \\
    \dot v = \frac{T_y}{m} + g\cos\theta\sin\phi - Ru + Pw \\
    \dot w = \frac{T_z}{m} + g\cos\theta\cos\phi - Pv + Qu
    """)
    st.markdown("""
    **Engineering meaning:** axial velocity `u` responds to thrust minus drag
    minus a gravity component; lateral/normal velocities `v,w` respond to
    side/normal aerodynamic force plus gravity - together producing angle of
    attack and sideslip. The `-Qw+Rv` etc. terms are *not* extra forces - they
    appear because we differentiate a vector in a **rotating** body frame.
    """)
    st.subheader("Euler's Equation: Rotational dynamics (axisymmetric case)")
    st.latex(r"""
    I_{xx}\dot p = L - (I_{zz}-I_{yy})qr \\
    I_{yy}\dot q = M - (I_{xx}-I_{zz})rp \\
    I_{zz}\dot r = N - (I_{yy}-I_{xx})pq
    """)
    st.markdown("""
    **Engineering meaning:** roll rate `p` responds only to roll moment `L`
    (no coupling, since `Izz=Iyy`); pitch and yaw rates are *gyroscopically
    coupled* through spin `p` - a hallmark of spin-stabilized/fin-stabilized
    projectile dynamics ("coning"/epicyclic motion).
    """)
    st.subheader("Kinematics and Navigation equations")
    st.latex(r"[\dot\phi\ \dot\theta\ \dot\psi]^T = \mathbf{K}(\phi,\theta)\,[P\ Q\ R]^T "
             r"\qquad [\dot N\ \dot E\ \dot D]^T = L_{BE}\,[u\ v\ w]^T")
    st.markdown("""
    **Assumptions behind these equations:**
    - Rigid body (no structural flexing).
    - Axisymmetric mass distribution: `Iyy = Izz`, cross products of inertia zero.
    - Body-fixed frame for aerodynamic coefficients and inertia (constant Ixx,Iyy,Izz
      in this frame, at each instant - though they vary slowly with time
      during boost as propellant burns).
    - Flat, non-rotating Earth by default; full ellipsoidal/rotating Earth is an
      optional fidelity toggle.
    """)

with st.expander("Forces and moments (background)"):
    st.markdown("Total force/moment = **thrust** + **aerodynamic** + **gravity** (resolved via Euler angles).")
    st.latex(r"""
    T_x = -\bar q S\, C_A \qquad
    T_z = -\bar q S\, C_{N\alpha}\,\alpha \qquad
    M = \bar q S D\left(C_{m\alpha}\alpha + C_{mq}\frac{qD}{2V}\right)
    """)
    st.markdown("""
    Dynamic pressure `q_bar = 0.5 * rho * V^2` scales every aerodynamic term,
    which is why altitude (through air density `rho`) and airspeed both matter
    so much to force/moment magnitude - see the Atmosphere background below.
    """)

with st.sidebar:
    st.header("Rocket physical properties")
    caliber_mm = st.number_input("Caliber (mm)", value=122.0, step=1.0)
    length_mm = st.number_input("Length (mm)", value=2870.0, step=10.0)
    mass_total = st.number_input("Total mass incl. propellant (kg)", value=66.0, step=0.5)
    mass_propellant = st.number_input("Propellant mass (kg)", value=20.5, step=0.5)
    burn_time = st.number_input("Propellant burn time (s)", value=1.67, step=0.01, format="%.2f")
    mean_thrust = st.number_input("Mean thrust (N)", value=23600.0, step=100.0)
    ixx_i = st.number_input("Ixx initial - axial inertia (kg.m^2)", value=0.1499, step=0.001, format="%.4f")
    ixx_f = st.number_input("Ixx final - axial inertia (kg.m^2)", value=0.1238, step=0.001, format="%.4f")
    iyy_i = st.number_input("Iyy=Izz initial - transverse inertia (kg.m^2)", value=41.58, step=0.1)
    iyy_f = st.number_input("Iyy=Izz final - transverse inertia (kg.m^2)", value=33.83, step=0.1)
    fin_cant = st.number_input(
        "Fin-cant roll-drive coefficient", value=2.0, step=0.1,
        help="Not published in Table 1 -- the paper's own Fig. 7 shows spin *increasing* "
             "during boost from canted fins, which requires a roll-driving term the table "
             "doesn't provide. Calibrated here; see docs/aerodynamic-model.md.")

    st.header("Initial conditions")
    v0 = st.number_input("Muzzle velocity (m/s)", value=26.7, step=0.1)
    spin0_rps = st.number_input("Muzzle spin rate (rev/s)", value=5.8, step=0.1)
    elev_deg = st.number_input("Firing elevation angle (deg)", value=50.0, step=0.5)
    azim_deg = st.number_input("Azimuth angle (deg)", value=0.0, step=0.5)

    st.header("Atmosphere / wind")
    wind_n = st.number_input("Wind - north component (m/s)", value=0.0, step=0.5)
    wind_e = st.number_input("Wind - east component (m/s)", value=0.0, step=0.5)

    st.header("Numerical solver settings")
    method = st.selectbox("Integration method", ["RK4", "Euler", "solve_ivp (RK45)"], index=0)
    t_max = st.number_input("Max flight time (s)", value=150.0, step=5.0)
    dt = st.number_input(
        "Fixed / max step size (s)", value=0.002, step=0.0005, format="%.4f",
        help="This system's pitch/yaw dynamics are numerically stiff near launch (fast "
             "gyroscopic coning) -- dt above ~0.005s can diverge. See docs/numerical-methods.md.")

    st.header("Dispersion / uncertainty parameters")
    st.caption(
        "Defaults are Table 2 of Khalil, Abdalla & Kamal (2009) - the paper's own "
        "ten uncertainty parameters, drawn here as independent Gaussians for a joint "
        "Monte Carlo sweep (see docs/uncertainty-analysis.md)."
    )
    run_dispersion = st.checkbox("Run dispersion sensitivity sweep", value=False)
    n_samples = st.number_input("Monte Carlo samples", value=150, step=10, min_value=10)
    v0_std_pct = st.number_input("Muzzle velocity std dev (%)", value=2.0 / 3, step=0.1)
    elev_std = st.number_input("Firing pitch angle std dev (deg)", value=2.0 / 3, step=0.05)
    mass_std_pct = st.number_input("Rocket total mass std dev (%)", value=2.0 / 3, step=0.1)
    prop_mass_std_pct = st.number_input("Propellant mass std dev (%)", value=2.0 / 3, step=0.1)
    burn_time_std = st.number_input("Burn time std dev (s)", value=0.1 / 3, step=0.01, format="%.3f")
    thrust_std_pct = st.number_input("Mean thrust std dev (%)", value=2.0 / 3, step=0.1)
    density_std_pct = st.number_input("Air density std dev (%)", value=4.0 / 3, step=0.1)
    ixx_std_pct = st.number_input("Axial moment of inertia std dev (%)", value=1.0 / 3, step=0.1)
    iyy_std_pct = st.number_input("Lateral moment of inertia std dev (%)", value=1.0 / 3, step=0.1)
    spin_std_pct = st.number_input("Muzzle spin rate std dev (%)", value=2.0 / 3, step=0.1)


with st.expander("Numerical integrator (background)"):
    st.markdown("""
    This lab compares **forward Euler**, **RK4** (implemented from scratch), and
    SciPy's adaptive **`solve_ivp`** (RK45) on the full 6-DOF trajectory.

    - **Forward Euler** is a first-order method: cheap per step, but its local
      error is O(dt^2) and it can go numerically unstable well before RK4 does.
      This system's pitch/yaw dynamics are stiff near launch (fast gyroscopic
      coning), so Euler needs a much smaller `dt` to stay stable.
    - **RK4** is fourth-order: its error shrinks roughly as `dt^4`, so it stays
      accurate and stable at step sizes that make Euler diverge.
    - **`solve_ivp` (RK45)** adapts its step size automatically to hit a target
      error tolerance, trading some control over the step schedule for
      convenience and robustness.

    If you push `dt` up and switch to Euler, watch for the trajectory going
    numerically unstable (altitude or velocity blowing up) - a direct,
    hands-on illustration of numerical stability limits.
    """)

with st.expander("Atmosphere model (background)"):
    st.markdown("""
    Standard 1976 US Standard Atmosphere (troposphere + lower stratosphere) is
    used here as a stand-in for the paper's unpublished atmosphere table. It
    provides temperature, density, and sonic speed as functions of altitude,
    which in turn set the dynamic pressure `q_bar = 0.5 * rho * V^2` and the
    Mach number `M = V / a(h)` used to look up aerodynamic coefficients.
    Density falls off with altitude, so aerodynamic forces and moments weaken
    as the rocket climbs even at constant airspeed.
    """)


def build_rocket() -> RocketParams:
    return RocketParams(
        caliber=caliber_mm / 1000.0, length=length_mm / 1000.0,
        mass_total=mass_total, mass_propellant=mass_propellant, burn_time=burn_time,
        mean_thrust=mean_thrust, Ixx_initial=ixx_i, Ixx_final=ixx_f,
        Iyy_initial=iyy_i, Iyy_final=iyy_f, v_muzzle=v0, p_muzzle=spin0_rps * 2 * np.pi,
        fin_cant_coefficient=fin_cant,
    )


st.subheader("Aerodynamic coefficient table (Table 1) - editable")
st.caption(
    "**This table accepts direct edits.** Double-click any cell to change it, or use the "
    "+/- row controls to add or remove Mach points - your changes are used the next time "
    "you click **Run simulation**. Defaults are Table 1 from Khalil, Abdalla & Kamal (2009), "
    "\"Trajectory Prediction for a Typical Fin Stabilized Artillery Rocket\" (122mm rocket, "
    "computed with Missile Datcom); Active = motor burning, Passive = coasting. See "
    "docs/aerodynamic-model.md for the column mapping and sign conventions."
)


def _default_aero_df() -> pd.DataFrame:
    return pd.DataFrame({
        "Mach": MACH,
        "CA_active": CA_ACTIVE, "CA_passive": CA_PASSIVE,
        "CN_alpha": CN_ALPHA, "Clp": CLP,
        "Cm_alpha_active": CM_ALPHA_ACTIVE, "Cm_alpha_passive": CM_ALPHA_PASSIVE,
        "Cmq_active": CMQ_ACTIVE, "Cmq_passive": CMQ_PASSIVE,
    })


if "aero_df" not in st.session_state:
    st.session_state.aero_df = _default_aero_df()

upload_col, reset_col, download_col = st.columns([2, 1, 1])
with upload_col:
    uploaded_aero_csv = st.file_uploader("Load your own table (CSV, same column headers)", type="csv")
    if uploaded_aero_csv is not None:
        st.session_state.aero_df = pd.read_csv(uploaded_aero_csv)
with reset_col:
    st.write("")
    if st.button("Reset to paper defaults"):
        st.session_state.aero_df = _default_aero_df()
with download_col:
    st.write("")
    st.download_button("Download current table", st.session_state.aero_df.to_csv(index=False),
                        file_name="aero_table.csv")

edited_aero_df = st.data_editor(st.session_state.aero_df, num_rows="dynamic", key="aero_table_editor")
st.session_state.aero_df = edited_aero_df

with st.expander("What each aerodynamic coefficient means"):
    st.markdown("""
    | Symbol | Meaning | Stability role |
    |---|---|---|
    | `CA` | Axial (drag-like) force coefficient | Determines deceleration in flight direction |
    | `CN_alpha` | Normal force curve slope | Analogous to lift-curve slope; builds normal force with alpha |
    | `Clp` | Roll-damping derivative | Always negative - decays spin over time |
    | `Cmq` | Pitch/yaw damping derivative | Negative = stabilizing (opposes angular rate) |
    | `Cm_alpha` | Pitching moment curve slope | **Key stability parameter** - large negative value = strong weathercock (fin) stability |

    Active vs. passive columns correspond to motor burning vs. coasting flight;
    look for a transonic hump (M ~ 1.0-1.4) in most of these curves - that is
    a genuine aerodynamic feature (shock formation and center-of-pressure
    shift), even though these particular numbers are a reconstruction of the
    paper's Table 1 (see docs/aerodynamic-model.md).
    """)


def build_aero() -> AeroModel:
    df = edited_aero_df
    return AeroModel(
        mach=df["Mach"].to_numpy(), ca_active=df["CA_active"].to_numpy(),
        ca_passive=df["CA_passive"].to_numpy(), cn_alpha=df["CN_alpha"].to_numpy(),
        clp=df["Clp"].to_numpy(), cm_alpha_active=df["Cm_alpha_active"].to_numpy(),
        cm_alpha_passive=df["Cm_alpha_passive"].to_numpy(), cmq_active=df["Cmq_active"].to_numpy(),
        cmq_passive=df["Cmq_passive"].to_numpy(),
    )


def _stop_event(t, x):
    return (-x[11]) < 0.0 and t > 0.1


def run_case(rocket, aero, atmo, elev_deg, azim_deg, wind_ned, t_max, dt, method):
    x0 = initial_state(rocket, elev_deg, azim_deg)

    def f(t, x):
        return state_derivative(t, x, rocket, atmo, aero, wind_ned=wind_ned)

    if method == "solve_ivp (RK45)":
        t, x = integrate_solve_ivp(f, x0, 0.0, t_max, max_step=dt, stop_event=_stop_event)
    else:
        m = "euler" if method == "Euler" else "rk4"
        t, x = integrate_fixed_step(f, x0, 0.0, t_max, dt, method=m, stop_event=_stop_event)
    return t, x


run_clicked = st.button("Run simulation", type="primary")

if run_clicked:
    rocket = build_rocket()
    aero = build_aero()
    atmo = Atmosphere()

    with st.spinner("Integrating 6-DOF equations of motion..."):
        t, x = run_case(rocket, aero, atmo, elev_deg, azim_deg, (wind_n, wind_e, 0.0), t_max, dt, method)

    downrange = x[:, 9]
    crossrange = x[:, 10]
    altitude = -x[:, 11]
    u, v, w = x[:, 0], x[:, 1], x[:, 2]
    p, q, r = x[:, 3], x[:, 4], x[:, 5]
    theta_deg = np.degrees(x[:, 7])
    p_rps = p / (2 * np.pi)
    vel_mag = np.sqrt(u**2 + v**2 + w**2)
    alpha_deg = np.degrees(np.arctan2(w, u))
    accel_axial = np.gradient(u, t)
    accel_normal = np.gradient(np.sqrt(v**2 + w**2), t)

    data = {"t": t, "x": downrange, "y": crossrange, "z": altitude, "u": u, "v": v, "w": w,
            "p": p, "q": q, "r": r, "theta_deg": theta_deg, "p_rps": p_rps,
            "vel_mag": vel_mag, "alpha_deg": alpha_deg}

    st.success(f"Integration finished. Time of flight: {t[-1]:.2f} s, range: {downrange[-1]:.1f} m.")

    col1, col2 = st.columns(2)
    with col1:
        st.metric("Time of flight (s)", f"{t[-1]:.2f}")
        st.metric("Range (m)", f"{downrange[-1]:.1f}")
    with col2:
        st.metric("Cross-range / drift (m)", f"{crossrange[-1]:.2f}")
        st.metric("Max altitude (m)", f"{np.max(altitude):.1f}")

    st.subheader("3D trajectory")
    fig3d = go.Figure(data=[go.Scatter3d(x=downrange, y=crossrange, z=altitude, mode="lines",
                                          line=dict(width=4))])
    fig3d.update_layout(scene=dict(xaxis_title="Downrange (m)", yaxis_title="Cross-range (m)",
                                    zaxis_title="Altitude (m)"), height=600)
    st.plotly_chart(fig3d, use_container_width=True)
    with st.expander("About these plots (Figs. 2-9 reproduction)"):
        st.markdown("""
        This run reproduces every standard trajectory plot the paper's Figs. 2-9
        show, in one place: 3D trajectory, altitude/velocity/acceleration vs.
        time, pitch angle, spin rate, and total angle of attack. A single
        trajectory here is one draw from a distribution of possible outcomes -
        the dispersion sweep below shows how impact-point scatter (range,
        drift) emerges from parameter uncertainty; conceptually, imagine many
        overlaid trajectories like this one, each with slightly perturbed
        inputs, landing at slightly different points.
        """)

    plots = [
        ("Altitude vs flight time", t, altitude, "Time (s)", "Altitude (m)"),
        ("Velocity magnitude vs flight time", t, vel_mag, "Time (s)", "Velocity (m/s)"),
        ("Axial acceleration vs flight time", t, accel_axial, "Time (s)", "Axial accel (m/s^2)"),
        ("Normal acceleration vs flight time", t, accel_normal, "Time (s)", "Normal accel (m/s^2)"),
        ("Pitch angle vs flight time", t, theta_deg, "Time (s)", "Pitch angle (deg)"),
        ("Spin rate vs flight time", t, p_rps, "Time (s)", "Spin rate (rev/s)"),
        ("Total angle of attack vs flight time", t, alpha_deg, "Time (s)", "Alpha total (deg)"),
    ]
    cols = st.columns(2)
    for i, (title, xdat, ydat, xlabel, ylabel) in enumerate(plots):
        fig = go.Figure(data=go.Scatter(x=xdat, y=ydat, mode="lines"))
        fig.update_layout(title=title, xaxis_title=xlabel, yaxis_title=ylabel, height=350)
        cols[i % 2].plotly_chart(fig, use_container_width=True)

    st.subheader("Impact point summary")
    st.write(
        f"Impact at downrange = {downrange[-1]:.1f} m, cross-range = {crossrange[-1]:.2f} m, "
        f"time of flight = {t[-1]:.2f} s. (Teaching-lab reproduction only - not a fire-control solution.)"
    )
    with st.expander("Validation against the paper's own reported numbers (Sec. 3.3, 50 deg)"):
        st.markdown("""
        | Quantity | Paper (exact quote) | This run |
        |---|---|---|
        | Total flight time | "79 sec" | see metric above |
        | Summit time | "nearly 36 sec" | see Altitude plot peak |
        | Muzzle velocity | 26.7 m/s | input parameter |
        | Burn-out velocity (t=1.67s) | "705 m/s" | see Velocity plot at t=1.67s |
        | Initial axial acceleration | "35.4 g" | see Axial accel plot at t~0 |

        See `docs/aerodynamic-model.md` for why late-flight attitude behavior can diverge
        from the paper's reported stability: Table 1's rotational-damping columns are
        ambiguous in the source PDF, and the fin-cant roll-drive coefficient above is not
        published and had to be calibrated.
        """)

    df_export = pd.DataFrame(data)
    csv_buf = io.StringIO()
    df_export.to_csv(csv_buf, index=False)
    st.download_button("Export trajectory as CSV", csv_buf.getvalue(), file_name="trajectory.csv")
    st.download_button("Export trajectory as JSON",
                        json.dumps({k: v.tolist() for k, v in data.items()}),
                        file_name="trajectory.json")

    if run_dispersion:
        st.subheader("Dispersion sensitivity analysis (Monte Carlo)")
        with st.expander("About this sweep (Sec. 3.4, Table 2, Figs. 10-21)"):
            st.markdown("""
            This reproduces the paper's joint dispersion sweep: every uncertain
            parameter in Table 2 (muzzle velocity, elevation angle, masses,
            burn time, thrust, air density, inertias, spin rate) is drawn as an
            independent Gaussian and propagated through a full trajectory, so
            the scatter below reflects the combined effect of all ten
            uncertainties at once - compare against the corresponding figures
            in FM04.pdf Sec. 3.4. The paper's Sec. 4 conclusion is that motor
            parameters (thrust, burn time) tend to dominate range error; try
            zeroing out other std-dev inputs in the sidebar one at a time to
            see whether that holds here too.
            """)
        std_devs = {
            "elevation_deg": elev_std, "mass_pct": mass_std_pct, "propellant_mass_pct": prop_mass_std_pct,
            "burn_time_s": burn_time_std, "thrust_pct": thrust_std_pct, "density_pct": density_std_pct,
            "ixx_pct": ixx_std_pct, "iyy_pct": iyy_std_pct, "v0_pct": v0_std_pct, "spin_pct": spin_std_pct,
        }
        with st.spinner(f"Running {int(n_samples)}-sample dispersion sweep..."):
            ranges, drifts, tofs = run_joint_dispersion(
                rocket, aero, elevation_deg=elev_deg, n_samples=int(n_samples),
                std_devs=std_devs, t_end=t_max, dt=dt, method="rk4" if method != "Euler" else "euler")

        summary = {
            "n_samples": int(n_samples),
            "range_mean_m": float(np.mean(ranges)), "range_std_m": float(np.std(ranges)),
            "drift_mean_m": float(np.mean(drifts)), "drift_std_m": float(np.std(drifts)),
            "tof_mean_s": float(np.mean(tofs)), "tof_std_s": float(np.std(tofs)),
        }
        st.json(summary)

        fig_disp = go.Figure(data=go.Scatter(x=drifts, y=ranges, mode="markers", marker=dict(size=5, opacity=0.6)))
        fig_disp.update_layout(title="Impact point dispersion (drift vs. range)",
                                xaxis_title="Cross-range / drift (m)", yaxis_title="Downrange (m)", height=500)
        st.plotly_chart(fig_disp, use_container_width=True)

        disp_df = pd.DataFrame({"range_m": ranges, "drift_m": drifts, "time_of_flight_s": tofs})
        disp_csv = io.StringIO()
        disp_df.to_csv(disp_csv, index=False)
        st.download_button("Export dispersion samples as CSV", disp_csv.getvalue(), file_name="dispersion.csv")

st.divider()
st.caption(
    "RocketDynamicsLab is an offline educational simulator reproducing a published academic "
    "6-DOF model and teaching numerical methods. It is not a fire-control system, targeting "
    "tool, or operational artillery calculator, and provides no aim solutions or weapon-"
    "deployment guidance. See docs/ and the project wiki for the full course material."
)
