import numpy as np
import streamlit as st
from common import page_header, professor_notes, student_exercises

page_header("Forces and Moments", paper_ref="Sec. 3.2, Table 1, Fig. 1 aero block")

st.markdown("Total force/moment = **thrust** + **aerodynamic** + **gravity** (resolved via Euler angles).")

st.subheader("Interactive: force/moment build-up at one flight condition")
c1, c2, c3, c4 = st.columns(4)
V = c1.slider("Airspeed V [m/s]", 10.0, 900.0, 300.0)
alt = c2.slider("Altitude [m]", 0.0, 8000.0, 1000.0)
alpha_deg = c3.slider("Angle of attack α [deg]", -5.0, 5.0, 0.5)
p_rate = c4.slider("Roll rate p [rad/s]", 0.0, 40.0, 10.0)

from simulator.atmosphere import Atmosphere
from simulator.aerodynamics import AeroModel
from simulator.rocket import ROCKET_122MM

atmo = Atmosphere()
aero = AeroModel()
rho = atmo.density(alt)
M = atmo.mach(alt, V)
alpha = np.radians(alpha_deg)

Tx_a, Ty_a, Tz_a, L, Mm, N = aero.forces_moments(rho, V, M, alpha, 0.0, p_rate, 0.0, 0.0,
                                                   ROCKET_122MM.reference_area, ROCKET_122MM.caliber)

cols = st.columns(3)
cols[0].metric("Mach number", f"{M:.3f}")
cols[1].metric("Air density [kg/m^3]", f"{rho:.4f}")
cols[2].metric("Dynamic pressure q̄ [Pa]", f"{0.5*rho*V*V:.1f}")

cols = st.columns(3)
cols[0].metric("Axial (drag) force [N]", f"{Tx_a:.1f}")
cols[1].metric("Normal force [N]", f"{Tz_a:.1f}")
cols[2].metric("Roll moment [N·m]", f"{L:.2f}")

cols = st.columns(3)
cols[0].metric("Pitch moment [N·m]", f"{Mm:.2f}")
thrust = ROCKET_122MM.thrust_at(0.5)
cols[1].metric("Thrust (boost phase) [N]", f"{thrust:.0f}")
cols[2].metric("Weight (nominal mass) [N]", f"{ROCKET_122MM.mass_total*9.80665:.0f}")

st.latex(r"""
T_x = -\bar q S\, C_A \qquad
T_z = -\bar q S\, C_{N\alpha}\,\alpha \qquad
M = \bar q S D\left(C_{m\alpha}\alpha + C_{mq}\frac{qD}{2V}\right)
""")

professor_notes("""
Fix V and alpha, sweep altitude 0→8000 m and watch the axial/normal force
metrics fall as density drops — a direct link to `docs/atmosphere-model.md`.
""")
student_exercises("See `docs/assignments.md` Exercise 7 and `docs/aerodynamic-model.md`.")
