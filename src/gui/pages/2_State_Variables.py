import streamlit as st
from common import page_header, professor_notes, student_exercises, variable_card

page_header("State Variables", paper_ref="Fig. 1 signal-flow diagram")

st.markdown("""
The 12-state vector `x = [u,v,w, p,q,r, φ,θ,ψ, N,E,D]` fully describes the
rocket's instantaneous condition. Every simulation is: start from an
initial `x(0)` (muzzle conditions), integrate `dx/dt = f(t,x)` forward.
""")

variable_card("u, v, w", "Body-axis velocity components (axial, side, normal)",
              "Integrated from Eq. (1) translational dynamics",
              "m/s", "Determine airspeed V, angle of attack α, sideslip β")
variable_card("p, q, r", "Body-axis angular rates (roll, pitch, yaw rate)",
              "Integrated from Euler's rotational equations",
              "rad/s", "Drive attitude change; feed aerodynamic damping moments")
variable_card("φ, θ, ψ", "Euler angles: roll (bank), pitch (inclination), yaw (azimuth)",
              "Integrated from the kinematic equation",
              "rad (deg in plots)", "Determine gravity components in body axes and L_BE")
variable_card("N, E, D", "Geodetic position: North, East, Down",
              "Integrated from the navigation equation (L_BE · [u v w])",
              "m", "Directly gives range, drift, altitude (=-D)")

st.subheader("Derived (not integrated) quantities")
variable_card("α (alpha)", "Angle of attack: atan2(w_r, u_r)",
              "Computed each step from relative velocity",
              "rad (deg)", "Drives normal force and pitching moment")
variable_card("β (beta)", "Sideslip angle: asin(v_r/V)",
              "Computed each step from relative velocity",
              "rad (deg)", "Drives side force and yawing moment")
variable_card("M (Mach)", "V / local sonic speed a(h)",
              "Atmosphere model + current altitude/speed",
              "dimensionless", "Selects aerodynamic coefficients from Table 1 lookup")

professor_notes("""
Ask students to identify, for each state variable, which *equation group*
(translational, rotational, kinematic, navigation) produces its
derivative — this reinforces the Fig. 1 block-diagram structure before
diving into the equations themselves.
""")
student_exercises("Cross-reference with `docs/equations.md` before Exercise 1.")
