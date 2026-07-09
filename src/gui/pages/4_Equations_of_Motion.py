import streamlit as st
from common import page_header, professor_notes, student_exercises

page_header("Equations of Motion", paper_ref="Eq. (1)-(2), Euler's Equation box, Fig. 1")

st.subheader("Eq. (1): Translational dynamics")
st.latex(r"""
\dot u = \frac{T_x}{m} - g\sin\theta - Qw + Rv \\
\dot v = \frac{T_y}{m} + g\cos\theta\sin\phi - Ru + Pw \\
\dot w = \frac{T_z}{m} + g\cos\theta\cos\phi - Pv + Qu
""")
st.markdown("""
**Engineering meaning:** axial velocity `u` responds to thrust minus drag
minus a gravity component; lateral/normal velocities `v,w` respond to
side/normal aerodynamic force plus gravity — together producing angle of
attack and sideslip. The `-Qw+Rv` etc. terms are *not* extra forces — they
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
coupled* through spin `p` — a hallmark of spin-stabilized/fin-stabilized
projectile dynamics ("coning"/epicyclic motion).
""")

st.subheader("Kinematics and Navigation equations")
st.latex(r"[\dot\phi\ \dot\theta\ \dot\psi]^T = \mathbf{K}(\phi,\theta)\,[P\ Q\ R]^T "
         r"\qquad [\dot N\ \dot E\ \dot D]^T = L_{BE}\,[u\ v\ w]^T")

st.info("Try the **Simulation Results** page to see these equations integrated "
        "into a full trajectory, and the **Coordinate Frames** page for L_BE.")

with st.expander("📘 Assumptions behind these equations"):
    st.markdown("""
    - Rigid body (no structural flexing).
    - Axisymmetric mass distribution: `Iyy = Izz`, cross products of inertia zero.
    - Body-fixed frame for aerodynamic coefficients and inertia (constant Ixx,Iyy,Izz
      in this frame, at each instant — though they vary slowly with time
      during boost as propellant burns).
    - Flat, non-rotating Earth by default (`include_earth_rotation=False`);
      full ellipsoidal/rotating Earth is an optional fidelity toggle.
    """)

professor_notes("""
Walk through `src/simulator/equations_of_motion.py` live, matching each
Python line to the LaTeX above — students consistently report this
"code-next-to-math" walkthrough is what makes the block diagram click.
""")
student_exercises("See `docs/assignments.md` Exercises 1, 3, 4, 6.")
