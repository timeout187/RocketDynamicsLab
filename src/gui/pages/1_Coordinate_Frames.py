import numpy as np
import streamlit as st
from common import page_header, professor_notes, student_exercises

page_header("Coordinate Frames", paper_ref="Eq. (3)-(4), Fig. 1")

st.markdown("""
Two reference frames matter for this model:

- **Body-fixed frame (F_B)** — origin at the C.G., x forward along the
  symmetry axis, y right, z down. Moments of inertia are constant here.
- **Local geodetic / NED frame (F_E)** — origin at launch, x North, y East,
  z Down. Trajectory position and "range/drift" are naturally expressed here.

Attitude is described by three Euler angles applied in the 3-2-1 (yaw,
pitch, roll) sequence: **ψ** (yaw/azimuth), **θ** (pitch/inclination),
**φ** (roll/bank).
""")

st.subheader("Interactive: direction cosine matrix L_BE")
c1, c2, c3 = st.columns(3)
phi = np.radians(c1.slider("φ roll [deg]", -180, 180, 0))
theta = np.radians(c2.slider("θ pitch [deg]", -90, 90, 45))
psi = np.radians(c3.slider("ψ yaw [deg]", -180, 180, 0))

from simulator.frames import euler_to_LBE
L_BE = euler_to_LBE(phi, theta, psi)
st.write("L_BE (body → geodetic):")
st.dataframe(np.round(L_BE, 4))

v_body = np.array([1.0, 0.0, 0.0])
v_geo = L_BE @ v_body
st.markdown(f"A unit vector pointing along the body's nose `[1,0,0]` maps to "
            f"geodetic `[N,E,D] = [{v_geo[0]:.3f}, {v_geo[1]:.3f}, {v_geo[2]:.3f}]`.")

st.subheader("Kinematic equation (Euler angle rates)")
st.latex(r"""
\begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix} =
\begin{bmatrix}1 & \sin\phi\tan\theta & \cos\phi\tan\theta\\
0 & \cos\phi & -\sin\phi\\
0 & \sin\phi\sec\theta & \cos\phi\sec\theta\end{bmatrix}
\begin{bmatrix}P\\Q\\R\end{bmatrix}
""")
st.warning("Singular at θ = ±90° (gimbal lock) — sec θ → ∞. See Exercise 5.")

professor_notes("""
Have students verify L_BE by hand-multiplying R_z(ψ)·R_y(θ)·R_x(φ) (or the
appropriate order for your sign convention) and matching every element
against `frames.euler_to_LBE()`.
""")
student_exercises("See `docs/assignments.md` Exercises 1(c) and 5.")
