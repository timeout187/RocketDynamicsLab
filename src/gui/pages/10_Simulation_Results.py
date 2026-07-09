import numpy as np
import streamlit as st
from common import page_header, professor_notes, student_exercises, default_rocket_sidebar

page_header("Simulation Results", paper_ref="Fig. 1 'Output Parameters' box")

st.markdown("Run a full trajectory and inspect the summary output parameters "
            "the paper's block diagram lists: summit point, impact point, time of flight.")

t, x = default_rocket_sidebar()
altitude = -x[:, 11]
speed = np.sqrt(x[:, 0]**2 + x[:, 1]**2 + x[:, 2]**2)

summit_idx = int(np.argmax(altitude))
c1, c2, c3 = st.columns(3)
c1.metric("Time of flight [s]", f"{t[-1]:.2f}")
c2.metric("Summit altitude [m]", f"{altitude[summit_idx]:.1f}")
c3.metric("Summit time [s]", f"{t[summit_idx]:.2f}")

c1, c2, c3 = st.columns(3)
c1.metric("Impact range (North) [m]", f"{x[-1,9]:.1f}")
c2.metric("Impact drift (East) [m]", f"{x[-1,10]:.1f}")
c3.metric("Max speed [m/s]", f"{np.max(speed):.1f}")

st.subheader("Raw state history")
import pandas as pd
df = pd.DataFrame(x, columns=["u", "v", "w", "p", "q", "r", "phi", "theta", "psi", "N", "E", "D"])
df.insert(0, "t", t)
st.dataframe(df.iloc[::max(1, len(df)//200)], use_container_width=True)

st.download_button("Download full state history (CSV)", df.to_csv(index=False),
                    file_name="trajectory.csv", mime="text/csv")

professor_notes("""
Compare these summary numbers directly against FM04.pdf Sec. 3.3's reported
values (79 s total flight time, 36 s summit time, 705 m/s burn-out
velocity, at a 50° firing angle) — remind students the default here is
45° and coefficients are reconstructed, so exact match is not expected
(Exercise 7).
""")
student_exercises("See `docs/assignments.md` Exercise 7.")
