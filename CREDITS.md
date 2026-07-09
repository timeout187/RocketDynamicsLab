# Credits

## Author / Maintainer

- **[timeout187](https://github.com/timeout187)** — repository design, simulator implementation, GUI, documentation, and assignments.

## Required reading / theoretical foundation

This project is built as a teaching companion to:

> M. Khalil, H. Abdalla, O. Kamal, *"Trajectory Prediction for a Typical Fin
> Stabilized Artillery Rocket"*, ASAT-13-FM-04, 13th International Conference
> on Aerospace Sciences & Aviation Technology, Military Technical College,
> Cairo, Egypt, 26–28 May 2009.

All credit for the original 6-DOF model, case-study data, and dispersion
methodology belongs to the paper's authors. This repository is an independent
educational reimplementation for classroom use and is not affiliated with the
paper's authors or institution. See `docs/aerodynamic-model.md` for a note on
where this repo's aerodynamic-coefficient table diverges from the source
(the source PDF's Table 1 text layer is corrupted and could not be recovered
exactly).

## References cited by the source paper

1. Bernard Etkin, *"Dynamics of Atmospheric Flight"*, John Wiley & Sons, 1972.
2. Douglas O., Mark C., *"Model Predictive Control of a Direct Fire Projectile
   Equipped With Canards"*, J. Dynamic Systems, Measurement, and Control,
   Vol. 130, Nov 2008.
3. Ezeddine Salem Ab. Ali, *"Parametric Study Of Missile Trajectory"*, M.Sc.
   thesis, Military Technical College, Cairo, March 2009.
4. Gagnon E., Lauzon M., *"Course Correction Fuze Concept Analysis for In
   Service 155mm Spin-Stabilized Gunnery Projectiles"*, AIAA GNC Conference,
   Honolulu, Aug 2008.
5. Jankovic S., Gallant J., Celens E., *"Dispersion of an Artillery Projectile
   due to Unbalance"*, 18th Int. Symposium on Ballistics, San Antonio, 1999.
6. Khalil M. S., *"Trajectory Prediction of Flying Vehicle"*, M.Sc. thesis,
   Military Technical College, Cairo, Egypt, 2008.
7. Lua K. B., Lim T. T., Luo S. C., *"Helical-Groove and Circular-Trip Effects
   on Side Force"*, Journal of Aircraft, Vol. 37, No. 5, 2000.
8. Saghafi F., Khalilidelshad M., *"A Monte-Carlo Dispersion Analysis of a
   Rocket Flight Simulation Software"*, 17th European Simulation
   Multi-Conference ESM2003, England, 2003.
9. Sailaranta T., Siltavuori A., Laine S., Fagerstrom B., *"On Projectile
   Stability and Firing Accuracy"*, 20th Int. Symposium on Ballistics,
   Orlando, 2002.

## Tools and libraries

- [NumPy](https://numpy.org/) and [SciPy](https://scipy.org/) — numerical computing and `solve_ivp`.
- [Streamlit](https://streamlit.io/) — the interactive lab GUI framework.
- [Plotly](https://plotly.com/python/) — interactive visualization.
- [pandas](https://pandas.pydata.org/) — tabular data display in the GUI.
- [pytest](https://pytest.org/) — the test suite.

## Generated with assistance from

Repository scaffolding, documentation, and code authored with assistance from
[Claude Code](https://claude.com/claude-code) (Anthropic).
