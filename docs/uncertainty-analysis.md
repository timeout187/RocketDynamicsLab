# Sensitivity Analysis and Uncertainty Propagation

*Reference: FM04.pdf Sec. 3.4 "Dispersion Analysis", Table 2, Figs. 10-21.
Code: `src/simulator/dispersion.py`.*

## Why dispersion matters (the paper's framing)

An *unguided* rocket has no feedback loop correcting its flight path — its
impact point is entirely determined by initial conditions and physical
parameters that are never known perfectly. The paper's introduction
identifies three phases contributing to total dispersion:

1. **Production inaccuracy** — propellant mass/composition, total mass,
   moments of inertia, resultant center of gravity.
2. **Boosting-phase disturbances** — launcher deflection, tip-off, thrust/fin
   misalignment, atmospheric gusts during the few seconds of powered flight.
3. **Free-flight disturbances** — wind-profile fluctuations, atmospheric
   density variation, and static rocket-property uncertainty acting over the
   whole remaining flight.

## The method: one-parameter-at-a-time sensitivity sweeps

Table 2 lists **twelve** uncertain parameters, each given a plausible
uncertainty range (e.g. launch pitch angle ±2°, rocket mass ±2%, air density
±4%). For each parameter, the paper:

1. Holds every other parameter at its nominal value.
2. Sweeps the one parameter of interest across its stated range.
3. Re-runs the full 6-DOF trajectory simulation for each sample.
4. Records the resulting **range error**, **drift error**, and **radial
   error** relative to the nominal impact point.
5. Plots error vs. parameter value (Figs. 10-21) to see which parameters the
   impact point is most sensitive to.

This is **local, one-at-a-time (OAT) sensitivity analysis** — simple,
interpretable, and cheap (one 1-D sweep per parameter, no combinatorial
explosion), but it does *not* capture interaction effects between
parameters (e.g. does a mass error matter more or less when air density is
also off?). Global/variance-based methods (Sobol indices, full factorial or
Latin-hypercube Monte Carlo) capture those interactions at higher
computational cost — a natural extension exercise.

## This lab's implementation

`dispersion.monte_carlo_dispersion(param, n_samples, ...)` reproduces this
workflow for any of the ten parameters in `DEFAULT_UNCERTAINTIES`
(a `Table 2`-derived list), returning value/range-error/drift-error/radial-
error arrays exactly analogous to Figs. 10-21. The **Sensitivity Analysis**
GUI page lets you pick a parameter, choose a sample count, and see the
scatter plot live, then compare it side-by-side across parameters to
reproduce the paper's qualitative conclusion:

> "the rocket motor parameters (burning time, propellant mass and mean
> thrust value) have a great effect on the rocket range and its impact
> point error." (Sec. 4, Conclusion)

## What "uncertainty range" means here

Table 2's ranges (e.g. `[-2%, +2%]` for total mass) represent a
**manufacturing/measurement tolerance band**, not a probability
distribution per se. This lab treats the sweep as either a uniform grid
across the range (default, matching the paper's plotting style) or, for
students who want to go further, a Monte-Carlo draw from a normal
distribution scaled to the range (`distribution="normal"`) — an explicit
invitation to discuss *which* distributional assumption is appropriate for
a manufacturing tolerance (uniform? normal? triangular?) and how that
choice changes the resulting *impact probability ellipse*, which real
artillery dispersion analysis ultimately cares about (Circular Error
Probable, CEP).

## Professor Notes

> A strong assignment extension: instead of one-at-a-time sweeps, draw all
> ten parameters simultaneously from their distributions (a full Monte
> Carlo run of, say, 500 trajectories) and plot the resulting impact-point
> scatter as a 2-D (range, drift) cloud — this is the actual dispersion
> ellipse a fire-control or accuracy-requirements engineer would compute,
> and it is *not* simply the sum of the individual OAT sensitivities unless
> the underlying model is linear (Exercise 8).

## Student Exercises

See `assignments.md` Exercise 8.
