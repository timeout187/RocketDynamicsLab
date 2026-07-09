# Coordinate Systems and Euler Angles

*Reference: FM04.pdf, Fig. 1 ("Kinematics Equation", "Navigation Equation"), Eq. (3)-(4).*
*Code: `src/simulator/frames.py`.*

## Frames used in this lab

| Frame | Symbol (paper) | Origin | Axes | Used for |
|---|---|---|---|---|
| Body-fixed | F_B | rocket C.G. | x: forward along symmetry axis; y: right; z: down (completing right-hand triad) | aerodynamic forces/moments, angular rates p,q,r |
| Local geodetic (NED) | F_E | launch point | x: North; y: East; z: Down | position, velocity in "range/altitude/drift" terms students actually plot |
| Earth-centered | — | Earth center | ellipsoidal, rotating | only used if `include_earth_rotation=True` (assumption d) |

Body axes are chosen because the rocket's moments of inertia (`Ixx, Iyy,
Izz`) are **constant** in that frame — a rigid body's mass distribution
doesn't change relative to itself as it tumbles, but it would appear
time-varying if you tried to write Euler's equations in an inertial frame.
This is the standard justification in Etkin's *Dynamics of Atmospheric
Flight* (paper reference [1]), the primary textbook source underlying this
paper's equations.

## Euler angles (3-2-1 sequence)

The paper uses the standard yaw→pitch→roll (`ψ→θ→φ`) 3-2-1 Euler sequence to
describe body attitude relative to the geodetic frame:

- **φ (phi)** — roll / bank angle, rotation about the final body x-axis.
- **θ (theta)** — pitch / inclination angle, rotation about the intermediate y-axis.
- **ψ (psi)** — yaw / azimuth angle, rotation about the geodetic z-axis (first rotation).

## The direction cosine matrix L_BE (Eq. 4)

`L_BE` transforms a vector expressed in body axes into the geodetic frame:
`v_E = L_BE @ v_B`. Its transpose, `L_BE.T = L_EB`, does the reverse. This
single 3×3 matrix is used in *two* places in Fig. 1:

1. **Navigation equation** — rotating body velocity `[u v w]` into
   `[Ṅ Ė Ḋ]` to update position.
2. **Earth-rotation term (Eq. 3)** — rotating the Earth's angular velocity
   vector into body axes so it can be added to the relative body rates.

```python
from src.simulator.frames import euler_to_LBE
L_BE = euler_to_LBE(phi, theta, psi)   # radians in, 3x3 matrix out
```

## The kinematic equation (Euler angle rates)

Body angular rates `[P Q R]` are *not* the same thing as the rates of change
of the Euler angles `[φ̇ θ̇ ψ̇]` — they are related by a matrix that itself
depends on the current attitude:

```
⎡φ̇⎤   ⎡1  sinφ·tanθ   cosφ·tanθ⎤ ⎡P⎤
⎢θ̇⎥ = ⎢0  cosφ        -sinφ    ⎥ ⎢Q⎥
⎣ψ̇⎦   ⎣0  sinφ·secθ   cosφ·secθ⎦ ⎣R⎦
```

**Gimbal lock**: this matrix is singular at `θ = ±90°` (`secθ → ∞`). A
rocket pitching to vertical would break this representation — the
practical fix (beyond this teaching lab's scope) is a quaternion attitude
representation, which has no singularities but is less intuitive to read.
See Exercise 5 in `assignments.md`.

## Aerodynamic angles: angle of attack and sideslip

Given the (relative-to-wind) body velocity components `u_r, v_r, w_r`:

```
α (alpha)  = atan2(w_r, u_r)             angle of attack
β (beta)   = asin(v_r / V)               sideslip angle
V          = sqrt(u_r² + v_r² + w_r²)    airspeed magnitude
```

These angles are what the aerodynamic coefficients respond to (see
`aerodynamic-model.md`) — physically, `α` and `β` measure how far the
oncoming relative wind is misaligned from the nose of the rocket in the
pitch and yaw planes respectively.

## Professor Notes

> Ask students to derive the `L_BE` matrix from three elementary rotation
> matrices (`R_z(ψ)`, `R_y(θ)`, `R_x(φ)`) multiplied in the correct order —
> this is a classic "do it once by hand" exercise that makes the Fig. 1
> block diagram click. See Exercise 1.

## Student Exercises

See `assignments.md` Exercises 1 and 5.
