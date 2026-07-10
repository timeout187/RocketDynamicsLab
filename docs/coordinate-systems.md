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

## Navigation equation: flat vs. rotating/curved Earth

`state_derivative`'s `include_earth_rotation` flag controls *two*
independent pieces of physics, both disabled by default:

- **False (default)** — flat, non-rotating Earth. Body rates `[p q r]` are
  used as-is, and position is simply `[Ṅ Ė Ḋ] = L_BE @ [u v w]`. This
  matches the paper's short-range 122mm case study exactly and is left
  byte-for-byte unchanged by the rotating-Earth work below.
- **True** — full rotating/curved-Earth model:
  - Eq. (3) Coriolis term on body rates (`earth_rotation_body_rates`,
    unchanged from before).
  - A rotating/curved-Earth **Navigation Equation** for the geodetic-frame
    velocity `[V_N V_E V_D]` (now carried as three *extra* integrated
    states, growing the state vector from 12 to 15):
    ```
    V_N_dot = (mu_dot + 2*omega)*sin(lambda)*V_E - lambda_dot*V_D + a_N
    V_E_dot = -(mu_dot + 2*omega)*sin(lambda)*V_N - (mu_dot+2*omega)*cos(lambda)*V_D + a_E
    V_D_dot = (mu_dot + 2*omega)*cos(lambda)*V_E + lambda_dot*V_N + a_D
    lambda_dot = V_N / (R_meridian + H_G)
    mu_dot     = V_E / ((R_normal + H_G) * cos(lambda))
    ```
    where `lambda`/`mu` are geodetic latitude/longitude, `H_G` is geodetic
    altitude, and `a_N, a_E, a_D` are the body-axis velocity rotated into
    the geodetic frame via `L_BE` (i.e. what the flat-Earth path calls
    `vel_ned`).

  **Simplification, documented explicitly**: the paper doesn't publish a
  closed-form rotating/curved-Earth Navigation Equation, only the flat-Earth
  form and the Eq. (3) body-rate term. This lab therefore uses the standard
  spherical-Earth approximation `R_meridian = R_normal = R_EARTH` (mean
  Earth radius, 6,378,137 m — WGS84 semi-major axis), recovering the current
  latitude from the North distance travelled as `lambda = lambda_0 + N /
  R_EARTH`. A full WGS84 ellipsoidal model (latitude-dependent
  `R_meridian`/`R_normal`) is *not* implemented — left as a possible
  extension exercise. See `equations_of_motion.py` for the code.

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
