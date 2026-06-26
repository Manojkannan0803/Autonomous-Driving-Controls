# Project 3 — Kinematic Bicycle Model

## Problem Statement

Every controller in this portfolio (P4–P9) needs a **vehicle dynamics model** to:

1. Predict where the car will be after applying a control command.
2. Simulate the "plant" so controllers can be tested without physical hardware.

The simplest model that captures the turning geometry of a car with four wheels is the
**kinematic bicycle model** — it treats the front and rear wheel pairs as single wheels
on a rigid axle and assumes no tyre slip (low speed, dry road). Despite its simplicity,
it is accurate enough for speeds below ~20 m/s and is universally used in AV prototype
development (Waymo, Cruise, Apollo all use it for planner-level simulation).

---

## Architecture

```
bicycle_model.hpp  (header-only, namespace vehicle)
│
├── State   { x, y, theta, v }         2-D position, heading, speed
├── Control { delta, a }               steering angle, acceleration
├── BicycleParams { wheelbase=2.7, max_steer=0.524, ... }
└── BicycleModel::step(State, Control, dt) → State  (RK4 integration)

reference_path.hpp  (namespace vehicle)
├── Waypoint { x, y }
├── ReferencePath::figure_eight(size, n_pts)  — parametric figure-8
├── ReferencePath::from_waypoints(pairs)      — factory from (x,y) list
├── nearest_index(x, y)                       — closest waypoint
└── cross_track_error(State, i)               — signed lateral deviation

src/main.cpp
├── Scenario A: straight-line with P-speed-controller
├── Scenario B: constant-radius circle (ground-truth turning radius check)
├── Scenario C: figure-eight (exercises heading wrap-around)
└── Writes: trajectories.csv, turning_radius.csv, speed_profile.csv
```

---

## Design & Implementation

### Equations of Motion

The kinematic bicycle model captures turning geometry via the **front-axle steering angle δ**:

```
ẋ     = v · cos(θ)
ẏ     = v · sin(θ)
θ̇     = (v / L) · tan(δ)       L = wheelbase (2.7 m)
v̇     = a
```

The yaw rate `θ̇ = v·tan(δ)/L` gives a turning radius `R = L/tan(|δ|)`.  At `δ = 30°`
(`0.524 rad`) and `L = 2.7 m`, the minimum turning radius is `R = 4.68 m` — consistent
with a compact car.

### Why Kinematic (Not Dynamic)?

A dynamic model adds lateral tyre forces, slip angles, and load transfer, requiring tyre
parameters that differ per vehicle.  Below ~15 m/s the slip angles are small (< 3°) so
the kinematic model matches a dynamic model to within 5 % — adequate for path planning
and linear controller design.

### Integration: RK4

The four-state ODE is integrated with the same RK4 algorithm developed in P1 (inlined
directly in `bicycle_model.hpp` to avoid a cross-project dependency).  Using RK4 rather
than Euler ensures that even at `dt = 0.02 s` (50 Hz), position drift over a 30 s
figure-eight is < 1 mm vs. < 30 mm for Euler.

### Heading Wrap

`wrap_angle(a)` maps any angle to `(−π, π]` using:

```
a − 2π · floor((a + π) / (2π))
```

This is called after every RK4 step on `θ` to prevent heading from drifting to ±10π
over a long simulation — a subtle bug that causes controllers to compute the "long way
around" heading error.

### `from_waypoints` Factory

Added to support P8 and P9 pipelines: converts a `vector<pair<double,double>>` from the
A\* planner directly into a `ReferencePath` without needing a parametric formula.

---

## Test & Validation

| Test | What it checks |
|---|---|
| `straight_line_kinematics` | δ=0, a=0, v=5 → x grows by v·dt each step |
| `turning_radius_at_30deg` | Circle radius matches L/tan(δ) analytically |
| `heading_wrap` | θ wraps correctly at ±π boundary |
| `rk4_vs_euler_drift` | RK4 position error < 1 mm over 30 s figure-eight; Euler < 30 mm |
| `state_limits_respected` | v clamped to [0, max_speed]; delta clamped to ±max_steer |
| `figure_eight_closes` | After one full figure-eight, return position within 0.1 m of start |
| `reference_path_nearest` | `nearest_index` returns correct index for 5 test points |
| `cross_track_error_sign` | CTE is positive left of path, negative right |

---

## Figures & Trend Rationale

### `trajectories.png` — Three Scenarios

**Straight line**: The vehicle accelerates from rest to cruise speed then holds it.
The path is exactly straight (θ = const), confirming that `δ = 0` produces no lateral
drift even over 60 s — a sanity check on the RK4 integration.

**Circle**: With constant `δ = 20°`, the vehicle traces a circle.  The trajectory closes
perfectly back to the start after `2πR` metres, and the measured radius matches
`L/tan(δ) = 7.43 m` to < 0.1 % error.  This validates the yaw-rate equation.

**Figure-eight**: Two overlapping circles with a crossing point.  The heading passes
through ±π at the crossover; the wrap function keeps `θ` continuous.  The figure-eight
is used as the reference path for P4, P5, P6, and P8 because it exercises:
- left and right turning
- speed changes on curvature transitions
- heading angle wrap-around

### `speed_profile.csv` — Speed vs. Time

A simple P-controller on speed demonstrates the plant response: exponential rise toward
the setpoint (first-order system), confirming that `v̇ = a` in the model matches the
expected characteristic time constant.
