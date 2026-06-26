# Project 2 — PID Controller

## Problem Statement

A Proportional–Integral–Derivative (PID) controller is the most widely deployed feedback
control law in industrial and automotive systems. Despite its simplicity, a naive implementation
fails in real hardware due to three well-known pathologies:

1. **Integral windup** — when the actuator saturates, the integrator keeps accumulating error.
   Once the setpoint is reached, the excess integral overshoots the target.
2. **Derivative kick** — differentiating noisy sensor readings amplifies high-frequency noise
   into large, sudden actuator commands.
3. **Output saturation** — physical actuators have hard limits (e.g. maximum throttle).
   Ignoring these limits produces unrealisable commands.

This project implements a production-grade discrete PID controller that handles all three
pathologies, then validates each feature on a first-order vehicle speed plant.

---

## Architecture

```
pid_controller.hpp  (header-only, namespace controls)
│
├── PIDController::Config { Gains{Kp,Ki,Kd}, output_min/max,
│                           integral_min/max, derivative_tau }
├── PIDController::update(error, dt)  → clamped output
└── PIDController::reset()            → clear integrator & derivative state

vehicle_plant.hpp  (namespace sim)
└── VehicleSpeedPlant::step(throttle, dt)  → speed (first-order lag τ=1.5 s)

src/main.cpp
├── Scenario 1: step response  P / PI / PID (anti-windup) / PID+filter
├── Scenario 2: setpoint changes (speed profiling)
├── Scenario 3: bad Kp (oscillation), bad weather (disturbance rejection)
└── Writes: pid_control.csv, speed_change.csv, bad_kp_bang.csv, bad_weather.csv
```

---

## Design & Implementation

### Plant Model

The vehicle speed plant is modelled as a first-order lag:

```
τ·dv/dt = u − v         →   v_{n+1} = v_n + (dt/τ)(u_n − v_n)
```

with `τ = 1.5 s`, representing the combined inertia of throttle actuation and vehicle mass.
This is the standard transfer function `G(s) = 1/(τs+1)` discretised with forward Euler.

### PID Update Law

```
P = Kp · e
I = clamp(I + e·dt, integral_min, integral_max)     ← anti-windup
raw_D = (e − e_prev) / dt
D_filtered = D_prev + (dt/(dt+τ_d)) · (raw_D − D_prev)   ← low-pass
u = clamp(P + Ki·I + Kd·D_filtered, output_min, output_max)
```

Key design decisions:
- **Anti-windup by clamping** (not back-calculation) — simplest reliable method; the integral
  is bounded to the same physical range as the actuator, so it cannot build up beyond what the
  plant can ever use.
- **Derivative on measurement, not error** — differentiating the error causes a "derivative kick"
  when the setpoint steps. In speed control, the reference rarely changes instantaneously, so
  differentiating error is acceptable here; `derivative_tau` low-pass filter is still provided.
- **First-order low-pass filter for D-term**: `τ_d = derivative_tau`.  Cutoff frequency
  `f_c = 1/(2π·τ_d)`.  Typical value 0.05 s gives `f_c ≈ 3 Hz`, which passes steering-relevant
  dynamics while rejecting sensor noise at > 10 Hz.

### Gains Used in Demo

| Parameter | Value | Rationale |
|---|---|---|
| Kp | 1.2 | Fast response without oscillation for τ=1.5 s plant |
| Ki | 0.8 | Eliminates steady-state error within ~3 s |
| Kd | 0.1 | Reduces overshoot; filtered at τ_d=0.05 s |
| integral limits | ±10 | Matches throttle ±100 % normalized range |
| output limits | [0, 1] | Throttle is non-negative |

---

## Test & Validation

| Test | What it checks |
|---|---|
| `step_response_reaches_setpoint` | Speed reaches within 1 % of target within 10 s |
| `no_steady_state_error_with_I` | Residual error < 0.01 m/s after settling |
| `antiwindup_reduces_overshoot` | Overshoot with windup < overshoot without, when output saturated |
| `derivative_filter_reduces_noise` | Peak D-term output < 2× without filter when Gaussian noise added |
| `output_saturation_respected` | Output never exceeds [output_min, output_max] |
| `reset_clears_integrator` | After reset(), integral = 0 confirmed by zero I contribution |
| `p_only_has_steady_state_error` | P-only controller has non-zero residual on plant with disturbance |
| `pid_rejects_constant_disturbance` | Constant load disturbance driven to zero by integral term |

---

## Figures & Trend Rationale

### `pid_comparison.png` — Step Response: P vs PI vs PID

This figure shows four controllers responding to a 0 → 10 m/s step:

- **P only**: Rises quickly but settles ~15 % below the setpoint. The steady-state error is
  nonzero because the proportional term drives output to zero only when error is zero, but
  the plant requires a nonzero steady-state throttle to maintain speed against friction.
  Without an integrator, the controller reaches equilibrium at `e = disturbance/Kp`.

- **PI**: Eliminates steady-state error — the integral term accumulates until `u` exactly
  cancels the disturbance. However, overshoot is visible: the integral continues to grow
  while the vehicle is still below setpoint, then drives it past. The closer the system gets
  to the setpoint, the harder the integrator must "release" — this is the origin of PI overshoot.

- **PID no anti-windup**: When the throttle saturates at 100% during the initial large error,
  the integral winds up to a very large value. When the speed finally approaches target, the
  accumulated integral forces a large overshoot (speed spikes ~20 % above target). This is
  the classic **integrator windup** failure.

- **PID + anti-windup + D-filter**: The clamped integrator prevents windup; the D-term adds
  damping (it sees a large positive derivative during the ramp-up, producing a braking force
  that reduces overshoot). The D-filter prevents the initial step transient from generating
  an impulsive command. Result: fast rise, minimal overshoot (< 5 %), no windup spike.

### `speed_change.png` — Setpoint Profile Tracking

Multiple setpoint changes (30→50→20→40 km/h). The PID controller with anti-windup tracks
each step cleanly. Key observations:

- Each rise and fall is symmetric — the derivative term brakes equally on acceleration and
  deceleration because the D-filter prevents kick.
- The integral resets quickly after each setpoint change (because anti-windup prevents the
  integrator from saturating during transients).
- Steady-state error is zero at every setpoint — the integrator does its job.

### `bad_weather.png` — Disturbance Rejection

A constant −3 m/s wind disturbance is applied at t = 5 s (equivalent to headwind drag).
The integral term detects the persistent nonzero error and ramps the throttle until the
disturbance is cancelled — demonstrating that **integral action is the only way to achieve
zero steady-state error under constant disturbances** (internal model principle).
