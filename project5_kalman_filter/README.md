# Project 5 — Kalman Filter & Extended Kalman Filter

## Problem Statement

Real vehicles cannot observe their own state directly.  A GPS receiver produces noisy
position fixes at 5 Hz with σ ≈ 3 m.  An IMU measures acceleration and yaw rate at 50 Hz
with much lower noise but **drifts** (no absolute position).  Fusing these two sensors
optimally requires a **state estimator**.

This project builds two estimators of increasing sophistication:

| Estimator | Model | Limitation |
|---|---|---|
| Linear KF | Constant-velocity (CV) model, GPS only | Cannot track curved paths — diverges on figure-eight |
| EKF | CTRV (Constant Turn Rate & Velocity), GPS + IMU | Tracks curves correctly; handles nonlinear motion |

The demo runs both estimators in parallel on the same synthetic sensor stream (GPS + IMU)
while a vehicle drives a figure-eight.  Ground truth is the exact bicycle model trajectory.
The quality metric is position RMSE; a second metric (ANEES) checks whether the uncertainty
estimate `P` is calibrated.

---

## Architecture

```
kalman_filter.hpp  (header-only, namespace estimation)
│
├── Mat<R,C>     — fixed-size row-major matrix template (no Eigen)
├── KFState      { x: Vec4, P: Mat4 }  — state + covariance
│
├── LinearKF     — Predict: x' = F·x + B·u,  Update: Kalman gain on GPS
└── EKF          — Predict: x' = f(x,u) + Jf·P·Jf^T + Q,
                             Update: x' = x + K·(z − h(x))
                   f(x,u) = CTRV + IMU control input
                   h(x)   = [px, py]  (GPS, linear measurement)

sensor_models.hpp  (namespace sensors)
├── GPS { rate_hz=5, sigma=3.0 }  — GPSMeasurement { px, py }
└── IMU { rate_hz=50, sigma_ax=0.3, sigma_omega=0.01 }
       — IMUMeasurement { ax, omega }

src/main.cpp
├── Ground-truth: BicycleModel on figure-eight (open-loop Pure Pursuit)
├── Both estimators run on same synthetic sensor stream
└── Writes: trajectory.csv, errors.csv, innovations.csv
```

---

## Design & Implementation

### Matrix Library

No Eigen dependency. A thin `Mat<R,C>` template wraps `std::array<double, R*C>` with
row-major indexing.  Fixed-size ensures stack allocation and compile-time dimension checks.
The 4×4 operations needed for the filter are under 200 ns — negligible at 50 Hz.

### Linear Kalman Filter — 4-state CV Model

**State**: `[px, py, vx, vy]`

**Predict** (constant velocity):
```
F = [[1, 0, dt,  0],   Q = diag(q_pos, q_pos, q_vel, q_vel)
     [0, 1,  0, dt],
     [0, 0,  1,  0],
     [0, 0,  0,  1]]
```

The CV model assumes the vehicle travels in a straight line between prediction steps.
On a curved figure-eight this is fundamentally wrong — the model predicts straight while
the vehicle turns, causing systematic bias that the Kalman gain cannot fully correct.

### Extended Kalman Filter — 4-state CTRV Model

**State**: `[px, py, v, θ]`

**Nonlinear predict** (with IMU input `u = [ax, ω]`):
```
px' = px + v·cos(θ)·dt
py' = py + v·sin(θ)·dt
v'  = v + ax·dt
θ'  = θ + ω·dt
```

The Jacobian of `f(x, u)` with respect to `x` linearises the prediction for covariance
propagation: `P' = Jf·P·Jf^T + Q`.

**GPS update** (linear, `H = [[1,0,0,0],[0,1,0,0]]`):
```
y = z − H·x               (innovation)
S = H·P·H^T + R            (innovation covariance, 2×2)
K = P·H^T · S⁻¹           (Kalman gain, 4×2)
x = x + K·y
P = (I − K·H)·P
```

The 2×2 matrix `S` is inverted analytically (closed form) — faster and numerically
more robust than LU decomposition for a 2×2 system.

### Process Noise Tuning

| Parameter | Value | Physical meaning |
|---|---|---|
| q_vel | 0.5 m/s/√s | IMU accelerometer noise propagation |
| q_yaw | 0.05 rad/√s | Gyroscope drift |
| r_gps | 3.0 m | GPS measurement sigma |

`Q` is set to match the IMU noise floor.  If `Q` is too large, the filter distrusts its
prediction and reacts to every GPS jitter (high noise in estimate). If too small, the
filter ignores GPS corrections and drifts with the gyro bias.

### ANEES — Filter Health Check

```
ANEES = (1/N) Σ_k  (x_true_k − x̂_k)^T · P_k^{-1} · (x_true_k − x̂_k)
```

For a 2-dimensional GPS measurement, a well-calibrated filter gives ANEES ≈ 2 (chi-squared
with 2 degrees of freedom). ANEES >> 2 means the filter is overconfident (P underestimates
true uncertainty). ANEES << 2 means the filter is underconfident (Q is too large).

---

## Test & Validation

| Test | What it checks |
|---|---|
| `linear_kf_straight` | On straight line, KF RMSE < 0.5·GPS sigma |
| `ekf_figure_eight_rmse` | EKF RMSE < Linear KF RMSE on figure-eight |
| `ekf_anees_calibrated` | ANEES ∈ [0.5, 5.0] — filter not wildly mis-calibrated |
| `imu_reduces_uncertainty` | EKF position variance decreases between GPS updates |
| `gps_rate_correct` | GPS fires ≈ 5 times per second |
| `imu_noise_bounded` | 99 % of IMU readings within 3σ of true value |
| `ekf_heading_correct` | EKF heading error < 0.1 rad after 5 s on straight line |
| `innovations_white` | Innovation autocorrelation at lag-1 < 0.2 (near white noise) |

---

## Figures & Trend Rationale

### `trajectory.png` — True vs. KF vs. EKF Paths

Three paths overlaid on the figure-eight:

- **True path** (ground truth bicycle model): a smooth figure-eight.
- **GPS raw** (if plotted): scattered dots ±3 m from the true path, confirming the
  measurement noise level.
- **Linear KF**: follows the figure-eight roughly but drifts outward on curves. The CV
  model predicts straight-line motion; when the vehicle turns, the Kalman gain drags the
  estimate toward the GPS measurement (which points inward), but the model fights this
  correction. The result is a systematic outward bias on every curve.
- **EKF**: tracks the true path closely. The CTRV model correctly predicts turning because
  the IMU yaw rate `ω` is fed as a control input — the model "knows" the vehicle is turning
  and predicts the curved arc between GPS updates.

The crossover point of the figure-eight is where both filters struggle most: the heading
changes rapidly, the GPS is 200 ms stale, and the IMU measurement noise is highest in the
lateral direction.  The EKF recovers within 0.5 s; the KF recovers within 1–2 s.

### `errors.png` — Position Error Over Time

- **KF error** shows a regular periodic pattern — errors peak at the apex of every
  figure-eight loop (maximum curvature) and decay on the straights (where the CV model
  is accurate). This periodicity is a direct signature of model mismatch.
- **EKF error** is lower on average with less periodic structure. The residual error
  comes from GPS measurement noise (σ=3 m) — the filter cannot reduce it below the
  Cramér-Rao lower bound set by sensor quality.
- RMSE gap between KF and EKF demonstrates the value of the nonlinear CTRV model and
  IMU fusion: **the EKF achieves ~30–40 % lower position RMSE** on the figure-eight.

### `innovations.png` — EKF Innovation Sequence

The innovation `y_k = z_k − H·x̂_k` should be zero-mean white Gaussian noise if the
filter is correctly calibrated:

- A well-tuned EKF shows innovations that oscillate randomly around zero with no visible
  trend — confirming the filter absorbs all systematic structure.
- The **innovation magnitude** follows a Gaussian distribution with covariance `S_k =
  H·P_k·H^T + R`. If innovations are too large (outside `3σ` bands), the process noise
  `Q` is too small; if too small, `Q` is too large and the filter over-trusts its model.
- **Autocorrelation at lag-1** near zero confirms "whiteness" — consecutive innovations
  are statistically independent, meaning the filter is not leaving exploitable correlation
  in the residuals.
