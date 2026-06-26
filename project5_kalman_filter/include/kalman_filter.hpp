// =============================================================================
// kalman_filter.hpp  —  Linear Kalman Filter + Extended Kalman Filter
// =============================================================================
// Two estimators that fuse GPS (5 Hz, σ≈3m) with an IMU (50 Hz):
//
//   LinearKF  — constant-velocity model, 4-state [px, py, vx, vy]
//               Works perfectly on straight roads; diverges on curves
//               because the process model has no notion of "turning".
//
//   EKF       — CTRV model (Constant Turn Rate & Velocity), 4-state [px, py, v, θ]
//               Accepts IMU yaw-rate + acceleration directly as control inputs.
//               Uses the Jacobian of f(x,u) to propagate uncertainty correctly.
//
// Implementation notes:
//   • No Eigen. Fixed-size matrices via a thin Mat<R,C> template wrapping
//     std::array<double, R*C>. Row-major indexing: M(r,c) = data[r*C+c].
//   • GPS update (2-state measurement) uses an analytical 2×2 matrix inverse
//     instead of LU decomposition — closed form is exact and fast.
//   • All angle differences are wrapped to (−π, π] before use.
// =============================================================================
#pragma once

#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string_view>

namespace estimation {

// =============================================================================
// Minimal fixed-size matrix — wraps std::array, row-major storage
// =============================================================================
// M(r, c) accesses row r, column c (both 0-based).
// Template arithmetic functions below handle multiplication, addition, etc.
// This replaces Eigen for small fixed-dimension control math.
// =============================================================================
template<int R, int C>
struct Mat {
    std::array<double, R * C> data{};

    double& operator()(int r, int c)       { return data[r * C + c]; }
    double  operator()(int r, int c) const { return data[r * C + c]; }

    static Mat<R, C> zeros() { Mat<R, C> m; m.data.fill(0.0); return m; }
    static Mat<R, C> identity() requires (R == C) {
        Mat<R, C> m;
        m.data.fill(0.0);
        for (int i = 0; i < R; ++i) m(i, i) = 1.0;
        return m;
    }
};

// Column vector alias (Nx1 matrix)
template<int N> using Vec = Mat<N, 1>;

// Common types used throughout the filters
using Vec2  = Vec<2>;
using Vec4  = Vec<4>;
using Mat2  = Mat<2, 2>;
using Mat4  = Mat<4, 4>;
using Mat24 = Mat<2, 4>;   // measurement Jacobian H:  2 meas × 4 states
using Mat42 = Mat<4, 2>;   // Kalman gain K:            4 states × 2 meas

// ── Matrix arithmetic ─────────────────────────────────────────────────────────

template<int R, int C>
Mat<R, C> operator+(const Mat<R, C>& A, const Mat<R, C>& B) {
    Mat<R, C> out;
    for (int i = 0; i < R * C; ++i) out.data[i] = A.data[i] + B.data[i];
    return out;
}

template<int R, int C>
Mat<R, C> operator-(const Mat<R, C>& A, const Mat<R, C>& B) {
    Mat<R, C> out;
    for (int i = 0; i < R * C; ++i) out.data[i] = A.data[i] - B.data[i];
    return out;
}

template<int R, int K, int C>
Mat<R, C> operator*(const Mat<R, K>& A, const Mat<K, C>& B) {
    auto out = Mat<R, C>::zeros();
    for (int r = 0; r < R; ++r)
        for (int c = 0; c < C; ++c)
            for (int k = 0; k < K; ++k)
                out(r, c) += A(r, k) * B(k, c);
    return out;
}

template<int R, int C>
Mat<C, R> transpose(const Mat<R, C>& A) {
    Mat<C, R> out;
    for (int r = 0; r < R; ++r)
        for (int c = 0; c < C; ++c)
            out(c, r) = A(r, c);
    return out;
}

/// Analytical inverse of a 2×2 matrix.
/// det = ad − bc; throws if the matrix is singular (det ≈ 0).
inline Mat2 inverse2(const Mat2& M) {
    const double det = M(0,0)*M(1,1) - M(0,1)*M(1,0);
    if (std::abs(det) < 1.0e-14)
        throw std::runtime_error("inverse2: singular matrix");
    const double inv_det = 1.0 / det;
    Mat2 out;
    out(0,0) =  M(1,1) * inv_det;
    out(0,1) = -M(0,1) * inv_det;
    out(1,0) = -M(1,0) * inv_det;
    out(1,1) =  M(0,0) * inv_det;
    return out;
}

/// Wrap angle to (−π, π]
inline double wrap(double a) noexcept {
    while (a >  std::numbers::pi) a -= 2.0 * std::numbers::pi;
    while (a <= -std::numbers::pi) a += 2.0 * std::numbers::pi;
    return a;
}

// =============================================================================
// KFState — mean vector + covariance matrix (shared by both filters)
// =============================================================================
struct KFState {
    Vec4 x{};    // state mean
    Mat4 P{};    // state covariance (how uncertain we are)
};

// =============================================================================
// 1. Linear Kalman Filter — constant-velocity 2D model
// =============================================================================
// State: x = [px, py, vx, vy]^T
//
// Process model (linear, exact):
//   F = [[1, 0, dt, 0 ],
//        [0, 1, 0,  dt],
//        [0, 0, 1,  0 ],
//        [0, 0, 0,  1 ]]
//
// GPS measurement model (linear, exact):
//   H = [[1, 0, 0, 0],
//        [0, 1, 0, 0]]   →   z = H·x  (picks out px, py)
//
// Why does this fail on curves?
//   The model assumes constant velocity in world-frame x and y independently.
//   A car turning a corner has vx = v·cos(θ) and vy = v·sin(θ) — they are
//   coupled through θ, which the constant-velocity model has no knowledge of.
//   When the car turns, the model predicts it continues straight, so P grows
//   rapidly in the lateral direction. The EKF fixes this by explicitly
//   including θ and v in the state.
//
// Challenge #1 answer:
//   Q = 0 means: "My model is perfect. No unmodelled forces exist."
//   But the filter's covariance update is: P^- = F·P·F^T + Q
//   If Q = 0, P^- can only stay flat or shrink over time (GPS pulls it down).
//   After many GPS updates with a consistent but WRONG motion model, P becomes
//   tiny — the filter is "very confident" about a wrong answer.
//   It will then ignore future GPS measurements (K → 0) because it thinks it
//   knows better than the sensor. This is called "filter divergence."
//   A well-tuned Q keeps P large enough that GPS corrections are always applied.
// =============================================================================
class LinearKF {
public:
    /// @param dt        Nominal predict timestep (s)
    /// @param q_accel   Process noise: std-dev of acceleration disturbance (m/s²)
    ///                  Larger → filter trusts GPS more, tracks rapid manoeuvres better
    /// @param r_gps     GPS measurement noise std-dev (m); ≈3m for urban GPS
    explicit LinearKF(double dt, double q_accel, double r_gps) {
        // ── Process noise Q ────────────────────────────────────────────────
        // Model: vehicle may have random accelerations up to q_accel m/s².
        // This is the "discrete white noise" model: Q = q² * G * G^T
        //   where G = [dt²/2, dt²/2, dt, dt]^T (how acceleration enters state)
        const double qa = q_accel * q_accel;
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;
        Q_ = Mat4::zeros();
        Q_(0,0) = qa * dt4 / 4.0;  Q_(0,2) = qa * dt3 / 2.0;
        Q_(1,1) = qa * dt4 / 4.0;  Q_(1,3) = qa * dt3 / 2.0;
        Q_(2,0) = qa * dt3 / 2.0;  Q_(2,2) = qa * dt2;
        Q_(3,1) = qa * dt3 / 2.0;  Q_(3,3) = qa * dt2;

        // ── Measurement noise R ─────────────────────────────────────────────
        // GPS noise is assumed independent in x and y (diagonal R).
        const double rv = r_gps * r_gps;
        R_ = Mat2::zeros();
        R_(0,0) = rv;
        R_(1,1) = rv;

        // ── H matrix (constant — picks px, py from state) ───────────────────
        H_ = Mat24::zeros();
        H_(0,0) = 1.0;
        H_(1,1) = 1.0;
    }

    /// Initialise the filter state (call before first predict/update).
    void init(double px, double py, double vx, double vy,
              double pos_sigma = 5.0, double vel_sigma = 2.0) {
        s_.x(0,0) = px; s_.x(1,0) = py;
        s_.x(2,0) = vx; s_.x(3,0) = vy;
        s_.P = Mat4::zeros();
        s_.P(0,0) = pos_sigma * pos_sigma;
        s_.P(1,1) = pos_sigma * pos_sigma;
        s_.P(2,2) = vel_sigma * vel_sigma;
        s_.P(3,3) = vel_sigma * vel_sigma;
    }

    /// Predict step: advance state by dt seconds using constant-velocity model.
    void predict(double dt) {
        // Build F for this timestep (handles variable dt gracefully)
        auto F = Mat4::identity();
        F(0,2) = dt;
        F(1,3) = dt;

        s_.x = F * s_.x;
        s_.P = F * s_.P * transpose(F) + Q_;
    }

    /// Update step: fuse a GPS measurement z = [px, py].
    ///
    /// Challenge #2 answer:
    ///   K = P^- H^T (H P^- H^T + R)^{-1}
    ///   When R → ∞: denominator dominates, K → 0.
    ///   K = 0 means the update equation  x = x^- + K·y  changes nothing.
    ///   The state stays at the prediction; the GPS measurement is ignored.
    ///   This is sensible — if the sensor is pure noise (R huge), weighting it
    ///   zero is optimal.  The covariance P does NOT shrink either, so the
    ///   filter remains appropriately uncertain.
    void update(double z_px, double z_py) {
        Vec2 z;  z(0,0) = z_px;  z(1,0) = z_py;

        // Innovation: how far is the measurement from the prediction?
        const Vec2 y = z - H_ * s_.x;

        // Innovation covariance: uncertainty of the innovation
        const Mat2 S = H_ * s_.P * transpose(H_) + R_;

        // Kalman gain: optimal fusion weight
        const Mat42 K = s_.P * transpose(H_) * inverse2(S);

        // State update: move prediction toward measurement, weighted by K
        s_.x = s_.x + K * y;

        // Covariance update: incorporating the measurement reduced uncertainty
        const Mat4 I = Mat4::identity();
        s_.P = (I - K * H_) * s_.P;
    }

    const KFState& state()    const { return s_; }
    std::string_view name()   const { return "LinearKF"; }

    // Expose last innovation for consistency checking (visualize.py)
    Vec2 last_innovation(double z_px, double z_py) const {
        Vec2 z; z(0,0) = z_px; z(1,0) = z_py;
        return z - H_ * s_.x;
    }

private:
    KFState s_;
    Mat4  Q_;
    Mat2  R_;
    Mat24 H_;
};

// =============================================================================
// 2. Extended Kalman Filter — CTRV model (nonlinear)
// =============================================================================
// State: x = [px, py, v, θ]^T
//
// Process (nonlinear) — IMU provides ax (accel) and ω (yaw rate):
//   f(x, u) = [ px + v·cos(θ)·dt ]
//             [ py + v·sin(θ)·dt ]
//             [ v  + ax·dt       ]
//             [ θ  + ω·dt        ]
//
// Process Jacobian (∂f/∂x evaluated at current x̂):
//   F_jac = [[ 1,  0,  cos(θ)·dt,  −v·sin(θ)·dt ],
//             [ 0,  1,  sin(θ)·dt,   v·cos(θ)·dt ],
//             [ 0,  0,  1,           0            ],
//             [ 0,  0,  0,           1            ]]
//
// GPS measurement model (linear — h(x) = H·x):
//   H = [[1, 0, 0, 0],
//        [0, 1, 0, 0]]
//
// Challenge #3 answer:
//   The banana-shaped distribution of position arises because positions at
//   different possible headings fan out along an arc.  Propagating only the
//   mean underestimates uncertainty: the mean position sits inside the arc
//   but the actual uncertainty is spread along it.
//   Safety implication: if P (position covariance) is too small, the AV
//   collision-checker thinks the vehicle occupies a smaller area than it
//   actually could.  A pedestrian who is actually "probably in the path" gets
//   labelled "definitely not in the path," and the planner does not brake.
//   The Jacobian-based covariance propagation captures the first-order shape
//   of the banana — not perfect (sigma-point UKF does it exactly) but far
//   safer than ignoring it.
// =============================================================================
class EKF {
public:
    struct IMUInput {
        double ax    = 0.0;   // longitudinal acceleration (m/s²)
        double omega = 0.0;   // yaw rate (rad/s), positive = left turn
    };

    /// @param q_vel   Process noise std-dev on velocity  (m/s per √s)
    /// @param q_yaw   Process noise std-dev on yaw rate  (rad per √s)
    /// @param r_gps   GPS position noise std-dev (m)
    explicit EKF(double q_vel, double q_yaw, double r_gps) {
        Q_ = Mat4::zeros();
        Q_(2,2) = q_vel * q_vel;
        Q_(3,3) = q_yaw * q_yaw;

        const double rv = r_gps * r_gps;
        R_ = Mat2::zeros();
        R_(0,0) = rv;  R_(1,1) = rv;

        H_ = Mat24::zeros();
        H_(0,0) = 1.0;  H_(1,1) = 1.0;
    }

    void init(double px, double py, double v, double theta,
              double pos_sigma = 5.0, double vel_sigma = 2.0,
              double yaw_sigma = 0.5) {
        s_.x(0,0) = px;    s_.x(1,0) = py;
        s_.x(2,0) = v;     s_.x(3,0) = theta;
        s_.P = Mat4::zeros();
        s_.P(0,0) = pos_sigma * pos_sigma;
        s_.P(1,1) = pos_sigma * pos_sigma;
        s_.P(2,2) = vel_sigma * vel_sigma;
        s_.P(3,3) = yaw_sigma * yaw_sigma;
    }

    /// Predict step: propagate state nonlinearly, covariance via Jacobian.
    void predict(const IMUInput& u, double dt) {
        const double px    = s_.x(0,0);
        const double py    = s_.x(1,0);
        const double v     = s_.x(2,0);
        const double theta = s_.x(3,0);

        // ── Nonlinear state propagation ─────────────────────────────────────
        s_.x(0,0) = px + v * std::cos(theta) * dt;
        s_.x(1,0) = py + v * std::sin(theta) * dt;
        s_.x(2,0) = v  + u.ax    * dt;
        s_.x(3,0) = wrap(theta + u.omega * dt);

        // ── Jacobian of f(x, u) w.r.t. x ───────────────────────────────────
        // Computed at the OLD state (before propagation), per convention.
        Mat4 Fj = Mat4::identity();
        Fj(0,2) =  std::cos(theta) * dt;
        Fj(0,3) = -v * std::sin(theta) * dt;
        Fj(1,2) =  std::sin(theta) * dt;
        Fj(1,3) =  v * std::cos(theta) * dt;
        // rows 2 and 3 are already identity (velocity and yaw walk linearly)

        // ── Covariance prediction ───────────────────────────────────────────
        s_.P = Fj * s_.P * transpose(Fj) + Q_;
    }

    /// Update step: identical to linear KF (GPS measurement is linear in state).
    void update(double z_px, double z_py) {
        Vec2 z;  z(0,0) = z_px;  z(1,0) = z_py;

        const Vec2 y = z - H_ * s_.x;     // innovation

        const Mat2  S  = H_ * s_.P * transpose(H_) + R_;
        const Mat42 K  = s_.P * transpose(H_) * inverse2(S);

        s_.x = s_.x + K * y;
        s_.x(3,0) = wrap(s_.x(3,0));      // keep θ in (−π, π]

        const Mat4 I = Mat4::identity();
        s_.P = (I - K * H_) * s_.P;
    }

    const KFState& state()  const { return s_; }
    std::string_view name() const { return "EKF"; }

    Vec2 last_innovation(double z_px, double z_py) const {
        Vec2 z; z(0,0) = z_px; z(1,0) = z_py;
        return z - H_ * s_.x;
    }

private:
    KFState s_;
    Mat4  Q_;
    Mat2  R_;
    Mat24 H_;
};

} // namespace estimation
