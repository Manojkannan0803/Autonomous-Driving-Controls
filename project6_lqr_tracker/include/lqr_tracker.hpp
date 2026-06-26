// =============================================================================
// lqr_tracker.hpp  —  LQR Trajectory Tracker with time-varying gain schedule
// =============================================================================
// Precomputes one 2×3 LQR gain matrix K_i for every waypoint on the reference
// trajectory.  At each control step:
//
//   1. Find nearest trajectory point i (forward-biased search).
//   2. Compute error state: e = [e_CTE, e_ψ, e_v]
//   3. Curvature feedforward: δ_ff = arctan(L · κ_i)
//   4. Feedback:  δ = δ_ff − K_i[0,:] · e
//                 a =       − K_i[1,:] · e
//
// Gain computation (per waypoint):
//   Discrete linearised bicycle (error dynamics at v_ref_i, curvature κ_i):
//
//     A_d = [[1,  v·dt,  0 ],          B_d = [[0,      0  ],
//             [0,  1,    0 ],                  [v/L·dt, 0  ],
//             [0,  0,    1 ]]                  [0,      dt ]]
//
//   Q = diag(q_cte, q_heading, q_speed),  R = diag(r_steer, r_accel)
//   Solve discrete Riccati → K = (R + Bd^T P Bd)^{-1} Bd^T P Ad
//
// All matrix operations use a local Mat<R,C> template (same design as P5's
// estimation::Mat, but in namespace control — no cross-project include needed).
// =============================================================================
#pragma once

#include "trajectory.hpp"
#include "bicycle_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>
#include <vector>

namespace control {

// =============================================================================
// Minimal fixed-size matrix — row-major, same pattern as estimation::Mat (P5)
// Redeclared here in control:: to keep projects independent.
// =============================================================================
template<int R, int C>
struct Mat {
    std::array<double, R * C> data{};
    double& operator()(int r, int c)       { return data[r * C + c]; }
    double  operator()(int r, int c) const { return data[r * C + c]; }
    static Mat<R,C> zeros()    { Mat<R,C> m; m.data.fill(0.0); return m; }
    static Mat<R,C> identity() requires (R == C) {
        Mat<R,C> m; m.data.fill(0.0);
        for (int i = 0; i < R; ++i) m(i,i) = 1.0;
        return m;
    }
};

template<int R, int K, int C>
Mat<R,C> operator*(const Mat<R,K>& A, const Mat<K,C>& B) {
    auto out = Mat<R,C>::zeros();
    for (int r=0;r<R;r++) for (int c=0;c<C;c++) for (int k=0;k<K;k++)
        out(r,c) += A(r,k) * B(k,c);
    return out;
}
template<int R, int C>
Mat<R,C> operator+(const Mat<R,C>& A, const Mat<R,C>& B) {
    Mat<R,C> out; for (int i=0;i<R*C;i++) out.data[i]=A.data[i]+B.data[i]; return out;
}
template<int R, int C>
Mat<R,C> operator-(const Mat<R,C>& A, const Mat<R,C>& B) {
    Mat<R,C> out; for (int i=0;i<R*C;i++) out.data[i]=A.data[i]-B.data[i]; return out;
}
template<int R, int C>
Mat<C,R> transpose(const Mat<R,C>& A) {
    Mat<C,R> out;
    for (int r=0;r<R;r++) for (int c=0;c<C;c++) out(c,r)=A(r,c);
    return out;
}

/// Analytical 2×2 inverse (exact, no pivoting needed for well-conditioned R).
inline Mat<2,2> inverse2(const Mat<2,2>& M) {
    const double det = M(0,0)*M(1,1) - M(0,1)*M(1,0);
    if (std::abs(det) < 1.0e-14)
        throw std::runtime_error("inverse2: singular matrix in LQR Riccati");
    const double id = 1.0 / det;
    Mat<2,2> out;
    out(0,0)= M(1,1)*id; out(0,1)=-M(0,1)*id;
    out(1,0)=-M(1,0)*id; out(1,1)= M(0,0)*id;
    return out;
}

// =============================================================================
// LQR Tracker parameters
// =============================================================================
struct LQRTrackerParams {
    double wheelbase     = 2.7;    // L (m)
    double dt            = 0.02;   // control timestep (s) — matches simulation rate
    double q_cte         = 10.0;   // Q[0,0]: lateral error cost
    double q_heading     = 2.0;    // Q[1,1]: heading error cost
    double q_speed       = 5.0;    // Q[2,2]: speed error cost
    double r_steer       = 0.1;    // R[0,0]: steering effort cost
    double r_accel       = 0.5;    // R[1,1]: acceleration effort cost
    int    riccati_iters = 500;    // Riccati value-iteration steps
    bool   use_feedforward = true; // add δ_ff = arctan(L·κ) — set false for P4 comparison
};

// =============================================================================
// LQR Tracker
// =============================================================================
// Gain storage: for each trajectory point, store a GainRow = [K00,K01,K02, K10,K11,K12]
// (K is 2×3 row-major: row 0 = steering feedback, row 1 = acceleration feedback)
// =============================================================================
class LQRTracker {
public:
    using GainRow = std::array<double, 6>;   // K[2×3], row-major

    explicit LQRTracker(const Trajectory& traj, LQRTrackerParams p = {})
        : traj_(&traj), p_(p)
    {
        gains_.reserve(traj.size());
        for (std::size_t i = 0; i < traj.size(); ++i)
            gains_.push_back(solve_gains(traj[i].v_ref));
    }

    // ── Compute control output ────────────────────────────────────────────────
    struct Control { double delta = 0.0, accel = 0.0; };

    Control compute(const vehicle::State& s) {
        // Forward-biased nearest index — advances hint_ each call
        hint_ = traj_->nearest_index_forward(s.x, s.y, hint_);
        const std::size_t i = hint_;
        const TrajectoryPoint& ref = (*traj_)[i];

        // Error state
        const double e_cte     = traj_->cross_track_error(s, i);
        const double e_heading = vehicle::wrap_angle(s.theta - ref.theta);
        const double e_speed   = s.v - ref.v_ref;

        // Gains for this waypoint
        const GainRow& K = gains_[i];
        // K row 0 → steering,  K row 1 → acceleration
        const double steer_fb = -(K[0]*e_cte + K[1]*e_heading + K[2]*e_speed);
        const double accel_fb = -(K[3]*e_cte + K[4]*e_heading + K[5]*e_speed);

        // Curvature feedforward: the steering angle needed just to follow the curve
        // δ_ff = arctan(L · κ)  ≈  L · κ  for small curvatures
        const double delta_ff = p_.use_feedforward
                                ? std::atan2(p_.wheelbase * ref.kappa, 1.0)
                                : 0.0;

        return { vehicle::wrap_angle(delta_ff + steer_fb),
                 accel_fb };
    }

    // Accessors for plotting / testing
    const std::vector<GainRow>& gain_schedule()    const { return gains_; }
    const Trajectory*           trajectory()       const { return traj_; }
    std::string_view            name()             const { return "LQRTracker"; }
    std::size_t                 current_hint()     const { return hint_; }
    void reset_hint(std::size_t hint = 0) { hint_ = hint; }

private:
    const Trajectory*   traj_;
    LQRTrackerParams    p_;
    std::vector<GainRow> gains_;
    std::size_t         hint_ = 0;

    // ── Discrete Riccati solver (3-state, 2-input) ────────────────────────────
    GainRow solve_gains(double v) const {
        const double L  = p_.wheelbase;
        const double dt = p_.dt;

        // Linearised error dynamics (see module-level comments)
        using M33 = Mat<3,3>;
        using M32 = Mat<3,2>;
        using M23 = Mat<2,3>;
        using M22 = Mat<2,2>;

        M33 Ad = M33::identity();
        Ad(0,1) = v * dt;        // e_cte evolves with v·e_heading

        M32 Bd = M32::zeros();
        Bd(1,0) = (v / L) * dt;  // heading driven by steering
        Bd(2,1) = dt;            // speed driven by acceleration

        M33 Q = M33::zeros();
        Q(0,0) = p_.q_cte;  Q(1,1) = p_.q_heading;  Q(2,2) = p_.q_speed;

        M22 R = M22::zeros();
        R(0,0) = p_.r_steer;  R(1,1) = p_.r_accel;

        // Riccati iteration: P ← Ad^T P Ad + Q − (Ad^T P Bd)(R + Bd^T P Bd)^{-1}(Bd^T P Ad)
        M33 P = Q;
        for (int iter = 0; iter < p_.riccati_iters; ++iter) {
            const M33 AtP  = transpose(Ad) * P;
            const M33 AtPA = AtP * Ad;
            const M22 BtPB = transpose(Bd) * P * Bd;
            const M23 BtPA = transpose(Bd) * (P * Ad);
            const M32 AtPB = AtP * Bd;
            const M22 S_inv = inverse2(R + BtPB);
            P = AtPA + Q - AtPB * S_inv * BtPA;
        }

        // K = (R + Bd^T P Bd)^{-1} · Bd^T P Ad  →  2×3
        const M22 S_inv = inverse2(R + transpose(Bd) * P * Bd);
        const M23 K = S_inv * (transpose(Bd) * (P * Ad));

        return { K(0,0), K(0,1), K(0,2),
                 K(1,0), K(1,1), K(1,2) };
    }
};

} // namespace control
