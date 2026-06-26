// =============================================================================
// controllers.hpp  —  Three lateral path-tracking controllers
// =============================================================================
// All three controllers share a common interface (LateralController) and
// consume the same inputs: a BicycleState and a ReferencePath.
// Each returns a steering angle δ (radians) for the current timestep.
//
// Controllers implemented:
//   1. PurePursuit  — geometric lookahead (1990s robotics, still widely used)
//   2. Stanley      — CTE + heading feedback (Stanford DARPA 2005 winner)
//   3. LQRLateral   — optimal linear-quadratic regulator (2-state linearised)
//
// Dependency: requires bicycle_model.hpp + reference_path.hpp from project3.
// =============================================================================
#pragma once

#include "bicycle_model.hpp"
#include "reference_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string_view>

namespace control {

// =============================================================================
// Abstract interface — every lateral controller implements this
// =============================================================================
class LateralController {
public:
    virtual ~LateralController() = default;

    /// Compute steering angle δ (rad) for the current vehicle state.
    virtual double compute(const vehicle::State&         state,
                           const vehicle::ReferencePath& path) = 0;

    virtual std::string_view name() const = 0;
};

// =============================================================================
// 1. Pure Pursuit
// =============================================================================
// Algorithm:
//   a) Find the "lookahead point" Ld metres ahead of the vehicle along the path.
//   b) Compute angle α: bearing to that point in the vehicle's own frame.
//   c) Steer along the arc that connects the rear axle to the lookahead point.
//
// Steering formula: δ = arctan(2L·sin(α) / Ld)
//
// Speed-adaptive lookahead: Ld = clamp(k·v, Ld_min, Ld_max)
//   Short Ld → aggressive, jerky, can oscillate (reacts to every bump)
//   Long  Ld → smooth, but slow to correct → large steady-state CTE on curves
//
// Challenge #1 answer is baked into those two bullet points above.
// =============================================================================

struct PurePursuitParams {
    double wheelbase       = 2.7;   // L (m)
    double lookahead_gain  = 0.3;   // k: Ld = k * v
    double min_lookahead   = 2.0;   // m — floor at very low speeds
    double max_lookahead   = 20.0;  // m — cap at high speeds
};

class PurePursuit : public LateralController {
public:
    explicit PurePursuit(PurePursuitParams p = {}) : p_(p) {}

    double compute(const vehicle::State& s, const vehicle::ReferencePath& path) override {
        const double L_d = std::clamp(p_.lookahead_gain * s.v,
                                      p_.min_lookahead, p_.max_lookahead);

        // ── Find lookahead point ──────────────────────────────────────────────
        // Start at nearest waypoint, walk forward until the first point that
        // is at least Ld metres from the vehicle.
        const auto&  wps     = path.waypoints();
        std::size_t  nearest = path.nearest_index(s.x, s.y);
        std::size_t  target  = wps.size() - 1;   // fallback: last point

        for (std::size_t i = nearest; i < wps.size(); ++i) {
            const double d = std::hypot(wps[i].x - s.x, wps[i].y - s.y);
            if (d >= L_d) { target = i; break; }
        }

        // ── Bearing angle α in vehicle frame ─────────────────────────────────
        // Rotate world-frame displacement (dx, dy) into vehicle frame:
        //   local_y = -dx·sin(θ) + dy·cos(θ)   (lateral component)
        const double dx      = wps[target].x - s.x;
        const double dy      = wps[target].y - s.y;
        const double local_y = -dx * std::sin(s.theta) + dy * std::cos(s.theta);

        // Clamp to avoid asin domain errors from floating-point rounding
        const double sin_alpha = std::clamp(local_y / L_d, -1.0, 1.0);
        const double alpha     = std::asin(sin_alpha);

        // ── Steering angle ────────────────────────────────────────────────────
        return std::atan2(2.0 * p_.wheelbase * std::sin(alpha), L_d);
    }

    std::string_view name() const override { return "PurePursuit"; }

private:
    PurePursuitParams p_;
};

// =============================================================================
// 2. Stanley Controller
// =============================================================================
// Named after Stanford's car "Stanley" that won the 2005 DARPA Grand Challenge.
//
// Formula: δ = ψ_e + arctan(k·e_CTE / (v + ε))
//
//   ψ_e       — heading error: vehicle heading minus path tangent heading
//               Corrects the direction the car is facing.
//   k·e_CTE/v — CTE correction, inversely scaled by speed:
//               At high speed, even small steering corrects CTE quickly.
//               At low  speed, a larger angle is needed for the same correction.
//               ε (k_soft) prevents division by zero at standstill.
//
// Challenge #2 answer: without v in the denominator, a stationary car with
// e_CTE = 1m would compute δ = arctan(0.5 * 1 / 0) = ±90° → full lock.
// The softening constant ε keeps the angle finite and physically meaningful.
// =============================================================================

struct StanleyParams {
    double k_cte  = 0.5;   // CTE gain — larger → tighter path tracking
    double k_soft = 1.0;   // ε — softening constant (prevents ∞ at v≈0)
};

class Stanley : public LateralController {
public:
    explicit Stanley(StanleyParams p = {}) : p_(p) {}

    double compute(const vehicle::State& s, const vehicle::ReferencePath& path) override {
        const double cte = path.cross_track_error(s);
        const double he  = path.heading_error(s);

        // Stanley formula — wrap output to keep δ in (−π, π]
        // Sign note: our CTE is positive=LEFT and heading_error is positive=LEFT.
        // Original Stanley uses opposite sign conventions, so both terms are negated:
        //   δ = −ψ_e − arctan(k·e_CTE / (v + ε))
        return vehicle::wrap_angle(-he - std::atan2(p_.k_cte * cte, p_.k_soft + s.v));
    }

    std::string_view name() const override { return "Stanley"; }

private:
    StanleyParams p_;
};

// =============================================================================
// 3. LQR Lateral Controller (2-state, linearised kinematic bicycle)
// =============================================================================
// We linearise the bicycle model around a straight reference path at constant
// speed v, giving the error-state dynamics:
//
//   ė_CTE     = v · e_heading
//   ė_heading = (v/L) · δ
//
// Continuous system:  A = [[0, v], [0, 0]],  B = [[0], [v/L]]
//
// Discretised (Euler, timestep dt):
//   Ad = [[1, v·dt], [0, 1]]
//   Bd = [[0], [(v/L)·dt]]
//
// LQR cost:  J = Σ ( x^T Q x + u^T R u )
//   Q = diag(q_cte, q_heading)  — penalise position and heading errors
//   R = r_steer                 — penalise steering effort
//
// Challenge #3 answer: q_cte=100, r=0.001 → controller will command large,
// rapid steering to eliminate any CTE instantly. On a real car this means
// aggressive wheel inputs → passenger discomfort, tyre wear, potential loss
// of control at high speed. Production systems use R ≈ Q to balance comfort
// and precision.
//
// Gains are computed once at construction via discrete-time Riccati iteration.
// (A production system would recompute gains as v changes — see Project 6.)
// =============================================================================

struct LQRLateralParams {
    double wheelbase     = 2.7;    // L (m)
    double nominal_speed = 10.0;   // m/s — speed at which gains are computed
    double dt            = 0.05;   // s   — discretisation timestep
    double q_cte         = 5.0;    // Q[0,0]: CTE cost weight
    double q_heading     = 1.0;    // Q[1,1]: heading error cost weight
    double r_steer       = 0.1;    // R:      steering effort cost weight
    int    riccati_iters = 500;    // Riccati value-iteration steps
};

class LQRLateral : public LateralController {
public:
    explicit LQRLateral(LQRLateralParams p = {}) : p_(p) {
        K_ = solve_gains(p_.nominal_speed);
    }

    double compute(const vehicle::State& s, const vehicle::ReferencePath& path) override {
        const double e_cte     = path.cross_track_error(s);
        const double e_heading = path.heading_error(s);
        // Feedback law: δ = −K · [e_cte, e_heading]
        return vehicle::wrap_angle(-(K_[0] * e_cte + K_[1] * e_heading));
    }

    std::string_view name()          const override { return "LQR"; }
    const std::array<double,2>& gains() const       { return K_; }

private:
    LQRLateralParams      p_;
    std::array<double, 2> K_{};

    // ── Minimal 2×2 matrix algebra (no external dependency) ──────────────────
    // Stored row-major: M = {M[0][0], M[0][1], M[1][0], M[1][1]}
    using M2 = std::array<double, 4>;

    static constexpr double& elem(M2& m, int r, int c)       { return m[r*2+c]; }
    static constexpr double  elem(const M2& m, int r, int c) { return m[r*2+c]; }

    static M2 mul(const M2& A, const M2& B) {
        M2 C{};
        for (int i=0;i<2;i++)
            for (int j=0;j<2;j++)
                for (int k=0;k<2;k++)
                    elem(C,i,j) += elem(A,i,k) * elem(B,k,j);
        return C;
    }
    static M2 add(const M2& A, const M2& B) {
        return {A[0]+B[0], A[1]+B[1], A[2]+B[2], A[3]+B[3]};
    }
    static M2 sub(const M2& A, const M2& B) {
        return {A[0]-B[0], A[1]-B[1], A[2]-B[2], A[3]-B[3]};
    }
    static M2 T(const M2& A) { return {A[0], A[2], A[1], A[3]}; }   // transpose

    // ── Discrete-time algebraic Riccati iteration ─────────────────────────────
    // P ← Ad^T P Ad − (Ad^T P Bd)(R + Bd^T P Bd)^{-1}(Bd^T P Ad) + Q
    // B is 2×1: [Bd0; Bd1] — many products simplify to scalars.
    std::array<double, 2> solve_gains(double v) const {
        const double L  = p_.wheelbase;
        const double dt = p_.dt;

        // Discretised system matrices
        const M2    Ad  = {1.0, v*dt, 0.0, 1.0};      // 2×2
        const double Bd0 = 0.0;                         // B is 2×1
        const double Bd1 = (v / L) * dt;

        const M2 Q  = {p_.q_cte, 0.0, 0.0, p_.q_heading};
        const double R = p_.r_steer;

        M2 P = Q;   // initialise with Q (warm start)

        for (int iter = 0; iter < p_.riccati_iters; ++iter) {
            // Bd^T P Bd  (scalar, since B is 2×1)
            const double BtPB = Bd0*Bd0*elem(P,0,0)
                              + (Bd0*Bd1 + Bd1*Bd0)*(elem(P,0,1)+elem(P,1,0))*0.5
                              + Bd1*Bd1*elem(P,1,1);
            const double S_inv = 1.0 / (R + BtPB);

            // Ad^T P  (2×2)
            const M2 AtP = mul(T(Ad), P);

            // Ad^T P Bd  (2×1 column vector)
            const double AtPBd0 = elem(AtP,0,0)*Bd0 + elem(AtP,0,1)*Bd1;
            const double AtPBd1 = elem(AtP,1,0)*Bd0 + elem(AtP,1,1)*Bd1;

            // Rank-1 correction: (Ad^T P Bd)(Bd^T P Ad) / S
            const M2 correction = {
                AtPBd0 * AtPBd0 * S_inv,  AtPBd0 * AtPBd1 * S_inv,
                AtPBd1 * AtPBd0 * S_inv,  AtPBd1 * AtPBd1 * S_inv
            };

            P = sub(add(mul(AtP, Ad), Q), correction);
        }

        // K = (R + Bd^T P Bd)^{-1} · Bd^T P Ad   (1×2 row vector)
        const double BtPB = Bd0*Bd0*elem(P,0,0)
                          + Bd1*Bd1*elem(P,1,1);    // cross terms: Bd0=0 simplifies
        const double S_inv = 1.0 / (R + BtPB);

        // Bd^T P  (1×2 row)
        const double BtP0 = Bd0*elem(P,0,0) + Bd1*elem(P,1,0);
        const double BtP1 = Bd0*elem(P,0,1) + Bd1*elem(P,1,1);

        // Bd^T P Ad  (1×2 row)
        const double BtPAd0 = BtP0*elem(Ad,0,0) + BtP1*elem(Ad,1,0);
        const double BtPAd1 = BtP0*elem(Ad,0,1) + BtP1*elem(Ad,1,1);

        return {BtPAd0 * S_inv, BtPAd1 * S_inv};
    }
};

} // namespace control
