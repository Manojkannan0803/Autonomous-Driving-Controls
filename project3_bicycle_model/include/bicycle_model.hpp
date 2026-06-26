// =============================================================================
// bicycle_model.hpp  —  Kinematic Bicycle Model
// =============================================================================
// Models a car as two wheels on a rigid chassis.
//
//  State  x = [x, y, θ, v]     position (m), heading (rad), speed (m/s)
//  Input  u = [δ, a]            steering angle (rad), acceleration (m/s²)
//
//  Equations of motion (kinematic — assumes no tyre slip):
//
//      ẋ = v · cos(θ)
//      ẏ = v · sin(θ)
//      θ̇ = (v / L) · tan(δ)       ← L = wheelbase
//      v̇ = a
//
//  Turning radius at constant steering angle δ:
//      R = L / tan(|δ|)
//
//  Integration: RK4 (same algorithm as Project 1, applied to the 4-D state).
//  State is kept self-contained — no external integrators.hpp dependency.
// =============================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>   // std::numbers::pi  (C++20)
#include <stdexcept>

namespace vehicle {

// ── Angle utilities ───────────────────────────────────────────────────────────

/// Wrap an angle into (-π, π].
/// Essential whenever we add/subtract headings — raw sums can exceed ±π
/// which would cause controllers to command the wrong direction.
inline double wrap_angle(double a) noexcept {
    // Equivalent to fmod(a + π, 2π) - π but handles negatives correctly.
    return a - 2.0 * std::numbers::pi
             * std::floor((a + std::numbers::pi) / (2.0 * std::numbers::pi));
}

// ── State ─────────────────────────────────────────────────────────────────────

/// Complete vehicle state in 2-D plane.
/// All four variables must be known to predict future motion.
struct State {
    double x     = 0.0;   ///< Position X (m), East-positive
    double y     = 0.0;   ///< Position Y (m), North-positive
    double theta = 0.0;   ///< Heading (rad), 0=East, π/2=North (CCW positive)
    double v     = 0.0;   ///< Forward speed (m/s)
};

// ── Control ───────────────────────────────────────────────────────────────────

/// Control inputs applied for one timestep.
struct Control {
    double delta = 0.0;   ///< Front-wheel steering angle (rad); positive = left
    double a     = 0.0;   ///< Longitudinal acceleration (m/s²); positive = throttle
};

// ── Bicycle model parameters (defined outside BicycleModel so it can be used
// as a default constructor argument — C++ requires the type to be complete) ──────

struct BicycleParams {
    double wheelbase  = 2.7;    ///< L: front-to-rear axle distance (m)
    double max_steer  = 0.524;  ///< Max |δ| ≈ 30° in radians
    double max_speed  = 50.0;   ///< m/s  (≈ 180 km/h)
    double min_speed  = 0.0;    ///< m/s  (no reverse in kinematic model)
    double max_accel  = 3.0;    ///< m/s²  throttle limit
    double max_brake  = 8.0;    ///< m/s²  braking deceleration magnitude
};

// ── BicycleModel ──────────────────────────────────────────────────────────────

class BicycleModel {
public:
    using Params = BicycleParams;  ///< Alias kept for API symmetry

    explicit BicycleModel(Params p = Params{}) : p_(std::move(p)) {
        if (p_.wheelbase <= 0.0)
            throw std::invalid_argument("BicycleModel: wheelbase must be positive");
        if (p_.max_steer <= 0.0)
            throw std::invalid_argument("BicycleModel: max_steer must be positive");
    }

    // ── Kinematic equations ───────────────────────────────────────────────────

    /// Compute state derivative f(s, u) = ds/dt.
    /// This is the "physics" of the bicycle model — called 4× per RK4 step.
    State derivative(const State& s, const Control& u) const noexcept {
        // Clamp inputs to physical limits before computing kinematics
        const double delta = std::clamp(u.delta, -p_.max_steer, p_.max_steer);
        const double accel = (u.a >= 0.0)
            ?  std::min(u.a,  p_.max_accel)   // throttle
            : -std::min(-u.a, p_.max_brake);  // brake (capped magnitude)

        return State{
            .x     = s.v * std::cos(s.theta),
            .y     = s.v * std::sin(s.theta),
            .theta = (s.v / p_.wheelbase) * std::tan(delta),
            .v     = accel
        };
    }

    /// Advance state by dt seconds using 4th-order Runge-Kutta.
    /// Control input is held constant over the interval (zero-order hold).
    State step(const State& s, const Control& u, double dt) const noexcept {
        // The same RK4 formula you implemented in Project 1, now applied to
        // a 4-component state instead of a std::vector<double>.
        const State k1 = derivative(s, u);
        const State k2 = derivative(add(s, scale(k1, dt * 0.5)), u);
        const State k3 = derivative(add(s, scale(k2, dt * 0.5)), u);
        const State k4 = derivative(add(s, scale(k3, dt)),       u);

        State next;
        next.x     = s.x     + (dt / 6.0) * (k1.x     + 2*k2.x     + 2*k3.x     + k4.x);
        next.y     = s.y     + (dt / 6.0) * (k1.y     + 2*k2.y     + 2*k3.y     + k4.y);
        next.v     = std::clamp(
                         s.v + (dt / 6.0) * (k1.v + 2*k2.v + 2*k3.v + k4.v),
                         p_.min_speed, p_.max_speed);
        // Wrap heading into (-π, π] after integration to avoid unbounded growth
        next.theta = wrap_angle(
                         s.theta + (dt / 6.0) * (k1.theta + 2*k2.theta + 2*k3.theta + k4.theta));
        return next;
    }

    // ── Derived quantities ────────────────────────────────────────────────────

    /// Theoretical turning radius at steering angle δ.
    /// Derivation: arc length = v·dt, heading change = (v/L)·tan(δ)·dt
    ///             radius = arc_length / heading_change = L / tan(δ)
    double turning_radius(double delta) const noexcept {
        const double d = std::clamp(std::abs(delta), 1.0e-6, p_.max_steer);
        return p_.wheelbase / std::tan(d);
    }

    const Params& params() const noexcept { return p_; }

private:
    Params p_;

    // ── RK4 state arithmetic (State is not a vector, so we define +/scale) ───
    static State add(const State& a, const State& b) noexcept {
        return {a.x + b.x, a.y + b.y, a.theta + b.theta, a.v + b.v};
    }
    static State scale(const State& s, double c) noexcept {
        return {s.x * c, s.y * c, s.theta * c, s.v * c};
    }
};

} // namespace vehicle
