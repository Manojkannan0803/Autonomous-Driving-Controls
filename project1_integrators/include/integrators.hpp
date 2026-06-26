// =============================================================================
// integrators.hpp  —  Header-only numerical ODE integration library
// =============================================================================
// Implements three classic fixed-step methods for solving
//   dx/dt = f(t, x),   x(t0) = x0
//
// Methods        Global error order   Notes
// ──────────────────────────────────────────────────────────────────────────────
// Euler          O(dt)                simple, rarely used in production
// Midpoint/RK2   O(dt²)               2 evaluations of f per step
// Runge-Kutta 4  O(dt⁴)               4 evaluations; gold standard for ODEs
// =============================================================================
#pragma once

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <vector>

namespace numerics {

// ── Types ─────────────────────────────────────────────────────────────────────

/// State vector: one double per degree of freedom.
using State = std::vector<double>;

/// Derivative function signature:  f(t, x) → dx/dt
using DerivFn = std::function<State(double t, const State& x)>;

// ── Internal helpers ──────────────────────────────────────────────────────────

namespace detail {

/// result[i] = x[i] + alpha * y[i]
inline State axpy(const State& x, double alpha, const State& y) {
    State out(x.size());
    for (std::size_t i = 0; i < x.size(); ++i)
        out[i] = x[i] + alpha * y[i];
    return out;
}

/// RK4 weighted combine:  x + (dt/6) * (k1 + 2k2 + 2k3 + k4)
inline State rk4_combine(const State& x, double dt,
                          const State& k1, const State& k2,
                          const State& k3, const State& k4) {
    State out(x.size());
    for (std::size_t i = 0; i < x.size(); ++i)
        out[i] = x[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    return out;
}

} // namespace detail

// ── Single-step integrators ───────────────────────────────────────────────────

/// Forward Euler:  x_{n+1} = x_n + dt·f(t_n, x_n)
/// Global error O(dt).  Use only for quick prototyping.
inline State euler_step(const DerivFn& f, double t, const State& x, double dt) {
    return detail::axpy(x, dt, f(t, x));
}

/// Explicit midpoint (RK2):  global error O(dt²).
inline State rk2_step(const DerivFn& f, double t, const State& x, double dt) {
    const State k1    = f(t, x);
    const State x_mid = detail::axpy(x, dt * 0.5, k1);
    const State k2    = f(t + dt * 0.5, x_mid);
    return detail::axpy(x, dt, k2);
}

/// 4th-order Runge-Kutta:  global error O(dt⁴).  Industry standard.
/// Four evaluations of f per step; accurate enough for most control systems.
inline State rk4_step(const DerivFn& f, double t, const State& x, double dt) {
    const State k1 = f(t,            x);
    const State k2 = f(t + dt * 0.5, detail::axpy(x, dt * 0.5, k1));
    const State k3 = f(t + dt * 0.5, detail::axpy(x, dt * 0.5, k2));
    const State k4 = f(t + dt,       detail::axpy(x, dt,        k3));
    return detail::rk4_combine(x, dt, k1, k2, k3, k4);
}

// ── Full trajectory integration ───────────────────────────────────────────────

enum class Method { EULER, RK2, RK4 };

/// Time-series output from integrate().
struct Trajectory {
    std::vector<double> times;
    std::vector<State>  states;

    std::size_t  size()         const { return times.size(); }
    double       final_time()   const { return times.back(); }
    const State& final_state()  const { return states.back(); }
};

/// Integrate an ODE from t0 to t_end with a fixed step dt.
///
/// @param f       derivative function f(t, x) → dx/dt
/// @param t0      start time
/// @param t_end   end time (must be > t0)
/// @param x0      initial state
/// @param dt      timestep (seconds, must be > 0)
/// @param method  integration scheme (default: RK4)
/// @returns       Trajectory containing all (time, state) pairs
inline Trajectory integrate(const DerivFn& f,
                             double t0,
                             double t_end,
                             const State& x0,
                             double dt,
                             Method method = Method::RK4) {
    if (dt <= 0.0)
        throw std::invalid_argument("integrate: dt must be positive");
    if (t_end <= t0)
        throw std::invalid_argument("integrate: t_end must be greater than t0");

    Trajectory traj;
    traj.times.push_back(t0);
    traj.states.push_back(x0);

    double t = t0;
    State  x = x0;

    while (t < t_end - dt * 1.0e-10) {
        const double step = std::min(dt, t_end - t);
        switch (method) {
            case Method::EULER: x = euler_step(f, t, x, step); break;
            case Method::RK2:   x = rk2_step  (f, t, x, step); break;
            case Method::RK4:   x = rk4_step  (f, t, x, step); break;
        }
        t += step;
        traj.times.push_back(t);
        traj.states.push_back(x);
    }

    return traj;
}

} // namespace numerics
