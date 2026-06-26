// =============================================================================
// project1_integrators / tests / test_integrators.cpp
//
// Unit tests for the numerics::integrate() library.
// Verifies:
//   - Correctness  against analytical solutions
//   - Convergence  rates  O(dt) Euler, O(dt²) RK2, O(dt⁴) RK4
//   - Conservation  energy in a Hamiltonian system (harmonic oscillator)
//   - Input validation  (invalid dt, invalid time range)
//   - Trajectory  properties  (starts at x0, ends near t_end)
// =============================================================================
#include "integrators.hpp"

#include <cmath>
#include <gtest/gtest.h>

// ── Shared ODE factories ──────────────────────────────────────────────────────

static numerics::DerivFn make_exp_decay(double lambda) {
    return [lambda](double /*t*/, const numerics::State& x) -> numerics::State {
        return {-lambda * x[0]};
    };
}

static numerics::DerivFn make_harmonic(double omega) {
    return [omega](double /*t*/, const numerics::State& s) -> numerics::State {
        return {s[1], -omega * omega * s[0]};
    };
}

static double exp_exact(double t, double y0, double lambda) {
    return y0 * std::exp(-lambda * t);
}

static double l2(const numerics::State& a, const numerics::State& b) {
    double s = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double d = a[i] - b[i];
        s += d * d;
    }
    return std::sqrt(s);
}

// =============================================================================
// Euler
// =============================================================================

TEST(EulerStep, SingleStep_ExpDecay) {
    // x0=1, f(t,x)=-x  =>  x1 = 1 + 0.1*(-1) = 0.9
    auto f = make_exp_decay(1.0);
    auto x1 = numerics::euler_step(f, 0.0, {1.0}, 0.1);
    EXPECT_NEAR(x1[0], 0.9, 1.0e-12);
}

TEST(EulerIntegrate, GlobalError_Linear_in_dt) {
    // Halving dt should halve the global error (O(dt) convergence).
    auto f = make_exp_decay(2.0);
    const numerics::State x0{1.0};
    const double t_end = 1.0;
    const double exact  = exp_exact(t_end, 1.0, 2.0);

    const auto t1 = numerics::integrate(f, 0.0, t_end, x0, 0.01,  numerics::Method::EULER);
    const auto t2 = numerics::integrate(f, 0.0, t_end, x0, 0.001, numerics::Method::EULER);

    const double err1 = std::abs(t1.final_state()[0] - exact);
    const double err2 = std::abs(t2.final_state()[0] - exact);
    // 10× smaller dt → ~10× smaller error
    EXPECT_GT(err1 / err2, 8.0);
    EXPECT_LT(err1 / err2, 12.0);
}

// =============================================================================
// RK2
// =============================================================================

TEST(RK2Integrate, MoreAccurateThanEuler) {
    auto f = make_exp_decay(1.0);
    const numerics::State x0{1.0};
    const double t_end = 2.0;
    const double exact  = exp_exact(t_end, 1.0, 1.0);
    const double dt     = 0.1;

    const auto t_euler = numerics::integrate(f, 0.0, t_end, x0, dt, numerics::Method::EULER);
    const auto t_rk2   = numerics::integrate(f, 0.0, t_end, x0, dt, numerics::Method::RK2);

    const double err_euler = std::abs(t_euler.final_state()[0] - exact);
    const double err_rk2   = std::abs(t_rk2.final_state()[0]   - exact);
    EXPECT_LT(err_rk2, err_euler);
}

// =============================================================================
// RK4
// =============================================================================

TEST(RK4Integrate, ExpDecay_HighAccuracy) {
    // With dt=0.1 and t_end=2, error should be well below 1e-6
    auto f = make_exp_decay(1.0);
    const auto traj = numerics::integrate(f, 0.0, 2.0, {1.0}, 0.1, numerics::Method::RK4);
    const double exact = exp_exact(2.0, 1.0, 1.0);
    EXPECT_NEAR(traj.final_state()[0], exact, 1.0e-6);
}

TEST(RK4Integrate, GlobalError_O_dt4) {
    // Halving dt should reduce error by ~16× (2^4 = 16).
    auto f = make_exp_decay(1.0);
    const numerics::State x0{1.0};
    const double t_end = 1.0;
    const double exact  = exp_exact(t_end, 1.0, 1.0);

    const auto t1 = numerics::integrate(f, 0.0, t_end, x0, 0.1,  numerics::Method::RK4);
    const auto t2 = numerics::integrate(f, 0.0, t_end, x0, 0.05, numerics::Method::RK4);

    const double err1 = std::abs(t1.final_state()[0] - exact);
    const double err2 = std::abs(t2.final_state()[0] - exact);
    // Expect ratio ≈ 16, accept [12, 20] to tolerate round-off near t_end
    EXPECT_GT(err1 / err2, 12.0);
    EXPECT_LT(err1 / err2, 20.0);
}

TEST(RK4Integrate, HarmonicOscillator_EnergyConservation) {
    // Total mechanical energy E = ½(v² + ω²x²) must stay constant for
    // an undamped harmonic oscillator.
    const double omega = 2.0;
    auto f = make_harmonic(omega);
    const numerics::State x0{1.0, 0.0};  // x=1, v=0
    const double E0 = 0.5 * (x0[1] * x0[1] + omega * omega * x0[0] * x0[0]);

    const auto traj = numerics::integrate(f, 0.0, 10.0, x0, 0.01, numerics::Method::RK4);

    const auto&  xf = traj.final_state();
    const double Ef = 0.5 * (xf[1] * xf[1] + omega * omega * xf[0] * xf[0]);
    EXPECT_NEAR(Ef, E0, 1.0e-6);
}

// =============================================================================
// Input validation
// =============================================================================

TEST(Integrate, NegativeDt_Throws) {
    auto f = make_exp_decay(1.0);
    EXPECT_THROW(numerics::integrate(f, 0.0, 1.0, {1.0}, -0.01), std::invalid_argument);
}

TEST(Integrate, ZeroDt_Throws) {
    auto f = make_exp_decay(1.0);
    EXPECT_THROW(numerics::integrate(f, 0.0, 1.0, {1.0}, 0.0), std::invalid_argument);
}

TEST(Integrate, ReversedTimeRange_Throws) {
    auto f = make_exp_decay(1.0);
    EXPECT_THROW(numerics::integrate(f, 1.0, 0.0, {1.0}, 0.01), std::invalid_argument);
}

// =============================================================================
// Trajectory structure
// =============================================================================

TEST(Trajectory, StartsAtInitialCondition) {
    auto traj = numerics::integrate(make_exp_decay(1.0), 0.0, 1.0, {7.5}, 0.1);
    EXPECT_DOUBLE_EQ(traj.times[0],     0.0);
    EXPECT_DOUBLE_EQ(traj.states[0][0], 7.5);
}

TEST(Trajectory, EndsAtTEnd) {
    auto traj = numerics::integrate(make_exp_decay(1.0), 0.0, 2.0, {1.0}, 0.1);
    EXPECT_NEAR(traj.final_time(), 2.0, 1.0e-10);
}

TEST(Trajectory, HasExpectedNumberOfPoints) {
    // t0=0, t_end=1, dt=0.1 → 11 points (including start)
    auto traj = numerics::integrate(make_exp_decay(1.0), 0.0, 1.0, {1.0}, 0.1);
    EXPECT_EQ(traj.size(), 11u);
}

TEST(Trajectory, RK4MoreAccurateThanEuler_Harmonic) {
    const double omega = 2.0;
    auto f = make_harmonic(omega);
    const numerics::State x0{1.0, 0.0};
    const double t_end = 5.0;
    const double dt    = 0.05;

    const auto t_euler = numerics::integrate(f, 0.0, t_end, x0, dt, numerics::Method::EULER);
    const auto t_rk4   = numerics::integrate(f, 0.0, t_end, x0, dt, numerics::Method::RK4);

    const numerics::State exact{std::cos(omega * t_end), -omega * std::sin(omega * t_end)};
    EXPECT_LT(l2(t_rk4.final_state(), exact), l2(t_euler.final_state(), exact));
}
