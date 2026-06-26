// =============================================================================
// project2_pid_controller / tests / test_pid.cpp
//
// Unit tests for PIDController and VehiclePlant.
// Covers:
//   - Individual P / I / D term correctness
//   - Output saturation
//   - Anti-windup integral clamping
//   - Derivative low-pass filter
//   - Reset behaviour
//   - Invalid configuration rejection
//   - VehiclePlant physics (throttle, brake, non-negativity)
//   - Closed-loop convergence: PID + plant reaches target speed
// =============================================================================
#include "pid_controller.hpp"
#include "vehicle_plant.hpp"

#include <cmath>
#include <gtest/gtest.h>

using namespace controls;

// ── Helpers ───────────────────────────────────────────────────────────────────

static PIDController make_p_only(double Kp,
                                  double out_min = -1.0e9,
                                  double out_max =  1.0e9) {
    PIDController::Config cfg;
    cfg.gains        = {.Kp = Kp, .Ki = 0.0, .Kd = 0.0};
    cfg.output_min   = out_min;
    cfg.output_max   = out_max;
    cfg.integral_min = -1.0e9;
    cfg.integral_max =  1.0e9;
    return PIDController(cfg);
}

// =============================================================================
// PIDController — term isolation
// =============================================================================

TEST(PIDController, Proportional_OutputEqualsKp_times_error) {
    auto pid = make_p_only(3.0);
    EXPECT_NEAR(pid.update(5.0, 0.1), 15.0, 1.0e-12);
}

TEST(PIDController, Integral_AccumulatesOverTime) {
    PIDController::Config cfg;
    cfg.gains        = {.Kp = 0.0, .Ki = 1.0, .Kd = 0.0};
    cfg.output_min   = -1.0e9;
    cfg.output_max   =  1.0e9;
    cfg.integral_min = -1.0e9;
    cfg.integral_max =  1.0e9;
    PIDController pid(cfg);

    // After one step: integral = 1.0 * 0.1 = 0.1  → output = 0.1
    EXPECT_NEAR(pid.update(1.0, 0.1), 0.1, 1.0e-12);
    // After two steps: integral = 0.2  → output = 0.2
    EXPECT_NEAR(pid.update(1.0, 0.1), 0.2, 1.0e-12);
}

TEST(PIDController, Derivative_ZeroOnFirstCallFromReset) {
    // After reset, prev_error = 0.  First call: D = Kd*(e-0)/dt.
    PIDController::Config cfg;
    cfg.gains        = {.Kp = 0.0, .Ki = 0.0, .Kd = 2.0};
    cfg.output_min   = -1.0e9;
    cfg.output_max   =  1.0e9;
    cfg.integral_min = -1.0e9;
    cfg.integral_max =  1.0e9;
    PIDController pid(cfg);

    // D = 2.0 * (3.0 - 0.0) / 0.1 = 60
    EXPECT_NEAR(pid.update(3.0, 0.1), 60.0, 1.0e-10);
}

// =============================================================================
// Output saturation
// =============================================================================

TEST(PIDController, OutputClampedToUpperLimit) {
    auto pid = make_p_only(1000.0, -1.0, 1.0);
    EXPECT_DOUBLE_EQ(pid.update(10.0, 0.01), 1.0);
}

TEST(PIDController, OutputClampedToLowerLimit) {
    auto pid = make_p_only(1000.0, -1.0, 1.0);
    EXPECT_DOUBLE_EQ(pid.update(-10.0, 0.01), -1.0);
}

// =============================================================================
// Anti-windup
// =============================================================================

TEST(PIDController, Integral_ClampedByAntiWindup) {
    PIDController::Config cfg;
    cfg.gains        = {.Kp = 0.0, .Ki = 1.0, .Kd = 0.0};
    cfg.output_min   = -1.0e9;
    cfg.output_max   =  1.0e9;
    cfg.integral_min = -0.5;
    cfg.integral_max =  0.5;
    PIDController pid(cfg);

    // Accumulate many steps — integral should saturate at 0.5
    for (int i = 0; i < 2000; ++i)
        pid.update(1.0, 0.1);

    EXPECT_NEAR(pid.integral(), 0.5, 1.0e-12);
}

// =============================================================================
// Derivative low-pass filter
// =============================================================================

TEST(PIDController, DerivativeFilter_ReducesOutputVariance) {
    // With no filter, a noisy error signal amplifies the D-term greatly.
    // With a filter (τ = 0.1), successive outputs should be smoother.
    PIDController::Config cfg_nofilter;
    cfg_nofilter.gains        = {.Kp = 0.0, .Ki = 0.0, .Kd = 1.0};
    cfg_nofilter.output_min   = -1.0e9;
    cfg_nofilter.output_max   =  1.0e9;
    cfg_nofilter.integral_min = -1.0e9;
    cfg_nofilter.integral_max =  1.0e9;
    cfg_nofilter.derivative_tau = 0.0;

    PIDController::Config cfg_filter = cfg_nofilter;
    cfg_filter.derivative_tau = 0.1;

    PIDController pid_nf(cfg_nofilter);
    PIDController pid_f(cfg_filter);

    // Alternating ±1 error signal at dt=0.01 → raw derivative = ±200
    double sum_sq_nf = 0.0, sum_sq_f = 0.0;
    for (int i = 0; i < 100; ++i) {
        const double e   = (i % 2 == 0) ? 1.0 : -1.0;
        const double out_nf = pid_nf.update(e, 0.01);
        const double out_f  = pid_f.update(e,  0.01);
        sum_sq_nf += out_nf * out_nf;
        sum_sq_f  += out_f  * out_f;
    }
    EXPECT_LT(sum_sq_f, sum_sq_nf);   // filtered has lower output variance
}

// =============================================================================
// Reset
// =============================================================================

TEST(PIDController, Reset_ClearsAllState) {
    PIDController::Config cfg;
    cfg.gains        = {.Kp = 1.0, .Ki = 1.0, .Kd = 0.0};
    cfg.output_min   = -1.0e9;
    cfg.output_max   =  1.0e9;
    cfg.integral_min = -1.0e9;
    cfg.integral_max =  1.0e9;
    PIDController pid(cfg);

    pid.update(5.0, 0.1);
    pid.update(5.0, 0.1);
    pid.reset();

    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);

    // First update after reset: I = Ki * (error * dt) = 1 * 2 * 0.1 = 0.2
    //                           P = Kp * error        = 1 * 2.0     = 2.0
    //                           Total = 2.2
    EXPECT_NEAR(pid.update(2.0, 0.1), 2.2, 1.0e-12);
}

// =============================================================================
// Configuration validation
// =============================================================================

TEST(PIDController, InvalidConfig_OutputMinGtMax_Throws) {
    PIDController::Config cfg;
    cfg.output_min = 1.0;
    cfg.output_max = -1.0;
    EXPECT_THROW(PIDController pid(cfg), std::invalid_argument);
}

TEST(PIDController, InvalidConfig_IntegralMinGtMax_Throws) {
    PIDController::Config cfg;
    cfg.output_min   = -1.0;
    cfg.output_max   =  1.0;
    cfg.integral_min =  5.0;
    cfg.integral_max = -5.0;
    EXPECT_THROW(PIDController pid(cfg), std::invalid_argument);
}

// =============================================================================
// VehiclePlant
// =============================================================================

TEST(VehiclePlant, ThrottleIncreasesVelocity) {
    VehiclePlant p;
    p.velocity = 0.0;
    p.step(1.0, 0.1);
    EXPECT_GT(p.velocity, 0.0);
}

TEST(VehiclePlant, BrakeDecreasesVelocity) {
    VehiclePlant p;
    p.velocity = 20.0;
    p.step(-1.0, 0.5);
    EXPECT_LT(p.velocity, 20.0);
}

TEST(VehiclePlant, VelocityNeverGoesNegative) {
    VehiclePlant p;
    p.velocity = 0.0;
    p.step(-1.0, 100.0);   // full brake from standstill for 100 s
    EXPECT_GE(p.velocity, 0.0);
}

TEST(VehiclePlant, CoastingDeceleratesDueToDrag) {
    VehiclePlant p;
    p.velocity = 30.0;
    p.step(0.0, 5.0);   // no throttle, no brake
    EXPECT_LT(p.velocity, 30.0);
}

// =============================================================================
// Closed-loop integration test: PID + plant converges to target
// =============================================================================

TEST(PIDClosedLoop, ConvergesToTarget_Within50s) {
    PIDController::Config cfg;
    cfg.gains          = {.Kp = 0.4, .Ki = 0.08, .Kd = 0.2};
    cfg.output_min     = -1.0;
    cfg.output_max     =  1.0;
    cfg.integral_min   = -5.0;
    cfg.integral_max   =  5.0;
    cfg.derivative_tau =  0.05;
    PIDController pid(cfg);

    VehiclePlant plant;
    const double target = 100.0 / 3.6;   // 100 km/h
    const double dt     = 0.01;           // 100 Hz

    for (int i = 0; i < 5000; ++i) {     // 50 s of simulation
        const double cmd = pid.update(target - plant.velocity, dt);
        plant.step(cmd, dt);
    }

    // Accept within ±1% of target
    EXPECT_NEAR(plant.velocity, target, target * 0.01);
}
