// =============================================================================
// project4_lateral_controllers / tests / test_controllers.cpp
//
// Tests organised by controller type, each covering:
//   - On-path behaviour (error ≈ 0 → steering ≈ 0)
//   - Correct sign convention (left of path → steer right = negative δ)
//   - Convergence: closed-loop CTE reduces over time
//   - Edge cases: stationary vehicle, extreme error
// =============================================================================
#include "controllers.hpp"
#include "bicycle_model.hpp"
#include "reference_path.hpp"

#include <cmath>
#include <numbers>
#include <gtest/gtest.h>

using namespace vehicle;
using namespace control;

// ── Helpers ───────────────────────────────────────────────────────────────────

// Straight east-going path from (0,0) to (100,0)
static ReferencePath make_straight_path() {
    return ReferencePath::straight(100.0, 50);
}

// Vehicle perfectly on path (y=0) heading East, travelling at 10 m/s
static State on_path_state(double x = 10.0) {
    return {.x = x, .y = 0.0, .theta = 0.0, .v = 10.0};
}

// =============================================================================
// Pure Pursuit
// =============================================================================

TEST(PurePursuit, OnPath_AlignedHeading_SteeringNearZero) {
    PurePursuit ctrl;
    const auto path  = make_straight_path();
    const auto state = on_path_state(5.0);
    // Vehicle is on a straight path facing forward — no correction needed
    EXPECT_NEAR(ctrl.compute(state, path), 0.0, 0.05);
}

TEST(PurePursuit, LeftOfPath_SteeringNegative) {
    // Vehicle is above (y>0 = left) the eastward path → must steer right (δ < 0)
    PurePursuit ctrl;
    const auto path  = make_straight_path();
    const State s{.x = 10.0, .y = 1.5, .theta = 0.0, .v = 10.0};
    EXPECT_LT(ctrl.compute(s, path), 0.0);
}

TEST(PurePursuit, RightOfPath_SteeringPositive) {
    PurePursuit ctrl;
    const auto path  = make_straight_path();
    const State s{.x = 10.0, .y = -1.5, .theta = 0.0, .v = 10.0};
    EXPECT_GT(ctrl.compute(s, path), 0.0);
}

TEST(PurePursuit, HigherSpeed_LargerLookahead_SmoothOutput) {
    // At higher speed the lookahead is larger → steering magnitude should be smaller
    // for the same lateral offset.
    PurePursuit ctrl;
    const auto path = make_straight_path();
    const State s_slow{.x=5.0, .y=1.0, .theta=0.0, .v= 5.0};
    const State s_fast{.x=5.0, .y=1.0, .theta=0.0, .v=20.0};
    const double d_slow = std::abs(ctrl.compute(s_slow, path));
    const double d_fast = std::abs(ctrl.compute(s_fast, path));
    // Faster → larger Ld → smaller steering correction
    EXPECT_LT(d_fast, d_slow);
}

// =============================================================================
// Stanley
// =============================================================================

TEST(Stanley, OnPath_AlignedHeading_SteeringZero) {
    Stanley ctrl;
    const auto path  = make_straight_path();
    const auto state = on_path_state(10.0);
    EXPECT_NEAR(ctrl.compute(state, path), 0.0, 1.0e-9);
}

TEST(Stanley, LeftOfPath_SteeringNegative) {
    Stanley ctrl;
    const auto path = make_straight_path();
    // Left of path (y > 0), heading aligned → only CTE term active → steer right (δ<0)
    const State s{.x = 10.0, .y = 1.0, .theta = 0.0, .v = 10.0};
    EXPECT_LT(ctrl.compute(s, path), 0.0);
}

TEST(Stanley, HeadingErrorOnly_CorrectSign) {
    Stanley ctrl;
    const auto path = make_straight_path();
    // On path (y=0) but heading 10° left of path → steer right
    const State s{.x=10.0, .y=0.0, .theta=10.0*std::numbers::pi/180.0, .v=10.0};
    EXPECT_LT(ctrl.compute(s, path), 0.0);  // heading_error = +10° → δ < 0
}

TEST(Stanley, LowSpeedDoesNotProduceExcessiveSteering) {
    // ε (k_soft) prevents blow-up at low speed
    StanleyParams p; p.k_soft = 1.0;
    Stanley ctrl(p);
    const auto path = make_straight_path();
    const State s_slow{.x=5.0, .y=0.5, .theta=0.0, .v=0.001};
    const double delta = ctrl.compute(s_slow, path);
    // Should not exceed max_steer (~30°)
    EXPECT_LT(std::abs(delta), std::numbers::pi / 6.0);
}

// =============================================================================
// LQR
// =============================================================================

TEST(LQRLateral, GainsArePositive) {
    // Both gains must be positive so the feedback drives errors to zero
    LQRLateral ctrl;
    EXPECT_GT(ctrl.gains()[0], 0.0);   // k_cte
    EXPECT_GT(ctrl.gains()[1], 0.0);   // k_heading
}

TEST(LQRLateral, OnPath_AlignedHeading_SteeringZero) {
    LQRLateral ctrl;
    const auto path  = make_straight_path();
    const auto state = on_path_state(10.0);
    EXPECT_NEAR(ctrl.compute(state, path), 0.0, 1.0e-9);
}

TEST(LQRLateral, LeftOfPath_SteeringNegative) {
    LQRLateral ctrl;
    const auto path = make_straight_path();
    const State s{.x=10.0, .y=1.0, .theta=0.0, .v=10.0};
    EXPECT_LT(ctrl.compute(s, path), 0.0);
}

TEST(LQRLateral, HigherCTEWeight_LargerGain) {
    // Larger q_cte should produce a larger k_cte gain
    LQRLateralParams p_low  = {.q_cte = 1.0,  .q_heading = 1.0, .r_steer = 0.1};
    LQRLateralParams p_high = {.q_cte = 50.0, .q_heading = 1.0, .r_steer = 0.1};
    LQRLateral ctrl_low(p_low);
    LQRLateral ctrl_high(p_high);
    EXPECT_GT(ctrl_high.gains()[0], ctrl_low.gains()[0]);
}

TEST(LQRLateral, HigherSteeringCost_SmallerGain) {
    // Larger r_steer should produce smaller gains (controller is more conservative)
    LQRLateralParams p_cheap = {.q_cte = 5.0, .q_heading = 1.0, .r_steer = 0.01};
    LQRLateralParams p_costly= {.q_cte = 5.0, .q_heading = 1.0, .r_steer = 10.0};
    LQRLateral ctrl_cheap(p_cheap);
    LQRLateral ctrl_costly(p_costly);
    EXPECT_GT(ctrl_cheap.gains()[0], ctrl_costly.gains()[0]);
}

// =============================================================================
// Closed-loop convergence: controller + bicycle model
// =============================================================================

static double closed_loop_final_cte(LateralController& ctrl,
                                     double initial_y_offset,
                                     int steps = 2000,
                                     double dt = 0.01) {
    BicycleModel   model;
    ReferencePath  path = make_straight_path();
    State s{.x=0.0, .y=initial_y_offset, .theta=0.0, .v=10.0};

    for (int i=0; i<steps; ++i) {
        const double delta = ctrl.compute(s, path);
        s = model.step(s, {.delta=delta, .a=0.0}, dt);
    }
    return path.cross_track_error(s);
}

TEST(ClosedLoop, PurePursuit_CTEReduces) {
    PurePursuit ctrl;
    // After 20 s at 10 m/s, CTE from 1m offset should shrink significantly
    EXPECT_LT(std::abs(closed_loop_final_cte(ctrl, 1.0)), 0.3);
}

TEST(ClosedLoop, Stanley_CTEReduces) {
    Stanley ctrl;
    EXPECT_LT(std::abs(closed_loop_final_cte(ctrl, 1.0)), 0.1);
}

TEST(ClosedLoop, LQR_CTEReduces) {
    LQRLateral ctrl;
    EXPECT_LT(std::abs(closed_loop_final_cte(ctrl, 1.0)), 0.1);
}
