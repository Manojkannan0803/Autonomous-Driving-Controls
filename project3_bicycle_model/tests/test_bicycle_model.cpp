// =============================================================================
// project3_bicycle_model / tests / test_bicycle_model.cpp
//
// Tests for BicycleModel and ReferencePath.
//
// Key tests (tied to Challenge Questions):
//   - Circular motion radius matches analytical formula  R = L / tan(δ)
//   - CTE cross-product formula verified on known geometry
//   - Heading wrapping stays in (-π, π]
//   - Frozen vehicle (v=0) does not move regardless of steering
//   - Open path CTE sign convention (left = positive)
// =============================================================================
#include "bicycle_model.hpp"
#include "reference_path.hpp"

#include <cmath>
#include <numbers>
#include <gtest/gtest.h>

using namespace vehicle;

// =============================================================================
// wrap_angle
// =============================================================================

TEST(WrapAngle, AlreadyInRange)    { EXPECT_NEAR(wrap_angle(1.0), 1.0, 1e-12); }
TEST(WrapAngle, PositiveOverflow)  { EXPECT_NEAR(wrap_angle(std::numbers::pi + 0.1),
                                                  -std::numbers::pi + 0.1, 1e-10); }
TEST(WrapAngle, NegativeOverflow)  { EXPECT_NEAR(wrap_angle(-std::numbers::pi - 0.1),
                                                   std::numbers::pi - 0.1, 1e-10); }
TEST(WrapAngle, FullCircle)        { EXPECT_NEAR(wrap_angle(2.0 * std::numbers::pi), 0.0, 1e-10); }

// =============================================================================
// BicycleModel — straight line
// =============================================================================

TEST(BicycleModel, StraightLine_NoSteering) {
    BicycleModel model;
    State s{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 10.0};
    Control u{.delta = 0.0, .a = 0.0};

    for (int i = 0; i < 100; ++i)
        s = model.step(s, u, 0.01);  // 1 second at 10 m/s

    // Should travel ~10 m East with no lateral deviation
    EXPECT_NEAR(s.x, 10.0, 1.0e-4);
    EXPECT_NEAR(s.y,  0.0, 1.0e-6);
    EXPECT_NEAR(s.theta, 0.0, 1.0e-6);
    EXPECT_NEAR(s.v, 10.0, 1.0e-10);
}

// =============================================================================
// BicycleModel — frozen vehicle
// =============================================================================

TEST(BicycleModel, ZeroSpeed_PositionFrozen) {
    // With v=0, ẋ=0, ẏ=0, θ̇=0 regardless of steering. Position must not change.
    BicycleModel model;
    State s{.x = 3.0, .y = 7.0, .theta = 1.2, .v = 0.0};
    Control u{.delta = 0.4, .a = 0.0};

    for (int i = 0; i < 1000; ++i)
        s = model.step(s, u, 0.01);

    EXPECT_DOUBLE_EQ(s.x, 3.0);
    EXPECT_DOUBLE_EQ(s.y, 7.0);
    EXPECT_DOUBLE_EQ(s.v, 0.0);
}

// =============================================================================
// BicycleModel — circular motion (Challenge #2 answer is embedded here)
// =============================================================================

TEST(BicycleModel, CircularMotion_RadiusMatchesFormula) {
    // Challenge #2 answer:
    //   R = L / tan(δ)  →  δ = arctan(L / R)
    //   For L=2.7, R=10:  δ = arctan(0.27) ≈ 15.1°
    BicycleModel model;
    const double L     = model.params().wheelbase;    // 2.7 m
    const double R     = 10.0;                        // desired radius
    const double delta = std::atan(L / R);            // ≈ 0.263 rad ≈ 15.1°
    const double speed = 5.0;                         // m/s
    const double dt    = 0.001;                       // small dt for accuracy

    // Vehicle starts at (R, 0) heading North — tangent to circle centred at origin
    State s{.x = R, .y = 0.0, .theta = std::numbers::pi / 2.0, .v = speed};
    Control u{.delta = delta, .a = 0.0};

    // Simulate one full lap
    const int steps = static_cast<int>(2.0 * std::numbers::pi * R / speed / dt);
    for (int i = 0; i < steps; ++i)
        s = model.step(s, u, dt);

    // Distance from origin should equal R (we chose centre = (0,0))
    const double actual_r = std::hypot(s.x, s.y);
    EXPECT_NEAR(actual_r, R, 0.01);           // within 1 cm

    // Should have completed one full lap and returned to start
    EXPECT_NEAR(s.x, R, 0.05);
    EXPECT_NEAR(s.y, 0.0, 0.05);
}

TEST(BicycleModel, TurningRadius_Formula) {
    BicycleModel model;
    const double L   = model.params().wheelbase;
    const double R30 = model.turning_radius(30.0 * std::numbers::pi / 180.0);
    EXPECT_NEAR(R30, L / std::tan(30.0 * std::numbers::pi / 180.0), 1.0e-10);
}

// =============================================================================
// BicycleModel — actuator limits enforced
// =============================================================================

TEST(BicycleModel, SteeringClamped_NoExcessiveHeadingRate) {
    BicycleModel model;
    // Ask for 90° steering — should be clamped to max_steer (~30°)
    State s{.x = 0, .y = 0, .theta = 0, .v = 10.0};
    Control u{.delta = std::numbers::pi / 2.0, .a = 0.0};  // 90° — beyond limit

    const State k = model.derivative(s, u);
    const double max_heading_rate =
        (s.v / model.params().wheelbase) * std::tan(model.params().max_steer);

    EXPECT_LE(std::abs(k.theta), max_heading_rate + 1.0e-10);
}

TEST(BicycleModel, SpeedClamped_AtMaxSpeed) {
    BicycleModel model;
    State s{.x = 0, .y = 0, .theta = 0, .v = model.params().max_speed};
    Control u{.delta = 0.0, .a = 100.0};   // huge throttle

    for (int i = 0; i < 500; ++i)
        s = model.step(s, u, 0.01);

    EXPECT_LE(s.v, model.params().max_speed + 1.0e-9);
}

TEST(BicycleModel, SpeedClamped_AtMinSpeed) {
    BicycleModel model;
    State s{.x = 0, .y = 0, .theta = 0, .v = 1.0};
    Control u{.delta = 0.0, .a = -100.0};  // huge brake

    for (int i = 0; i < 500; ++i)
        s = model.step(s, u, 0.01);

    EXPECT_GE(s.v, model.params().min_speed - 1.0e-9);
}

// =============================================================================
// BicycleModel — heading wrap during long run
// =============================================================================

TEST(BicycleModel, HeadingStaysWrapped_LongRun) {
    // After many loops the heading must stay in (-π, π], not grow unbounded
    BicycleModel model;
    const double L     = model.params().wheelbase;
    const double R     = 5.0;
    const double delta = std::atan(L / R);
    State s{.x = R, .y = 0, .theta = std::numbers::pi / 2.0, .v = 3.0};
    Control u{.delta = delta, .a = 0.0};

    for (int i = 0; i < 50000; ++i)   // 500 s — many laps
        s = model.step(s, u, 0.01);

    EXPECT_GE(s.theta, -std::numbers::pi - 1.0e-9);
    EXPECT_LE(s.theta,  std::numbers::pi + 1.0e-9);
}

// =============================================================================
// ReferencePath — CTE (Challenge #3 answer)
// =============================================================================

TEST(ReferencePath, CTE_VehicleAbovePath_IsPositive) {
    // Challenge #3:
    //   Path from (0,0) to (10,0) — East direction
    //   Vehicle at (5, 3)
    //   CTE = (dx·ey - dy·ex) / |seg|
    //       = (10·3 - 0·5) / 10 = 30 / 10 = +3 m
    //   Positive because vehicle is LEFT (North) of eastward path.
    ReferencePath path;
    path.add({0.0, 0.0});
    path.add({10.0, 0.0});

    State s{.x = 5.0, .y = 3.0, .theta = 0.0, .v = 0.0};
    EXPECT_NEAR(path.cross_track_error(s), 3.0, 1.0e-10);
}

TEST(ReferencePath, CTE_VehicleBelowPath_IsNegative) {
    ReferencePath path;
    path.add({0.0, 0.0});
    path.add({10.0, 0.0});

    State s{.x = 5.0, .y = -2.0, .theta = 0.0, .v = 0.0};
    EXPECT_NEAR(path.cross_track_error(s), -2.0, 1.0e-10);
}

TEST(ReferencePath, CTE_VehicleOnPath_IsZero) {
    ReferencePath path;
    path.add({0.0, 0.0});
    path.add({10.0, 0.0});
    path.add({20.0, 0.0});

    State s{.x = 7.0, .y = 0.0, .theta = 0.0, .v = 0.0};
    EXPECT_NEAR(path.cross_track_error(s), 0.0, 1.0e-10);
}

// =============================================================================
// ReferencePath — heading error
// =============================================================================

TEST(ReferencePath, HeadingError_AlignedWithPath_IsZero) {
    ReferencePath path;
    path.add({0.0, 0.0});
    path.add({1.0, 0.0});   // East-going path, heading = 0

    State s{.x = 0.5, .y = 0.0, .theta = 0.0, .v = 0.0};   // also heading East
    EXPECT_NEAR(path.heading_error(s), 0.0, 1.0e-10);
}

TEST(ReferencePath, HeadingError_PerpendicularToPath) {
    ReferencePath path;
    path.add({0.0, 0.0});
    path.add({1.0, 0.0});   // East-going path

    // Vehicle heading North (π/2) on an East-going path → error = π/2
    State s{.x = 0.5, .y = 0.0, .theta = std::numbers::pi / 2.0, .v = 0.0};
    EXPECT_NEAR(path.heading_error(s), std::numbers::pi / 2.0, 1.0e-10);
}

// =============================================================================
// ReferencePath — circle factory
// =============================================================================

TEST(ReferencePath, Circle_AllPointsAtRadius) {
    const double R    = 15.0;
    const auto   path = ReferencePath::circle(R, 100);

    EXPECT_EQ(path.size(), 100u);
    for (const auto& wp : path.waypoints()) {
        const double r = std::hypot(wp.x, wp.y);
        EXPECT_NEAR(r, R, 1.0e-10);
    }
}

TEST(ReferencePath, EmptyPath_Throws) {
    ReferencePath path;
    State s{};
    EXPECT_THROW(path.cross_track_error(s), std::runtime_error);
}
