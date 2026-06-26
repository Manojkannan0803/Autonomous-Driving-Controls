// =============================================================================
// test_lqr_tracker.cpp  —  Unit tests for Project 6
// =============================================================================
// Tests cover:
//   1.  Trajectory arc-length is monotonically increasing
//   2.  Trajectory heading is tangent to the path
//   3.  Speed profile: v_ref ≤ v_max everywhere
//   4.  Speed profile: v_ref decreases on high-curvature segments
//   5.  Curvature sign: left-turning arc has positive κ
//   6.  CTE = 0 when vehicle is exactly on the reference
//   7.  CTE sign: vehicle left of path → positive CTE
//   8.  Gain schedule has correct size (one entry per waypoint)
//   9.  Gains are positive (CTE and heading gains must be > 0 for stability)
//   10. Feedforward: on a curve, δ_ff ≠ 0 even when errors are zero
//   11. No feedforward: δ = 0 when errors are zero and use_feedforward = false
//   12. Controller convergence on a straight path
//   13. Nearest-index-forward always returns valid index
//   14. Gain at higher speed produces larger K_heading (speed-adaptive proof)
// =============================================================================
#include "lqr_tracker.hpp"
#include "trajectory.hpp"
#include "bicycle_model.hpp"
#include "reference_path.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

using namespace control;
using namespace vehicle;

// ── Helpers ───────────────────────────────────────────────────────────────────

static Trajectory make_circle_traj(double r = 10.0, double v_max = 5.0) {
    auto path = ReferencePath::circle(r, 200);
    return Trajectory::from_path(path, v_max, 4.0);
}

static Trajectory make_straight_traj(double len = 50.0, double v_max = 5.0) {
    auto path = ReferencePath::straight(len, 100);
    return Trajectory::from_path(path, v_max, 4.0);
}

// ── Trajectory geometry ───────────────────────────────────────────────────────

TEST(Trajectory, ArcLengthMonotone) {
    auto traj = make_circle_traj();
    for (std::size_t i = 1; i < traj.size(); ++i)
        EXPECT_GE(traj[i].s, traj[i-1].s);
}

TEST(Trajectory, HeadingIsTangent) {
    // On a circle of radius 10m, each waypoint heading should match atan2(y,x)+π/2
    auto path = ReferencePath::circle(10.0, 200);
    auto traj = Trajectory::from_path(path, 5.0, 4.0);
    for (std::size_t i = 5; i < traj.size() - 5; ++i) {
        // Expected tangent: perpendicular to radial direction
        const double expected = std::atan2(traj[i].x, -traj[i].y);  // 90° CCW from radial
        // Allow ±5° tolerance (finite-difference approximation)
        const double diff = std::abs(vehicle::wrap_angle(traj[i].theta - expected));
        EXPECT_LT(diff, 0.1) << "at index " << i;
    }
}

TEST(Trajectory, SpeedBoundedByVmax) {
    const double v_max = 6.0;
    auto traj = make_circle_traj(10.0, v_max);
    for (std::size_t i = 0; i < traj.size(); ++i)
        EXPECT_LE(traj[i].v_ref, v_max + 1e-9);
}

TEST(Trajectory, SpeedReducedOnTightCurves) {
    // Circle of radius 5m should have lower v_ref than circle of radius 50m
    // at the same v_max (tighter curve → slower)
    auto tight  = Trajectory::from_path(ReferencePath::circle( 5.0, 200), 10.0, 4.0);
    auto loose  = Trajectory::from_path(ReferencePath::circle(50.0, 200), 10.0, 4.0);
    // Average v_ref on tight curve should be lower
    double sum_tight = 0.0, sum_loose = 0.0;
    for (std::size_t i = 0; i < tight.size(); ++i) sum_tight += tight[i].v_ref;
    for (std::size_t i = 0; i < loose.size(); ++i) sum_loose += loose[i].v_ref;
    EXPECT_LT(sum_tight / tight.size(), sum_loose / loose.size());
}

TEST(Trajectory, CurvatureSignPositiveForLeftTurn) {
    // Circle traversed CCW (standard parametrisation) → left turn → κ > 0
    auto traj = make_circle_traj(10.0);
    int pos_count = 0;
    for (std::size_t i = 5; i < traj.size()-5; ++i)
        if (traj[i].kappa > 0.0) ++pos_count;
    EXPECT_GT(pos_count, static_cast<int>(traj.size()) / 2);
}

TEST(Trajectory, CTEZeroOnPath) {
    auto traj = make_straight_traj();
    // Vehicle exactly at waypoint 10 with correct heading → CTE = 0
    const auto& pt = traj[10];
    State s{ pt.x, pt.y, pt.theta, pt.v_ref };
    EXPECT_NEAR(traj.cross_track_error(s, 10), 0.0, 1e-9);
}

TEST(Trajectory, CTESignLeftIsPositive) {
    auto traj = make_straight_traj(50.0);
    const auto& pt = traj[50];
    // Perturb vehicle to the left (positive Y on an East-heading path)
    State s{ pt.x, pt.y + 1.0, pt.theta, pt.v_ref };
    EXPECT_GT(traj.cross_track_error(s, 50), 0.0);
}

TEST(Trajectory, NearestIndexForwardReturnsValidIndex) {
    auto traj = make_circle_traj();
    const std::size_t idx = traj.nearest_index_forward(10.0, 0.0, 0);
    EXPECT_LT(idx, traj.size());
}

// ── LQRTracker ────────────────────────────────────────────────────────────────

TEST(LQRTracker, GainScheduleSizeMatchesTraj) {
    auto traj = make_straight_traj();
    LQRTracker tracker(traj);
    EXPECT_EQ(tracker.gain_schedule().size(), traj.size());
}

TEST(LQRTracker, GainsCTEAndHeadingPositive) {
    // For the linearised system, optimal K[0,0] and K[0,1] must be positive
    // (positive CTE → positive steering feedback to correct it)
    auto traj = make_straight_traj();
    LQRTracker tracker(traj);
    for (const auto& K : tracker.gain_schedule()) {
        EXPECT_GT(K[0], 0.0) << "K_cte should be positive";
        EXPECT_GT(K[1], 0.0) << "K_heading should be positive";
    }
}

TEST(LQRTracker, FeedforwardNonZeroOnCurve) {
    // On a curved path, a vehicle with zero errors should still get non-zero steering
    auto path = ReferencePath::circle(10.0, 200);
    auto traj  = Trajectory::from_path(path, 5.0, 4.0);
    LQRTrackerParams p; p.use_feedforward = true;
    LQRTracker tracker(traj, p);

    // Place vehicle exactly on the path with zero errors
    const auto& pt = traj[50];
    State s{ pt.x, pt.y, pt.theta, pt.v_ref };
    const auto ctrl = tracker.compute(s);
    // Should produce non-trivial steering due to feedforward
    EXPECT_GT(std::abs(ctrl.delta), 0.01);
}

TEST(LQRTracker, NoFeedforwardZeroOnPerfectState) {
    // Without feedforward and with zero errors, steering should be near zero
    auto path = ReferencePath::straight(50.0, 100);
    auto traj  = Trajectory::from_path(path, 5.0, 4.0);
    LQRTrackerParams p; p.use_feedforward = false;
    LQRTracker tracker(traj, p);

    // Perfect state on a straight path
    const auto& pt = traj[50];
    State s{ pt.x, pt.y, pt.theta, pt.v_ref };
    const auto ctrl = tracker.compute(s);
    EXPECT_NEAR(ctrl.delta, 0.0, 0.01);
    EXPECT_NEAR(ctrl.accel, 0.0, 0.01);
}

TEST(LQRTracker, ConvergesOnStraightPath) {
    // Start 0.5m to the left of a straight reference, heading aligned.
    // After 5 seconds the CTE should drop below 0.05m.
    auto path = ReferencePath::straight(100.0, 200);
    auto traj  = Trajectory::from_path(path, 5.0, 4.0);
    LQRTracker tracker(traj);
    BicycleModel car;

    State s{ 0.0, 0.5, 0.0, 4.0 };   // 0.5m left, heading east
    const double dt = 0.02;
    for (int i = 0; i < 250; ++i) {   // 5 seconds
        const auto ctrl = tracker.compute(s);
        s = car.step(s, { ctrl.delta, ctrl.accel }, dt);
    }
    const std::size_t i = traj.nearest_index_forward(s.x, s.y);
    EXPECT_LT(std::abs(traj.cross_track_error(s, i)), 0.05);
}

TEST(LQRTracker, HigherSpeedGivesLargerHeadingGain) {
    // At higher v_ref the system matrix B is larger, so the optimal K_heading
    // must also be larger (compensating for the increased heading response rate).
    // Compare gain[0,1] at v=2 m/s vs v=8 m/s.
    auto slow_path = Trajectory::from_path(
        ReferencePath::straight(50.0, 100), 2.0, 4.0);
    auto fast_path = Trajectory::from_path(
        ReferencePath::straight(50.0, 100), 8.0, 4.0);

    LQRTracker slow_t(slow_path), fast_t(fast_path);
    // K[1] = heading gain (index 1 in GainRow)
    const double k_heading_slow = slow_t.gain_schedule()[50][1];
    const double k_heading_fast = fast_t.gain_schedule()[50][1];

    // At higher speed, the heading error propagates to CTE faster (v·e_heading).
    // LQR compensates with a larger gain to damp it sooner.
    EXPECT_GT(k_heading_fast, k_heading_slow);
}
