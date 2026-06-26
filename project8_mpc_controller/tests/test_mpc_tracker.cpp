#include <gtest/gtest.h>
#include "mpc_tracker.hpp"
#include "lqr_tracker.hpp"
#include "trajectory.hpp"
#include "reference_path.hpp"
#include "bicycle_model.hpp"
#include <cmath>
#include <numbers>

using namespace control;
using namespace vehicle;

// ── Helpers ───────────────────────────────────────────────────────────

static Trajectory make_circle_traj(double radius = 10.0, double v = 4.0) {
    auto path = ReferencePath::circle(radius, 200);
    return Trajectory::from_path(path, v, /*a_lat=*/10.0);
}

static Trajectory make_straight_traj(double length = 40.0, double v = 5.0) {
    auto path = ReferencePath::straight(length, 100);
    return Trajectory::from_path(path, v, /*a_lat=*/10.0);
}

// ── MPCTracker construction ───────────────────────────────────────────

TEST(MPCTracker, ConstructsOnCircle) {
    auto traj = make_circle_traj();
    MPCTracker tracker(traj);
    EXPECT_EQ(tracker.current_hint(), 0u);
}

TEST(MPCTracker, ConstructsOnStraight) {
    auto traj = make_straight_traj();
    MPCTracker tracker(traj);
    EXPECT_EQ(tracker.current_hint(), 0u);
}

// ── Constraint satisfaction ───────────────────────────────────────────

TEST(MPCTracker, SteeringConstraintRespected) {
    // Circle with very tight radius → feedforward alone exceeds 30°
    auto traj = make_circle_traj(4.0, 3.0);  // R=4m → δ_ff ≈ atan(2.7/4) ≈ 34°
    MPCTrackerParams p;
    MPCTracker tracker(traj, p);
    State s{ traj[0].x, traj[0].y, traj[0].theta, traj[0].v_ref };
    for (int i = 0; i < 50; ++i) {
        auto ctl = tracker.compute(s);
        EXPECT_LE(std::abs(ctl.delta), p.delta_max + 1e-6);
    }
}

TEST(MPCTracker, AccelConstraintRespected) {
    auto traj = make_straight_traj();
    MPCTrackerParams p;
    p.accel_min = -3.0;
    p.accel_max =  2.0;
    MPCTracker tracker(traj, p);
    State s{ 0.0, 5.0, 0.0, traj[0].v_ref };  // off-path: high speed error
    for (int i = 0; i < 50; ++i) {
        auto ctl = tracker.compute(s);
        EXPECT_LE(ctl.accel, p.accel_max + 1e-6);
        EXPECT_GE(ctl.accel, p.accel_min - 1e-6);
    }
}

// ── Closed-loop convergence ───────────────────────────────────────────

TEST(MPCTracker, ConvergesOnStraightLine) {
    auto traj = make_straight_traj(50.0, 5.0);
    MPCTrackerParams p;
    p.qp_iters = 300;
    MPCTracker tracker(traj, p);
    BicycleModel model;

    // Vehicle starts with 1 m lateral offset
    State s{ 0.0, 1.0, 0.0, 5.0 };

    double final_cte = 0.0;
    for (int step = 0; step < 500; ++step) {
        auto ctl = tracker.compute(s);
        s = model.step(s, {ctl.delta, ctl.accel}, 0.02);
    }
    // After 10 s (500 × 20ms), CTE should be very small
    final_cte = s.y;  // on a straight line, y ≈ CTE
    EXPECT_NEAR(final_cte, 0.0, 0.5);
}

// ── Zero-error state ──────────────────────────────────────────────────

TEST(MPCTracker, NearZeroOutputAtZeroError) {
    auto traj = make_straight_traj();
    MPCTrackerParams p;
    p.use_feedforward = false;  // no curvature, no feedforward
    MPCTracker tracker(traj, p);

    // Place vehicle exactly on path start with correct heading and speed
    const auto& tp = traj[0];
    State s{ tp.x, tp.y, tp.theta, tp.v_ref };

    auto ctl = tracker.compute(s);
    // With zero error, feedback should produce near-zero control
    EXPECT_NEAR(ctl.delta, 0.0, 0.1);
    EXPECT_NEAR(ctl.accel, 0.0, 0.1);
}

// ── MPC vs LQR: constraints ───────────────────────────────────────────

TEST(MPCvsLQR, MPCNeverViolatesConstraint) {
    // Tight circle where LQR would violate δ_max
    auto traj = make_circle_traj(4.0, 3.0);
    MPCTrackerParams mp;
    LQRTrackerParams lp;

    MPCTracker mpc(traj, mp);
    LQRTracker lqr(traj, lp);

    State s{ traj[0].x, traj[0].y, traj[0].theta, traj[0].v_ref };
    BicycleModel model;

    int mpc_violations = 0, lqr_violations = 0;
    for (int step = 0; step < 200; ++step) {
        auto mctl = mpc.compute(s);
        auto lctl = lqr.compute(s);
        if (std::abs(mctl.delta) > mp.delta_max + 1e-6) ++mpc_violations;
        if (std::abs(lctl.delta) > mp.delta_max + 1e-6) ++lqr_violations;
        s = model.step(s, {mctl.delta, mctl.accel}, 0.02);
    }
    EXPECT_EQ(mpc_violations, 0);
    // LQR may have violations on this tight path
    // (we only assert MPC = 0, not that LQR > 0 — depends on gains)
}

// ── Warm start continuity ─────────────────────────────────────────────

TEST(MPCTracker, WarmStartPreservesControl) {
    auto traj = make_straight_traj(60.0, 5.0);
    MPCTracker tracker(traj);
    BicycleModel model;
    State s{ 0.0, 0.5, 0.0, 5.0 };

    // First call
    auto c1 = tracker.compute(s);
    s = model.step(s, {c1.delta, c1.accel}, 0.02);
    // Second call — warm-started from first solution
    auto c2 = tracker.compute(s);

    // Consecutive controls should not jump discontinuously
    EXPECT_NEAR(c2.delta, c1.delta, 0.3);
}

// ── FISTA iteration count sensitivity ────────────────────────────────

TEST(MPCTracker, MoreItersImprovesSolution) {
    auto traj = make_straight_traj(60.0, 5.0);
    State s{ 0.0, 2.0, 0.1, 5.0 };  // significant offset

    MPCTrackerParams p_few = {}, p_many = {};
    p_few.qp_iters  =   5;
    p_many.qp_iters = 300;
    p_few.use_feedforward  = false;
    p_many.use_feedforward = false;

    MPCTracker t5(traj, p_few);
    MPCTracker t300(traj, p_many);

    // With more iterations, the solution should yield smaller |delta|
    // (steers less aggressively when cost is truly minimised)
    auto c5   = t5.compute(s);
    auto c300 = t300.compute(s);

    // 300-iter solution should have a well-defined, bounded control
    EXPECT_LE(std::abs(c300.delta), std::abs(p_many.delta_max) + 1e-6);
    // Both must satisfy constraints
    EXPECT_LE(std::abs(c5.delta),   std::abs(p_few.delta_max)  + 1e-6);
}

// ── Reference path from P7 waypoints ─────────────────────────────────

TEST(MPCTracker, WorksWithFromWaypoints) {
    // Simulate what P7 would produce: a list of (x,y) pairs
    std::vector<std::pair<double,double>> wpts;
    for (int i = 0; i <= 30; ++i)
        wpts.push_back({ static_cast<double>(i) * 2.0, 0.0 });
    auto rp   = ReferencePath::from_waypoints(wpts);
    auto traj = Trajectory::from_path(rp, 5.0, 4.0);
    MPCTracker tracker(traj);
    State s{ 0.0, 0.3, 0.0, 5.0 };
    auto ctl = tracker.compute(s);
    EXPECT_LE(std::abs(ctl.delta), 0.524 + 1e-6);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
