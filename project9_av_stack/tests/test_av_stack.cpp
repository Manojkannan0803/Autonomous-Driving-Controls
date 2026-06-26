// ============================================================
// Project 9 – Mini AV Stack  |  tests/test_av_stack.cpp
// Integration tests for the full planning + estimation + control
// pipeline assembled in project9_av_stack.
// ============================================================
#include <gtest/gtest.h>

#include "mpc_tracker.hpp"
#include "trajectory.hpp"
#include "reference_path.hpp"
#include "kalman_filter.hpp"
#include "sensor_models.hpp"
#include "grid_map.hpp"
#include "astar.hpp"
#include "spline_smoother.hpp"
#include "bicycle_model.hpp"

#include <cmath>
#include <numbers>
#include <vector>
#include <limits>

// ── Helpers ──────────────────────────────────────────────────────────────────
static vehicle::State ekf_to_vehicle(const estimation::KFState& kf) {
    return { kf.x(0,0), kf.x(1,0), kf.x(3,0), kf.x(2,0) };
}

static double nearest_cte(const vehicle::State& s,
                           const control::Trajectory& traj)
{
    double best = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < traj.size(); ++i) {
        double dx = s.x - traj[i].x;
        double dy = s.y - traj[i].y;
        best = std::min(best, std::sqrt(dx*dx + dy*dy));
    }
    return best;
}

// Shared fixture: builds the urban A*+spline trajectory once
class AVStackTest : public ::testing::Test {
protected:
    control::Trajectory traj;
    vehicle::State      init;

    void SetUp() override {
        planning::OccupancyGrid map(60, 60, 1.0);
        map.set_rect( 8,  8, 22, 22);
        map.set_rect( 8, 35, 22, 50);
        map.set_rect(35,  8, 50, 22);
        map.set_rect(35, 35, 50, 50);
        map.inflate(2);
        auto raw     = planning::astar(map, {2,2}, {57,57});
        auto thinned = planning::thin_waypoints(raw.waypoints, 3.0);
        auto smooth  = planning::fit_spline(thinned, 0.5);
        planning::add_speed_profile(smooth, 8.0, 3.5);

        std::vector<std::pair<double,double>> xy;
        for (auto& p : smooth) xy.push_back({p.x, p.y});
        auto ref = vehicle::ReferencePath::from_waypoints(xy, false);
        traj     = control::Trajectory::from_path(ref, 8.0, 3.5);

        const auto& tp0 = traj[0];
        init = { tp0.x, tp0.y, tp0.theta, tp0.v_ref };
    }
};

// ── Test 1: Planner produces a non-trivial trajectory ────────────────────────
TEST_F(AVStackTest, TrajectoryHasEnoughPoints) {
    EXPECT_GT(traj.size(), 50u) << "Expected ≥50 trajectory points";
}

TEST_F(AVStackTest, TrajectorySpansExpectedDistance) {
    double s_end = traj[traj.size()-1].s;
    EXPECT_GT(s_end, 80.0)  << "Path should be > 80 m";
    EXPECT_LT(s_end, 150.0) << "Path should be < 150 m";
}

// ── Test 2: MPC tracker instantiates and steps without crash ─────────────────
TEST_F(AVStackTest, MPCOneStepOracle) {
    control::MPCTrackerParams p;
    control::MPCTracker tracker(traj, p);
    auto ctl = tracker.compute(init);
    EXPECT_FALSE(std::isnan(ctl.delta));
    EXPECT_FALSE(std::isnan(ctl.accel));
}

// ── Test 3: EKF initialises and one predict+update cycle ─────────────────────
TEST(EKFIntegration, PredictUpdateCycle) {
    estimation::EKF ekf(0.5, 0.05, 3.0);
    ekf.init(10.0, 20.0, 5.0, 0.785, 1.0, 0.5, 0.1);

    EXPECT_NO_THROW(ekf.predict({0.0, 0.1}, 0.02));
    EXPECT_NO_THROW(ekf.update(10.1, 20.1));

    const auto& s = ekf.state();
    EXPECT_FALSE(std::isnan(s.x(0,0)));
    EXPECT_FALSE(std::isnan(s.x(1,0)));
}

// ── Test 4: EKF reduces position uncertainty over time ───────────────────────
TEST(EKFIntegration, UncertaintyReducesAfterGPS) {
    estimation::EKF ekf(0.5, 0.05, 3.0);
    ekf.init(0.0, 0.0, 0.0, 0.0, 5.0, 1.0, 0.5);  // large initial sigma
    double init_cov = ekf.state().P(0,0);

    for (int i = 0; i < 10; ++i) {
        ekf.predict({0.0, 0.0}, 0.02);
        ekf.update(0.0, 0.0);   // perfect GPS at origin
    }
    double final_cov = ekf.state().P(0,0);
    EXPECT_LT(final_cov, init_cov) << "Covariance should decrease with GPS updates";
}

// ── Test 5: Oracle run – zero steering violations ────────────────────────────
TEST_F(AVStackTest, OracleZeroSteeringViolations) {
    const double DELTA_MAX_DEG = 30.05;
    vehicle::BicycleModel model;
    vehicle::State s = init;
    control::MPCTrackerParams p;
    control::MPCTracker tracker(traj, p);
    int violations = 0;
    for (int step = 0; step < 2000; ++step) {
        auto ctl = tracker.compute(s);
        double deg = std::abs(ctl.delta) * (180.0 / std::numbers::pi);
        if (deg > DELTA_MAX_DEG) ++violations;
        s = model.step(s, {ctl.delta, ctl.accel}, 0.02);
        if (tracker.current_hint() + 1 >= traj.size()) break;
    }
    EXPECT_EQ(violations, 0) << "MPC should never violate steering constraint";
}

// ── Test 6: Oracle RMS CTE is bounded ────────────────────────────────────────
TEST_F(AVStackTest, OracleCTEBounded) {
    vehicle::BicycleModel model;
    vehicle::State s = init;
    control::MPCTrackerParams p;
    control::MPCTracker tracker(traj, p);
    double sq = 0.0; int n = 0;
    for (int step = 0; step < 2000; ++step) {
        auto ctl = tracker.compute(s);
        double cte = nearest_cte(s, traj);
        sq += cte * cte; ++n;
        s = model.step(s, {ctl.delta, ctl.accel}, 0.02);
        if (tracker.current_hint() + 1 >= traj.size()) break;
    }
    double rms = std::sqrt(sq / n);
    EXPECT_LT(rms, 8.0) << "Oracle RMS CTE should be < 8 m on urban path";
}

// ── Test 7: EKF+MPC – zero steering violations ───────────────────────────────
TEST_F(AVStackTest, EKFMPCZeroSteeringViolations) {
    const double DELTA_MAX_DEG = 30.05;
    sensors::GPS gps(5.0, 3.0, 99);
    sensors::IMU imu_sensor(50.0, 0.3, 0.01, 13);
    estimation::EKF ekf(0.5, 0.05, 3.0);
    ekf.init(init.x, init.y, init.v, init.theta, 1.0, 0.5, 0.1);
    vehicle::BicycleModel model;
    vehicle::State true_state = init;
    vehicle::Control last_ctrl = {0.0, 0.0};
    control::MPCTrackerParams p;
    control::MPCTracker tracker(traj, p);
    int violations = 0;
    for (int step = 0; step < 2000; ++step) {
        double omega = (std::abs(true_state.v) > 1e-3)
                       ? true_state.v / p.wheelbase * std::tan(last_ctrl.delta)
                       : 0.0;
        sensors::IMUMeasurement imu_true{last_ctrl.a, omega};
        auto imu_meas = imu_sensor.measure(imu_true.ax, imu_true.omega);
        ekf.predict({imu_meas.ax, imu_meas.omega}, 0.02);
        sensors::GPSMeasurement gps_meas;
        if (gps.try_measure(step * 0.02, true_state.x, true_state.y, gps_meas))
            ekf.update(gps_meas.px, gps_meas.py);
        vehicle::State est = ekf_to_vehicle(ekf.state());
        auto ctl = tracker.compute(est);
        double deg = std::abs(ctl.delta) * (180.0 / std::numbers::pi);
        if (deg > DELTA_MAX_DEG) ++violations;
        true_state = model.step(true_state, {ctl.delta, ctl.accel}, 0.02);
        last_ctrl  = {ctl.delta, ctl.accel};
        if (tracker.current_hint() + 1 >= traj.size()) break;
    }
    EXPECT_EQ(violations, 0) << "EKF+MPC should never violate steering constraint";
}

// ── Test 8: EKF position RMSE < GPS sigma (filter improves on raw GPS) ───────
TEST_F(AVStackTest, EKFImprovesOnGPS) {
    const double GPS_SIGMA = 3.0;
    sensors::GPS gps(5.0, GPS_SIGMA, 42);
    sensors::IMU imu_sensor(50.0, 0.3, 0.01, 7);
    estimation::EKF ekf(0.5, 0.05, GPS_SIGMA);
    ekf.init(init.x, init.y, init.v, init.theta, 1.0, 0.5, 0.1);
    vehicle::BicycleModel model;
    vehicle::State true_state = init;
    vehicle::Control last_ctrl = {0.0, 0.0};
    control::MPCTrackerParams p;
    control::MPCTracker tracker(traj, p);
    double sq = 0.0; int n = 0;
    for (int step = 0; step < 2000; ++step) {
        double omega = (std::abs(true_state.v) > 1e-3)
                       ? true_state.v / p.wheelbase * std::tan(last_ctrl.delta)
                       : 0.0;
        sensors::IMUMeasurement imu_true{last_ctrl.a, omega};
        auto imu_meas = imu_sensor.measure(imu_true.ax, imu_true.omega);
        ekf.predict({imu_meas.ax, imu_meas.omega}, 0.02);
        sensors::GPSMeasurement gps_meas;
        if (gps.try_measure(step * 0.02, true_state.x, true_state.y, gps_meas))
            ekf.update(gps_meas.px, gps_meas.py);
        vehicle::State est = ekf_to_vehicle(ekf.state());
        double err = std::hypot(est.x - true_state.x, est.y - true_state.y);
        sq += err * err; ++n;
        auto ctl = tracker.compute(est);
        true_state = model.step(true_state, {ctl.delta, ctl.accel}, 0.02);
        last_ctrl  = {ctl.delta, ctl.accel};
        if (tracker.current_hint() + 1 >= traj.size()) break;
    }
    double rmse = std::sqrt(sq / n);
    EXPECT_LT(rmse, GPS_SIGMA) << "EKF RMSE should be < GPS sigma (filter adds value)";
}

// ── Test 9: Sensor models behave correctly ───────────────────────────────────
TEST(SensorModels, GPSMeasuresAtCorrectRate) {
    sensors::GPS gps(5.0, 1.0, 1);  // 5 Hz
    int count = 0;
    sensors::GPSMeasurement m;
    for (int i = 0; i < 100; ++i) {
        if (gps.try_measure(i * 0.02, 0.0, 0.0, m)) ++count;
    }
    // 100 steps * 0.02 s = 2 s; at 5 Hz expect ≈10 measurements
    EXPECT_NEAR(count, 10, 2) << "GPS should fire at ~5 Hz over 2 seconds";
}

TEST(SensorModels, IMUAddsNoise) {
    sensors::IMU imu(50.0, 0.1, 0.01, 42);
    auto m = imu.measure(1.0, 0.5);
    // Noise is zero-mean Gaussian; measurement should be close to true value
    EXPECT_NEAR(m.ax,    1.0, 1.0);   // within 10-sigma band
    EXPECT_NEAR(m.omega, 0.5, 0.2);
}

// ── Test 10: Pipeline initialises at trajectory start (CTE=0) ────────────────
TEST_F(AVStackTest, InitialCTEIsZero) {
    double cte = nearest_cte(init, traj);
    EXPECT_LT(cte, 1e-3) << "Vehicle should start exactly on the trajectory";
}
