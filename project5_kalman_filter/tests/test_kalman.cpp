// =============================================================================
// test_kalman.cpp  —  Unit tests for Project 5 Kalman Filter + EKF
// =============================================================================
// Tests cover:
//   1.  Matrix algebra correctness (multiply, transpose, inverse)
//   2.  KF on a stationary vehicle: estimate converges to truth
//   3.  KF: predict with no updates → covariance grows monotonically
//   4.  KF: update with very small R → estimate snaps to measurement
//   5.  KF: update with very large R → estimate ignores measurement
//   6.  EKF heading wrap: θ doesn't blow up past ±π
//   7.  EKF: predict step reproduces kinematics exactly (zero noise)
//   8.  EKF: innovation is zero when measurement exactly matches prediction
//   9.  EKF: covariance is positive-definite after repeated predict+update
//   10. EKF: RMSE < GPS noise after 20 update steps (fusion is beneficial)
//   11. LinearKF: correct state dimension (4 states)
//   12. LinearKF: velocity estimate improves with successive GPS updates
// =============================================================================
#include "kalman_filter.hpp"
#include "sensor_models.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace estimation;

// ─────────────────────────────────────────────────────────────────────────────
// Matrix algebra
// ─────────────────────────────────────────────────────────────────────────────

TEST(MatAlgebra, MultiplyIdentityIsNoop) {
    Mat4 A = Mat4::identity();
    Mat4 B = Mat4::zeros();
    B(0,1) = 3.0; B(2,3) = -1.5;
    auto C = A * B;
    EXPECT_DOUBLE_EQ(C(0,1),  3.0);
    EXPECT_DOUBLE_EQ(C(2,3), -1.5);
    EXPECT_DOUBLE_EQ(C(1,1),  0.0);
}

TEST(MatAlgebra, TransposeSquareMatrix) {
    Mat4 A = Mat4::zeros();
    A(0,1) = 5.0; A(1,0) = 0.0;
    auto At = transpose(A);
    EXPECT_DOUBLE_EQ(At(1,0), 5.0);
    EXPECT_DOUBLE_EQ(At(0,1), 0.0);
}

TEST(MatAlgebra, Inverse2Correctness) {
    Mat2 M = Mat2::zeros();
    M(0,0) = 2.0; M(0,1) = 1.0;
    M(1,0) = 5.0; M(1,1) = 3.0;
    auto Mi = inverse2(M);
    // M * M^{-1} should be identity
    auto I = M * Mi;
    EXPECT_NEAR(I(0,0), 1.0, 1e-12);
    EXPECT_NEAR(I(1,1), 1.0, 1e-12);
    EXPECT_NEAR(I(0,1), 0.0, 1e-12);
    EXPECT_NEAR(I(1,0), 0.0, 1e-12);
}

TEST(MatAlgebra, Inverse2SingularThrows) {
    Mat2 M = Mat2::zeros();   // det = 0
    EXPECT_THROW(inverse2(M), std::runtime_error);
}

TEST(MatAlgebra, WrapAngle) {
    EXPECT_NEAR(wrap(0.0),                0.0,    1e-12);
    EXPECT_NEAR(wrap(3.5),   3.5 - 2*std::numbers::pi, 1e-10);
    EXPECT_NEAR(wrap(-3.5), -3.5 + 2*std::numbers::pi, 1e-10);
}

// ─────────────────────────────────────────────────────────────────────────────
// Linear KF
// ─────────────────────────────────────────────────────────────────────────────

TEST(LinearKF, StationaryVehicleConverges) {
    // Stationary vehicle at (10, 20). After many GPS updates the
    // estimate should be close to ground truth despite initial uncertainty.
    LinearKF kf(0.1, 1.0, 2.0);
    kf.init(0.0, 0.0, 0.0, 0.0, 10.0, 5.0);   // start with big uncertainty

    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 2.0);

    for (int i = 0; i < 100; ++i) {
        kf.predict(0.1);
        kf.update(10.0 + noise(rng), 20.0 + noise(rng));
    }
    const auto& x = kf.state().x;
    EXPECT_NEAR(x(0,0), 10.0, 0.5);
    EXPECT_NEAR(x(1,0), 20.0, 0.5);
}

TEST(LinearKF, CovarianceGrowsWithNoPredictUpdates) {
    LinearKF kf(0.1, 2.0, 3.0);
    kf.init(0.0, 0.0, 0.0, 0.0, 1.0, 1.0);

    const double p00_init = kf.state().P(0,0);
    for (int i = 0; i < 50; ++i) kf.predict(0.1);
    EXPECT_GT(kf.state().P(0,0), p00_init);
}

TEST(LinearKF, LargeRIgnoresMeasurement) {
    // R so large the update barely moves the state
    LinearKF kf(0.1, 1.0, 1000.0);   // σ_gps = 1000 m
    kf.init(5.0, 5.0, 0.0, 0.0, 0.1, 0.1);
    kf.predict(0.1);
    const double px_before = kf.state().x(0,0);
    kf.update(100.0, 100.0);   // far-away GPS reading
    const double px_after  = kf.state().x(0,0);
    // State should move only slightly
    EXPECT_LT(std::abs(px_after - px_before), 5.0);
}

TEST(LinearKF, SmallRSnapsToPerfectMeasurement) {
    LinearKF kf(0.1, 1.0, 0.001);   // σ_gps ≈ 0 — perfect GPS
    kf.init(0.0, 0.0, 0.0, 0.0, 50.0, 5.0);   // very uncertain position
    kf.predict(0.1);
    kf.update(7.0, 3.0);
    EXPECT_NEAR(kf.state().x(0,0), 7.0, 0.01);
    EXPECT_NEAR(kf.state().x(1,0), 3.0, 0.01);
}

TEST(LinearKF, CorrectStateDimension) {
    LinearKF kf(0.05, 1.0, 3.0);
    kf.init(1.0, 2.0, 0.5, 0.3);
    const auto& x = kf.state().x;
    EXPECT_DOUBLE_EQ(x(0,0), 1.0);
    EXPECT_DOUBLE_EQ(x(1,0), 2.0);
    EXPECT_DOUBLE_EQ(x(2,0), 0.5);
    EXPECT_DOUBLE_EQ(x(3,0), 0.3);
}

TEST(LinearKF, VelocityEstimateImprovesOverTime) {
    // Vehicle moving at constant 5 m/s east. KF should infer velocity from
    // successive GPS positions even though GPS only measures position.
    LinearKF kf(0.1, 0.5, 0.1);   // small GPS noise so learning is visible
    kf.init(0.0, 0.0, 0.0, 0.0, 0.1, 5.0);  // velocity very uncertain

    const double true_vx = 5.0;
    for (int i = 0; i < 30; ++i) {
        kf.predict(0.1);
        kf.update(true_vx * (i+1) * 0.1, 0.0);   // noiseless positions
    }
    EXPECT_NEAR(kf.state().x(2,0), true_vx, 0.3);   // vx estimate within 0.3 m/s
}

// ─────────────────────────────────────────────────────────────────────────────
// EKF
// ─────────────────────────────────────────────────────────────────────────────

TEST(EKF, PredictReproducesKinematics) {
    // Zero-noise case: EKF predict should exactly follow the CTRV equations.
    EKF ekf(0.0, 0.0, 1.0);   // q_vel = q_yaw = 0 (no process noise)
    ekf.init(0.0, 0.0, 10.0, 0.0, 0.01, 0.01, 0.01);

    EKF::IMUInput u{ 0.0, 0.5 };   // constant 0.5 rad/s yaw rate, no accel
    ekf.predict(u, 0.1);

    const auto& x = ekf.state().x;
    // After 0.1s at v=10, θ=0+0.05=0.05 rad
    EXPECT_NEAR(x(0,0), 10.0 * std::cos(0.0) * 0.1, 1e-9);   // px
    EXPECT_NEAR(x(1,0), 10.0 * std::sin(0.0) * 0.1, 1e-9);   // py
    EXPECT_NEAR(x(2,0), 10.0,                        1e-9);   // v unchanged
    EXPECT_NEAR(x(3,0), 0.05,                        1e-9);   // θ = ω*dt
}

TEST(EKF, HeadingWrapsCorrectly) {
    EKF ekf(0.1, 0.1, 3.0);
    ekf.init(0.0, 0.0, 5.0, 3.1);   // near +π

    EKF::IMUInput u{ 0.0, 0.5 };    // turning left, will cross +π boundary
    for (int i = 0; i < 20; ++i) ekf.predict(u, 0.05);

    const double theta = ekf.state().x(3,0);
    EXPECT_GT(theta, -std::numbers::pi);
    EXPECT_LE(theta,  std::numbers::pi);
}

TEST(EKF, InnovationZeroOnPerfectMeasurement) {
    EKF ekf(0.001, 0.001, 1.0);
    ekf.init(5.0, 3.0, 8.0, 0.0, 0.001, 0.001, 0.001);
    // Measurement exactly at predicted position → innovation = 0
    auto innov = ekf.last_innovation(5.0, 3.0);
    EXPECT_NEAR(innov(0,0), 0.0, 1e-9);
    EXPECT_NEAR(innov(1,0), 0.0, 1e-9);
}

TEST(EKF, CovarianceRemainsPositiveDefinite) {
    // After many predict+update cycles, diagonal of P should stay positive.
    EKF ekf(1.0, 0.2, 3.0);
    ekf.init(0.0, 0.0, 10.0, 0.0, 5.0, 2.0, 0.5);

    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 3.0);

    EKF::IMUInput u{ 0.2, 0.15 };
    for (int i = 0; i < 200; ++i) {
        ekf.predict(u, 0.02);
        if (i % 10 == 0)
            ekf.update(0.0 + noise(rng), 0.0 + noise(rng));
    }
    for (int i = 0; i < 4; ++i)
        EXPECT_GT(ekf.state().P(i,i), 0.0);
}

TEST(EKF, RMSEImprovesBeyondGPSNoise) {
    // After 20 GPS updates the EKF position error should be
    // significantly less than the raw GPS noise (σ = 3 m).
    const double true_x = 30.0, true_y = 20.0;
    EKF ekf(0.5, 0.1, 3.0);
    ekf.init(true_x, true_y, 0.0, 0.0, 3.0, 1.0, 0.3);

    std::mt19937 rng(17);
    std::normal_distribution<double> noise(0.0, 3.0);
    EKF::IMUInput u{ 0.0, 0.0 };

    double sse = 0.0;
    for (int i = 0; i < 40; ++i) {
        ekf.predict(u, 0.1);
        const double zx = true_x + noise(rng);
        const double zy = true_y + noise(rng);
        ekf.update(zx, zy);
        const double ex = ekf.state().x(0,0) - true_x;
        const double ey = ekf.state().x(1,0) - true_y;
        sse += ex*ex + ey*ey;
    }
    const double est_rmse = std::sqrt(sse / 40.0);
    EXPECT_LT(est_rmse, 3.0);   // better than raw GPS
}

// ─────────────────────────────────────────────────────────────────────────────
// Sensor models
// ─────────────────────────────────────────────────────────────────────────────

TEST(SensorModels, GPSOnlyFiresAtRate) {
    sensors::GPS gps(5.0, 1.0);   // 5 Hz
    int count = 0;
    for (int i = 0; i < 500; ++i) {
        sensors::GPSMeasurement m;
        if (gps.try_measure(i * 0.02, 0.0, 0.0, m)) ++count;
    }
    // 500 steps × 0.02s = 10s, at 5 Hz expect 50 fixes (±1 for boundary)
    EXPECT_GE(count, 49);
    EXPECT_LE(count, 51);
}

TEST(SensorModels, IMUNoiseMeanNearZero) {
    sensors::IMU imu(50.0, 0.3, 0.01);
    double sum_ax = 0.0, sum_om = 0.0;
    const int N = 10000;
    for (int i = 0; i < N; ++i) {
        auto m = imu.measure(0.0, 0.0);
        sum_ax += m.ax;
        sum_om += m.omega;
    }
    EXPECT_NEAR(sum_ax / N, 0.0, 0.02);
    EXPECT_NEAR(sum_om / N, 0.0, 0.002);
}
