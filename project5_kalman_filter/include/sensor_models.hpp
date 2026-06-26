// =============================================================================
// sensor_models.hpp  —  Simulated GPS and IMU for Project 5
// =============================================================================
// These classes produce realistic noisy measurements from a known true state.
// In a real AV the sensor driver hardware would provide these instead;
// here we simulate them so we can measure filter accuracy against ground truth.
//
// GPS  — 5 Hz position fix, σ ≈ 3 m (typical urban single-band GPS)
// IMU  — 50 Hz yaw-rate + longitudinal acceleration, low noise but slow drift
//
// The random engine is seeded deterministically so simulation is reproducible.
// =============================================================================
#pragma once

#include <cmath>
#include <random>

namespace sensors {

// ── GPS sensor ────────────────────────────────────────────────────────────────
// Produces a position measurement at a fixed rate with additive white noise.
// Real GPS also has biases, multipath, and intermittent outages — omitted here.

struct GPSMeasurement {
    double px = 0.0;    // noisy X position (m)
    double py = 0.0;    // noisy Y position (m)
};

class GPS {
public:
    /// @param rate_hz    Update frequency (Hz) — 5 Hz is typical
    /// @param sigma_pos  1-σ noise on each axis (m) — ~3 m for urban GPS
    /// @param seed       RNG seed for reproducibility
    explicit GPS(double rate_hz = 5.0, double sigma_pos = 3.0,
                 unsigned seed = 42)
        : dt_(1.0 / rate_hz)
        , dist_(0.0, sigma_pos)
        , rng_(seed)
    {}

    /// Returns true and fills `out` if a new GPS fix is ready at time `t`.
    /// Call every simulation timestep; only produces a measurement at 1/rate_hz intervals.
    bool try_measure(double t, double true_px, double true_py,
                     GPSMeasurement& out) {
        if (t >= next_time_ - 1.0e-9) {
            out.px = true_px + dist_(rng_);
            out.py = true_py + dist_(rng_);
            next_time_ += dt_;
            return true;
        }
        return false;
    }

    double dt() const { return dt_; }

private:
    double dt_;
    double next_time_ = 0.0;
    std::normal_distribution<double> dist_;
    std::mt19937 rng_;
};

// ── IMU sensor ────────────────────────────────────────────────────────────────
// Produces yaw-rate and longitudinal acceleration at a high rate.
// Real IMUs have bias (constant offset that drifts slowly) — modelled here
// as a fixed bias + white noise. The EKF does NOT estimate bias (that would
// require an augmented state vector — see Project 6).

struct IMUMeasurement {
    double ax    = 0.0;   // longitudinal acceleration (m/s²)
    double omega = 0.0;   // yaw rate (rad/s)
};

class IMU {
public:
    /// @param rate_hz     Update frequency (Hz) — 50 Hz typical MEMS IMU
    /// @param sigma_ax    1-σ noise on acceleration (m/s²)
    /// @param sigma_omega 1-σ noise on yaw rate (rad/s)
    /// @param seed        RNG seed
    explicit IMU(double rate_hz = 50.0,
                 double sigma_ax = 0.3, double sigma_omega = 0.01,
                 unsigned seed = 99)
        : dt_(1.0 / rate_hz)
        , dist_ax_   (0.0, sigma_ax)
        , dist_omega_(0.0, sigma_omega)
        , rng_(seed)
    {}

    /// Always produces a measurement (IMU runs faster than simulation for AVs).
    IMUMeasurement measure(double true_ax, double true_omega) {
        return { true_ax    + dist_ax_   (rng_),
                 true_omega + dist_omega_(rng_) };
    }

    double dt() const { return dt_; }

private:
    double dt_;
    std::normal_distribution<double> dist_ax_;
    std::normal_distribution<double> dist_omega_;
    std::mt19937 rng_;
};

} // namespace sensors
