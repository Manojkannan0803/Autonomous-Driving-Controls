// =============================================================================
// pid_controller.hpp  —  Discrete PID controller with production features
// =============================================================================
// Implements a sampled-data PID controller suitable for real-time control loops.
//
// Features:
//   - Proportional / Integral / Derivative terms
//   - Anti-windup: integral clamped to [integral_min, integral_max]
//   - Output saturation: output clamped to [output_min, output_max]
//   - Derivative low-pass filter: reduces noise amplification in the D-term
//   - Stateful reset: for mode-switch safety (e.g. manual → auto handover)
// =============================================================================
#pragma once

#include <algorithm>
#include <stdexcept>

namespace controls {

/// Discrete PID controller.
///
/// Call update(error, dt) at a fixed rate from your control loop.
/// 'dt' is the elapsed time (seconds) since the last update.
class PIDController {
public:

    // ── Configuration ─────────────────────────────────────────────────────────

    struct Gains {
        double Kp{1.0};   ///< Proportional gain
        double Ki{0.0};   ///< Integral gain
        double Kd{0.0};   ///< Derivative gain
    };

    struct Config {
        Gains  gains{};

        double output_min{-1.0e9};    ///< Actuator lower saturation limit
        double output_max{ 1.0e9};    ///< Actuator upper saturation limit

        double integral_min{-1.0e9};  ///< Anti-windup integral clamp (lower)
        double integral_max{ 1.0e9};  ///< Anti-windup integral clamp (upper)

        /// First-order low-pass time constant (seconds) for the derivative term.
        /// Set to 0 to disable filtering.  Typical values: 0.02 – 0.1 s.
        double derivative_tau{0.0};
    };

    // ── Construction ──────────────────────────────────────────────────────────

    explicit PIDController(const Config& cfg) : cfg_(cfg) {
        if (cfg.output_min >= cfg.output_max)
            throw std::invalid_argument("PIDController: output_min must be < output_max");
        if (cfg.integral_min >= cfg.integral_max)
            throw std::invalid_argument("PIDController: integral_min must be < integral_max");
    }

    // ── Control update ────────────────────────────────────────────────────────

    /// Compute the control output for the current timestep.
    ///
    /// @param error   setpoint − measurement  (positive = below setpoint)
    /// @param dt      elapsed time since last call (seconds); must be > 0
    /// @return        control command clamped to [output_min, output_max]
    double update(double error, double dt) {
        if (dt <= 0.0) return prev_output_;

        // ── Proportional ──────────────────────────────────────────────────────
        const double P = cfg_.gains.Kp * error;

        // ── Integral with anti-windup clamp ───────────────────────────────────
        integral_ = std::clamp(integral_ + error * dt,
                               cfg_.integral_min,
                               cfg_.integral_max);
        const double I = cfg_.gains.Ki * integral_;

        // ── Derivative with optional first-order low-pass filter ──────────────
        // Raw derivative:  (error[n] - error[n-1]) / dt
        const double raw_deriv = (error - prev_error_) / dt;
        if (cfg_.derivative_tau > 0.0) {
            // α = dt / (τ + dt);  approaches 1 as dt >> τ (no filtering)
            const double alpha    = dt / (cfg_.derivative_tau + dt);
            filtered_deriv_ = alpha * raw_deriv + (1.0 - alpha) * filtered_deriv_;
        } else {
            filtered_deriv_ = raw_deriv;
        }
        const double D = cfg_.gains.Kd * filtered_deriv_;

        prev_error_  = error;
        prev_output_ = std::clamp(P + I + D, cfg_.output_min, cfg_.output_max);
        return prev_output_;
    }

    // ── State management ──────────────────────────────────────────────────────

    /// Reset integrator and derivative memory to zero.
    /// Call this before switching from manual to automatic control.
    void reset() {
        integral_       = 0.0;
        prev_error_     = 0.0;
        filtered_deriv_ = 0.0;
        prev_output_    = 0.0;
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    void          set_gains(const Gains& g) { cfg_.gains = g; }
    const Config& config()    const         { return cfg_; }
    double        integral()  const         { return integral_; }

private:
    Config cfg_;
    double integral_       = 0.0;
    double prev_error_     = 0.0;
    double filtered_deriv_ = 0.0;
    double prev_output_    = 0.0;
};

} // namespace controls
