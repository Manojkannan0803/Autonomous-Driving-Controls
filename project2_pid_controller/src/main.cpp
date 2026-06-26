// =============================================================================
// project2_pid_controller / src / main.cpp
//
// Demonstrates PID longitudinal speed control of a simulated vehicle.
// Four experiments compare P-only, PI, PID, and a step-change scenario.
//
// Output files (written to the build directory):
//   p_only.csv           — P-only: shows steady-state error
//   pi_control.csv       — PI:     eliminates steady-state error
//   pid_control.csv      — PID:    faster response, smooth D-filtered
//   speed_change.csv     — PI: 60 → 120 km/h step change
//
// Use:   python ../visualize.py   (from the build directory) to plot.
// =============================================================================
#include "pid_controller.hpp"
#include "vehicle_plant.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using namespace controls;

// ── Helper: run one simulation and report performance metrics ─────────────────

struct Metrics {
    double rise_time     = -1.0;  ///< time (s) to first reach 90% of target
    double settling_time = -1.0;  ///< time (s) to stay within ±2% for 0.5 s
    double overshoot_pct =  0.0;  ///< peak overshoot as % of target
    double steady_state_error = 0.0; ///< |error| at end of simulation
};

Metrics run_experiment(const std::string& label,
                        PIDController&     pid,
                        VehiclePlant&      plant,
                        double             target_mps,
                        double             sim_time,
                        double             dt,
                        const std::string& csv_path) {
    pid.reset();
    plant.velocity = 0.0;

    std::ofstream csv(csv_path);
    csv << std::fixed << std::setprecision(4);
    csv << "time,velocity_mps,velocity_kph,error_mps,command\n";

    Metrics m;
    const double band_90pct = target_mps * 0.90;
    const double settle_band = target_mps * 0.02;
    double peak_v = 0.0;
    int steps_in_band = 0;
    const int band_hold = static_cast<int>(0.5 / dt);   // 0.5 s of consecutive steps

    for (double t = 0.0; t <= sim_time + dt * 0.5; t += dt) {
        const double error = target_mps - plant.velocity;
        const double cmd   = pid.update(error, dt);
        plant.step(cmd, dt);

        csv << t << ','
            << plant.velocity << ','
            << plant.velocity * 3.6 << ','
            << error << ','
            << cmd << '\n';

        // Rise time (first crossing of 90%)
        if (m.rise_time < 0.0 && plant.velocity >= band_90pct)
            m.rise_time = t;

        // Settling time (stays inside ±2% band for 0.5 s)
        if (std::abs(error) <= settle_band) {
            ++steps_in_band;
            if (steps_in_band >= band_hold && m.settling_time < 0.0)
                m.settling_time = t;
        } else {
            steps_in_band = 0;
        }

        if (plant.velocity > peak_v)
            peak_v = plant.velocity;
    }

    m.overshoot_pct      = (peak_v > target_mps) ? (peak_v - target_mps) / target_mps * 100.0 : 0.0;
    m.steady_state_error = std::abs(target_mps - plant.velocity);

    // ── Print report ──────────────────────────────────────────────────────────
    std::cout << '\n';
    std::cout << "  [" << label << "]\n";
    std::cout << "    Target:            " << target_mps * 3.6 << " km/h\n";
    std::cout << "    Gains:             Kp=" << pid.config().gains.Kp
              << "  Ki=" << pid.config().gains.Ki
              << "  Kd=" << pid.config().gains.Kd << '\n';
    std::cout << "    Rise time (90%):   " << (m.rise_time < 0 ? -1 : m.rise_time) << " s\n";
    std::cout << "    Settling (±2%):    " << (m.settling_time < 0 ? -1 : m.settling_time) << " s\n";
    std::cout << "    Overshoot:         " << m.overshoot_pct << " %\n";
    std::cout << "    SS error:          " << m.steady_state_error * 3.6 << " km/h\n";
    std::cout << "    Written -> " << csv_path << '\n';

    return m;
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "==============================================\n";
    std::cout << "   Project 2 — PID Speed Controller Demo     \n";
    std::cout << "==============================================\n";

    const double target  = 100.0 / 3.6;   // 100 km/h in m/s
    const double dt      = 0.01;           // 100 Hz control loop
    const double sim_dur = 40.0;           // seconds

    // ── Experiment 1: P-only ─────────────────────────────────────────────────
    // Shows the classic steady-state error caused by missing integral action.
    {
        PIDController::Config cfg;
        cfg.gains          = {.Kp = 0.3,  .Ki = 0.00, .Kd = 0.0};
        cfg.output_min     = -1.0;
        cfg.output_max     =  1.0;
        cfg.integral_min   = -5.0;
        cfg.integral_max   =  5.0;
        PIDController pid(cfg);
        VehiclePlant  plant;
        run_experiment("P-only  (Kp=0.3)", pid, plant, target, sim_dur, dt, "p_only.csv");
    }

    // ── Experiment 2: PI — eliminates steady-state error ─────────────────────
    {
        PIDController::Config cfg;
        cfg.gains          = {.Kp = 0.3,  .Ki = 0.05, .Kd = 0.0};
        cfg.output_min     = -1.0;
        cfg.output_max     =  1.0;
        cfg.integral_min   = -5.0;
        cfg.integral_max   =  5.0;
        PIDController pid(cfg);
        VehiclePlant  plant;
        run_experiment("PI      (Kp=0.3, Ki=0.05)", pid, plant, target, sim_dur, dt, "pi_control.csv");
    }

    // ── Experiment 3: Full PID with derivative filter ─────────────────────────
    {
        PIDController::Config cfg;
        cfg.gains          = {.Kp = 0.4,  .Ki = 0.06, .Kd = 0.5};
        cfg.output_min     = -1.0;
        cfg.output_max     =  1.0;
        cfg.integral_min   = -5.0;
        cfg.integral_max   =  5.0;
        cfg.derivative_tau =  0.05;    // 50 ms low-pass on D-term
        PIDController pid(cfg);
        VehiclePlant  plant;
        run_experiment("PID     (Kp=0.4, Ki=0.06, Kd=0.5, τ=50ms)", pid, plant, target, sim_dur, dt, "pid_control.csv");
    }

    // ── Experiment 4: Step change  60 → 120 km/h ─────────────────────────────
    // Demonstrates anti-windup and bumpless transfer after a setpoint change.
    {
        std::cout << "\n  [Step change  60 → 120 km/h]\n";

        PIDController::Config cfg;
        cfg.gains          = {.Kp = 0.4,  .Ki = 0.08, .Kd = 0.3};
        cfg.output_min     = -1.0;
        cfg.output_max     =  1.0;
        cfg.integral_min   = -3.0;
        cfg.integral_max   =  3.0;
        cfg.derivative_tau =  0.05;
        PIDController pid(cfg);

        VehiclePlant plant;
        plant.velocity = 60.0 / 3.6;   // start at 60 km/h

        std::ofstream csv("speed_change.csv");
        csv << std::fixed << std::setprecision(4);
        csv << "time,velocity_kph,target_kph,command\n";

        for (double t = 0.0; t <= 50.0 + dt * 0.5; t += dt) {
            const double setpoint = (t < 20.0) ? 60.0 / 3.6 : 120.0 / 3.6;
            // Reset PID state at the moment of setpoint change to avoid D-spike
            if (t >= 20.0 && t < 20.0 + dt)
                pid.reset();

            const double cmd = pid.update(setpoint - plant.velocity, dt);
            plant.step(cmd, dt);

            csv << t << ',' << plant.velocity * 3.6 << ','
                << setpoint * 3.6 << ',' << cmd << '\n';
        }
        std::cout << "    Written -> speed_change.csv\n";
    }

    // ── Bad-weather 1: P-only with aggressive gain → bang-bang oscillation ──────────
    // Kp=5.0 saturates the output for almost any error, turning the controller
    // into a bang-bang relay: full throttle below target, full brake above it.
    {
        PIDController::Config cfg;
        cfg.gains        = {.Kp = 5.0, .Ki = 0.0, .Kd = 0.0};
        cfg.output_min   = -1.0;
        cfg.output_max   =  1.0;
        cfg.integral_min = -5.0;
        cfg.integral_max =  5.0;
        PIDController pid(cfg);
        VehiclePlant  plant;
        run_experiment("P-only (Kp=5.0, bang-bang)", pid, plant, target, sim_dur, dt, "bad_kp_bang.csv");
    }

    // ── Bad-weather 2: PI without anti-windup → integral windup overshoot ────────
    // With no integral clamp, the integrator charges freely while the output is
    // saturated during acceleration.  When the car reaches 100 km/h the huge
    // accumulated integral keeps commanding full throttle → large overshoot.
    {
        PIDController::Config cfg;
        cfg.gains        = {.Kp = 0.3, .Ki = 0.05, .Kd = 0.0};
        cfg.output_min   = -1.0;
        cfg.output_max   =  1.0;
        cfg.integral_min = -1.0e6;  // effectively no clamp
        cfg.integral_max =  1.0e6;
        PIDController pid(cfg);
        VehiclePlant  plant;
        run_experiment("PI (no anti-windup)", pid, plant, target, sim_dur, dt, "bad_windup.csv");
    }

    std::cout << "\nAll experiments done.  Visualise with:  python ../visualize.py\n";
    return 0;
}
