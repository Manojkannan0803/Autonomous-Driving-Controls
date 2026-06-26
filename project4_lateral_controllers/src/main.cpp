// =============================================================================
// project4_lateral_controllers / src / main.cpp
//
// Benchmark: three controllers × two speeds, all on the same S-curve.
//
// For each combination:
//   - Run the vehicle in closed loop for the full path length
//   - Log: time, x, y, CTE, heading_error, steering_cmd to CSV
//   - Compute: RMS CTE, max |CTE|, RMS steering, max |steering|
//
// Output CSVs (written next to this script, loaded by visualize.py):
//   {controller}_{speed}kmh.csv
//   e.g. PurePursuit_36kmh.csv, Stanley_36kmh.csv, LQR_36kmh.csv
//
// Summary table is also printed to stdout.
// =============================================================================
#include "controllers.hpp"
#include "bicycle_model.hpp"
#include "reference_path.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

using namespace vehicle;
using namespace control;

// ── Metrics accumulated over one run ─────────────────────────────────────────

struct RunMetrics {
    double rms_cte      = 0.0;
    double max_cte      = 0.0;
    double rms_steer    = 0.0;
    double max_steer    = 0.0;
    int    steps        = 0;

    void update(double cte, double steer) {
        rms_cte  += cte * cte;
        rms_steer += steer * steer;
        if (std::abs(cte)   > max_cte)   max_cte   = std::abs(cte);
        if (std::abs(steer) > max_steer) max_steer = std::abs(steer);
        ++steps;
    }

    void finalise() {
        if (steps > 0) {
            rms_cte   = std::sqrt(rms_cte   / steps);
            rms_steer = std::sqrt(rms_steer / steps);
        }
    }
};

// ── Single closed-loop experiment ─────────────────────────────────────────────

RunMetrics run_experiment(LateralController&    ctrl,
                           const ReferencePath&  path,
                           double                speed_mps,
                           double                dt,
                           double                sim_time,
                           const std::string&    csv_path) {
    BicycleModel model;

    // Start at the first waypoint, heading along the path tangent
    const double start_heading = path.tangent_heading(0);
    State s{
        .x     = path[0].x,
        .y     = path[0].y,
        .theta = start_heading,
        .v     = speed_mps
    };

    std::ofstream csv(csv_path);
    csv << std::fixed << std::setprecision(6);
    csv << "time,x,y,cte,heading_error_deg,steering_deg\n";

    RunMetrics m;
    const int total_steps = static_cast<int>(sim_time / dt);

    for (int k = 0; k < total_steps; ++k) {
        const double t         = k * dt;
        const double cte       = path.cross_track_error(s);
        const double he        = path.heading_error(s);
        const double delta_cmd = ctrl.compute(s, path);

        csv << t << ','
            << s.x << ','
            << s.y << ','
            << cte << ','
            << he  * 180.0 / std::numbers::pi << ','
            << delta_cmd * 180.0 / std::numbers::pi << '\n';

        m.update(cte, delta_cmd);

        // Speed held constant — lateral-only benchmark (no longitudinal controller)
        Control u{.delta = delta_cmd, .a = 0.0};
        s = model.step(s, u, dt);
    }

    m.finalise();
    return m;
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "==============================================\n";
    std::cout << "   Project 4 — Lateral Controller Benchmark  \n";
    std::cout << "==============================================\n";

    // ── Reference path: S-curve ───────────────────────────────────────────────
    // Same geometry as Project 3 Scenario 2, so CTE comparisons are valid.
    const double path_length = 80.0;    // m
    const double amplitude   = 1.5;     // m lateral displacement
    const ReferencePath path = ReferencePath::s_curve(path_length, amplitude, 400);

    const double dt = 0.01;   // 100 Hz control loop

    // ── Controller configurations ─────────────────────────────────────────────
    struct CtrlEntry {
        std::unique_ptr<LateralController> ctrl;
        std::string label;
    };

    // ── Test speeds ───────────────────────────────────────────────────────────
    const std::vector<double> speeds_mps = {10.0 / 3.6,   // 10 km/h
                                            36.0 / 3.6,   // 36 km/h
                                            72.0 / 3.6};  // 72 km/h

    // Print header
    std::cout << '\n'
              << std::left
              << std::setw(14) << "Controller"
              << std::setw(10) << "Speed"
              << std::setw(12) << "RMS CTE"
              << std::setw(12) << "Max CTE"
              << std::setw(14) << "RMS steer"
              << std::setw(14) << "Max steer"
              << '\n';
    std::cout << std::string(76, '-') << '\n';

    for (double v : speeds_mps) {
        const double sim_time = path_length / v * 1.3;  // 30% buffer
        const int    speed_kmh = static_cast<int>(std::round(v * 3.6));

        // Build one instance of each controller per speed iteration
        // (LQR gains are speed-dependent, so we reconstruct)
        std::vector<CtrlEntry> controllers;
        controllers.push_back({
            std::make_unique<PurePursuit>(PurePursuitParams{
                .wheelbase      = 2.7,
                .lookahead_gain = 0.4,
                .min_lookahead  = 1.5,
                .max_lookahead  = 15.0
            }),
            "PurePursuit"
        });
        controllers.push_back({
            std::make_unique<Stanley>(StanleyParams{
                .k_cte  = 0.8,
                .k_soft = 0.5
            }),
            "Stanley"
        });
        controllers.push_back({
            std::make_unique<LQRLateral>(LQRLateralParams{
                .wheelbase     = 2.7,
                .nominal_speed = v,
                .dt            = dt,
                .q_cte         = 5.0,
                .q_heading     = 1.0,
                .r_steer       = 0.1,
                .riccati_iters = 500
            }),
            "LQR"
        });

        for (auto& entry : controllers) {
            const std::string csv_name =
                entry.label + "_" + std::to_string(speed_kmh) + "kmh.csv";

            RunMetrics m = run_experiment(
                *entry.ctrl, path, v, dt, sim_time, csv_name);

            std::cout << std::left
                      << std::setw(14) << entry.label
                      << std::setw(10) << (std::to_string(speed_kmh) + " km/h")
                      << std::setw(12) << std::fixed << std::setprecision(4) << m.rms_cte
                      << std::setw(12) << m.max_cte
                      << std::setw(14) << m.rms_steer * 180.0 / std::numbers::pi
                      << std::setw(14) << m.max_steer * 180.0 / std::numbers::pi
                      << '\n';
        }
        std::cout << '\n';
    }

    std::cout << "All CSVs written.  Run:  python visualize.py\n";
    return 0;
}
