// =============================================================================
// project3_bicycle_model / src / main.cpp
//
// Three open-loop experiments to validate the bicycle model.
// No controller yet — we command fixed inputs and observe what happens.
//
// Scenario 1 — Circular motion:
//   Constant steering and speed. Vehicle should trace a circle whose radius
//   matches the analytical formula R = L / tan(δ). If it doesn't, the model
//   has a bug. This is a fundamental sanity check for any vehicle model.
//
// Scenario 2 — S-curve (open-loop):
//   Vehicle drives forward while a sinusoidal steering command is applied.
//   We log cross-track error vs. a reference S-curve. The car will deviate
//   because there's no controller, but we can see how errors grow.
//
// Scenario 3 — Figure-eight:
//   Alternating left/right turns. We log the trajectory and verify the
//   vehicle returns to origin. Heading wrapping is stress-tested here.
//
// Output CSVs (written to the working / build directory):
//   circle.csv          position + CTE for the circular path
//   s_curve.csv         position + CTE for the S-curve
//   figure_eight.csv    full state trajectory for figure-eight
//
// Visualise with:   python ../visualize.py   (from build directory)
// =============================================================================
#include "bicycle_model.hpp"
#include "reference_path.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

using namespace vehicle;

// ── CSV writer ────────────────────────────────────────────────────────────────

struct CSVWriter {
    std::ofstream f;
    explicit CSVWriter(const std::string& path, const std::string& header) {
        f.open(path);
        f << std::fixed << std::setprecision(6);
        f << header << '\n';
    }
    void row(std::initializer_list<double> vals) {
        bool first = true;
        for (double v : vals) {
            if (!first) f << ',';
            f << v;
            first = false;
        }
        f << '\n';
    }
};

// ── Scenario 1: Circular motion ───────────────────────────────────────────────
//
// Theory: at constant δ and v, vehicle traces a circle of radius R = L/tan(δ).
// We verify this by:
//   a) Computing theoretical R before the run
//   b) Measuring the actual radius from the trajectory (distance from the
//      computed circle centre, which can be inferred from the initial state)
//
void run_circle_demo() {
    std::cout << "\n[Scenario 1] Circular Motion — open loop, constant steering\n";

    BicycleModel model;
    const double L      = model.params().wheelbase;     // 2.7 m
    const double speed  = 5.0;                          // m/s (~18 km/h)
    const double R      = 10.0;                         // desired turning radius (m)
    const double delta  = std::atan(L / R);             // steering angle for R
    const double dt     = 0.01;

    // Circle of radius R centred at origin.
    // Vehicle starts at (R, 0) facing North — tangent to the circle.
    // Centre of the circular trajectory is at (0, 0).
    const State  s0 {.x = R, .y = 0.0, .theta = std::numbers::pi / 2.0, .v = speed};
    const Control u {.delta = delta, .a = 0.0};         // constant inputs

    const double period  = 2.0 * std::numbers::pi * R / speed;   // one full lap (s)
    const int    steps   = static_cast<int>(period / dt) + 1;

    std::cout << "  Wheelbase     L = " << L     << " m\n";
    std::cout << "  Target radius R = " << R     << " m\n";
    std::cout << "  Steering    δ  = " << delta  << " rad ("
              << delta * 180.0 / std::numbers::pi << "°)\n";
    std::cout << "  Lap period      = " << period << " s  (" << steps << " steps)\n";

    const ReferencePath path = ReferencePath::circle(R, 300);
    CSVWriter csv("circle.csv", "time,x,y,theta_deg,v,cte,dist_from_centre");

    State s = s0;
    double max_cte = 0.0;
    double sum_radius_sq = 0.0;

    for (int k = 0; k < steps; ++k) {
        const double t       = k * dt;
        const double cte     = path.cross_track_error(s);
        const double r_actual= std::hypot(s.x, s.y);   // dist from centre (0,0)

        csv.row({t, s.x, s.y, s.theta * 180.0 / std::numbers::pi, s.v, cte, r_actual});

        if (std::abs(cte) > max_cte) max_cte = std::abs(cte);
        sum_radius_sq += r_actual * r_actual;

        s = model.step(s, u, dt);
    }

    const double rms_radius = std::sqrt(sum_radius_sq / steps);
    std::cout << "  RMS radius from centre = " << rms_radius << " m  (expected " << R << " m)\n";
    std::cout << "  Max |CTE|              = " << max_cte << " m\n";
    std::cout << "  Wrote -> circle.csv\n";
}

// ── Scenario 2: S-curve (open-loop drift) ────────────────────────────────────
//
// A sinusoidal steering command is applied to an otherwise straight-driving car.
// There is no controller — so CTE grows because the command is not optimised
// for the reference path. This sets up the motivation for Project 4 (controllers).
//
void run_s_curve_demo() {
    std::cout << "\n[Scenario 2] S-Curve — open loop, sinusoidal steering\n";

    BicycleModel  model;
    const double  speed    = 10.0;          // m/s (~36 km/h)
    const double  dt       = 0.01;
    const double  sim_time = 8.0;           // seconds
    const int     steps    = static_cast<int>(sim_time / dt);

    // Reference path: S-curve over 80 m, ±1.5 m lateral amplitude
    const double path_length = speed * sim_time;
    const double amplitude   = 1.5;
    const ReferencePath path = ReferencePath::s_curve(path_length, amplitude, 400);

    // Start at path origin, heading East
    State   s{.x = 0.0, .y = 0.0, .theta = 0.0, .v = speed};
    Control u{.delta = 0.0, .a = 0.0};

    CSVWriter csv("s_curve.csv", "time,x,y,theta_deg,cte,heading_error_deg");

    double max_cte = 0.0;
    for (int k = 0; k < steps; ++k) {
        const double t = k * dt;

        // Sinusoidal steering: amplitude ±8°, period 4 s
        // This drives the car in an S-shape but NOT along the reference path
        // (no feedback — pure open-loop).
        u.delta = 0.14 * std::sin(2.0 * std::numbers::pi * t / 4.0);

        const double cte = path.cross_track_error(s);
        const double he  = path.heading_error(s);

        csv.row({t, s.x, s.y, s.theta * 180.0 / std::numbers::pi,
                 cte, he * 180.0 / std::numbers::pi});

        if (std::abs(cte) > max_cte) max_cte = std::abs(cte);
        s = model.step(s, u, dt);
    }

    std::cout << "  Speed          = " << speed << " m/s\n";
    std::cout << "  Path length    = " << path_length << " m\n";
    std::cout << "  Max |CTE|      = " << max_cte << " m  (open-loop, no controller)\n";
    std::cout << "  Wrote -> s_curve.csv\n";
    std::cout << "  [Note] CTE is non-zero because there's no controller yet.\n";
    std::cout << "         Project 4 will close this loop with Pure Pursuit / LQR.\n";
}

// ── Scenario 3: Figure-eight ──────────────────────────────────────────────────
//
// Two full circles of opposite sign sharing the origin as a crossing point.
//
// Geometry:
//   Right lobe: circle of radius R centred at (+R, 0).
//               Vehicle at (0,0) heading North turns RIGHT → circles back.
//   Left lobe:  circle of radius R centred at (-R, 0).
//               Vehicle at (0,0) heading North turns LEFT  → circles back.
//
// After each full revolution the vehicle returns to (0,0) heading North.
// This ensures the path is closed and heading wrap is stress-tested.
//
void run_figure_eight_demo() {
    std::cout << "\n[Scenario 3] Figure-Eight — two full circles, opposite sign\n";

    BicycleModel model;
    const double L      = model.params().wheelbase;
    const double speed  = 5.0;           // m/s
    const double R      = 8.0;           // lobe radius (m)
    const double delta  = std::atan(L / R);
    const double dt     = 0.01;

    // Full revolution: circumference / speed
    const double full_circle = 2.0 * std::numbers::pi * R / speed;

    // Sequence: right full circle → left full circle (one complete figure-8)
    struct Segment { double duration; double steering; };
    const std::vector<Segment> sequence = {
        {full_circle, -delta},   // right turn → right lobe
        {full_circle, +delta},   // left  turn → left  lobe
    };

    // Start at crossing point (origin), heading North
    State s{.x = 0.0, .y = 0.0, .theta = std::numbers::pi / 2.0, .v = speed};
    CSVWriter csv("figure_eight.csv", "time,x,y,theta_deg,v");

    double t = 0.0;
    for (const auto& seg : sequence) {
        const int     seg_steps = static_cast<int>(seg.duration / dt);
        const Control u{.delta = seg.steering, .a = 0.0};
        for (int k = 0; k < seg_steps; ++k) {
            csv.row({t, s.x, s.y, s.theta * 180.0 / std::numbers::pi, s.v});
            s = model.step(s, u, dt);
            t += dt;
        }
    }

    // After two full circles the vehicle should be back at origin heading North
    const double return_dist = std::hypot(s.x, s.y);
    const double heading_err = std::abs(wrap_angle(s.theta - std::numbers::pi / 2.0));

    std::cout << "  Lobe radius     R = " << R  << " m\n";
    std::cout << "  Steering angle  δ = " << delta * 180.0 / std::numbers::pi << "°\n";
    std::cout << "  Total time        = " << 2.0 * full_circle << " s\n";
    std::cout << "  Return dist from origin = " << return_dist << " m  (ideally ≈ 0)\n";
    std::cout << "  Heading error at end    = " << heading_err * 180.0 / std::numbers::pi
              << "°  (ideally ≈ 0)\n";
    std::cout << "  Wrote -> figure_eight.csv\n";
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "==============================================\n";
    std::cout << "   Project 3 — Bicycle Model Simulator       \n";
    std::cout << "==============================================\n";

    run_circle_demo();
    run_s_curve_demo();
    run_figure_eight_demo();

    std::cout << "\nAll done.  Visualise with:  python ../visualize.py\n";
    return 0;
}
