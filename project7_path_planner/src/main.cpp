// ============================================================
// Project 7 – Path Planner
// Demo: urban 4-building map, A* search, cubic spline smoothing.
//
// Pipeline:
//   OccupancyGrid  ──►  A* search  ──►  thin_waypoints
//         ──►  fit_spline  ──►  add_speed_profile
//         ──►  CSV output  ──►  visualize.py
//
// Map layout (60×60 cells, 1 m/cell):
//
//   col  0          25  30          59
//        ┌───────────┬──┬───────────┐
//   r=59 │           │  │           │
//        │  [BL2]    │  │  [BR2]    │   ← building rows 35-50
//        │           │  │           │
//   r=33 ├───────────┴──┴───────────┤
//   r=24 │       corridor (9 rows)   │
//        ├───────────┬──┬───────────┤
//        │  [TL1]    │  │  [TR1]    │   ← building rows 8-22
//   r=0  │  Start(2,2)              Goal(57,57)
//
// A* threads between the buildings; spline turns the staircase
// path into a C2-continuous curve ready for an LQR/MPC controller.
// ============================================================
#include "grid_map.hpp"
#include "astar.hpp"
#include "spline_smoother.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numbers>

int main() {
    // ── 1. Build occupancy grid ──────────────────────────────────────
    //    60×60 cells, 1 m resolution → 60 m × 60 m world
    planning::OccupancyGrid map(60, 60, 1.0, 0.0, 0.0);

    // Four "building" blocks arranged to force the planner to thread
    // through the corridor between them.
    //                       r0   c0   r1   c1
    map.set_rect(            8,   8,  22,  22);   // top-left block
    map.set_rect(            8,  35,  22,  50);   // top-right block
    map.set_rect(           35,   8,  50,  22);   // bottom-left block
    map.set_rect(           35,  35,  50,  50);   // bottom-right block

    // Inflate by 2 cells ≈ 2 m safety margin around each building
    map.inflate(2);

    // ── 2. A* from corner to corner ─────────────────────────────────
    planning::Cell start{ 2,  2};
    planning::Cell goal {57, 57};

    auto t0  = std::chrono::steady_clock::now();
    auto raw = planning::astar(map, start, goal);
    auto t1  = std::chrono::steady_clock::now();
    double astar_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (!raw.found) {
        std::cerr << "A* failed — map may be fully blocked.\n";
        return 1;
    }

    // ── 3. Thin dense grid waypoints before spline fitting ──────────
    //    A* returns ~80 cell-centre waypoints (1 m apart).
    //    Thinning to ≥3 m spacing → ~25 control points — enough for a
    //    smooth spline without Runge oscillations.
    auto thinned = planning::thin_waypoints(raw.waypoints, 3.0);

    // ── 4. Fit natural cubic spline & add speed profile ─────────────
    auto t2     = std::chrono::steady_clock::now();
    auto smooth = planning::fit_spline(thinned, 0.5);
    planning::add_speed_profile(smooth, /*v_max=*/8.0, /*a_lat=*/3.5);
    auto t3     = std::chrono::steady_clock::now();
    double spline_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // ── 5. Compute stats ─────────────────────────────────────────────
    double smooth_len = smooth.empty() ? 0.0 : smooth.back().s;
    double kappa_max  = 0.0, kappa_rms_sq = 0.0;
    double v_min_act  = 1e9, v_max_act = 0.0;
    for (auto& p : smooth) {
        double ak = std::abs(p.kappa);
        kappa_max    = std::max(kappa_max, ak);
        kappa_rms_sq += p.kappa * p.kappa;
        v_min_act = std::min(v_min_act, p.v_ref);
        v_max_act = std::max(v_max_act, p.v_ref);
    }
    double kappa_rms = std::sqrt(kappa_rms_sq / static_cast<double>(smooth.size()));
    double R_min = (kappa_max > 1e-9) ? 1.0 / kappa_max : 1e9;

    std::cout << "\n================================================================\n";
    std::cout << "   Project 7 -- Path Planner Results\n";
    std::cout << "================================================================\n";
    std::cout << "  Map : 60×60 cells (60m×60m)  |  4 buildings  |  inflation=2\n";
    std::cout << "  Start (2,2)  →  Goal (57,57)\n\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  A* search\n";
    std::cout << "    Nodes expanded : " << raw.nodes_expanded << "\n";
    std::cout << "    Raw length     : " << raw.length_m     << " m  ("
              << raw.waypoints.size() << " waypoints)\n";
    std::cout << "    Search time    : " << astar_ms          << " ms\n\n";
    std::cout << "  Spline smoother (control pts after thinning: "
              << thinned.size() << ")\n";
    std::cout << "    Smooth length  : " << smooth_len         << " m  ("
              << smooth.size() << " pts @ 0.5m)\n";
    std::cout << "    Max |κ|        : " << kappa_max          << " 1/m"
              << "  (min turn R = " << R_min << " m)\n";
    std::cout << "    RMS |κ|        : " << kappa_rms          << " 1/m\n";
    std::cout << "    Spline time    : " << spline_ms          << " ms\n\n";
    std::cout << "  Speed profile (a_lat = 3.5 m/s²,  v_max = 8 m/s)\n";
    std::cout << "    v range        : " << v_min_act << " – " << v_max_act << " m/s\n";
    std::cout << "================================================================\n\n";

    // ── 6. Write CSVs ─────────────────────────────────────────────────
    // raw_path.csv — sparse A* waypoints
    {
        std::ofstream f("raw_path.csv");
        f << "x,y\n";
        for (auto& [x, y] : raw.waypoints)
            f << x << "," << y << "\n";
    }
    // smooth_path.csv — dense spline samples
    {
        std::ofstream f("smooth_path.csv");
        f << "s,x,y,kappa,v_ref\n";
        for (auto& p : smooth)
            f << p.s     << "," << p.x << "," << p.y << ","
              << p.kappa << "," << p.v_ref << "\n";
    }
    // grid.csv — obstacle map (one row per grid row, 0/1 values)
    {
        std::ofstream f("grid.csv");
        f << "# rows=" << map.rows() << " cols=" << map.cols() << "\n";
        const auto& data = map.data();
        for (int r = 0; r < map.rows(); ++r) {
            for (int c = 0; c < map.cols(); ++c) {
                if (c) f << ",";
                f << (data[static_cast<std::size_t>(r * map.cols() + c)] ? 1 : 0);
            }
            f << "\n";
        }
    }

    std::cout << "CSVs written: raw_path.csv, smooth_path.csv, grid.csv\n";
    std::cout << "Run:  python visualize.py\n\n";
    return 0;
}
