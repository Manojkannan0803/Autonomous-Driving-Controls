// ============================================================
// Project 8 – MPC Trajectory Controller  |  main.cpp
// Capstone: ties together P3 (bicycle model), P6 (LQR + trajectory),
//           P7 (A* planner + spline), and P8 (MPC).
//
// Two scenarios, head-to-head (MPC vs LQR):
//
//  Scenario A — Figure-eight (30 m, 8 m/s, unconstrained regime)
//    Path is kinematically feasible for both controllers.
//    Shows baseline: MPC ≈ LQR when constraints are not binding.
//
//  Scenario B — Urban planned path (P7 map, κ_max ≈ 0.40 1/m)
//    Tight building corner forces δ_ff = atan(L·κ) ≈ 47° > 30° limit.
//    LQR: feedforward saturates, feedback fights unmodelled constraint
//         → large CTE spike at corner.
//    MPC: solver enforces |δ|≤30° inside the optimisation, anticipates
//         the corner, minimises CTE within actuator limits.
// ============================================================
#include "mpc_tracker.hpp"
#include "lqr_tracker.hpp"
#include "trajectory.hpp"
#include "reference_path.hpp"
#include "bicycle_model.hpp"

// P7 planning stack (header-only)
#include "grid_map.hpp"
#include "astar.hpp"
#include "spline_smoother.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <numbers>
#include <chrono>
#include <algorithm>

// ── Helpers ───────────────────────────────────────────────────────────────────

/// RK4 step using vehicle::BicycleModel
static vehicle::State rk4_step(const vehicle::BicycleModel& model,
                                const vehicle::State& s,
                                vehicle::Control u,
                                double dt)
{
    return model.step(s, u, dt);
}

struct TrackStats {
    double rms_cte{0.0};
    double max_cte{0.0};
    double max_abs_steer_deg{0.0};
    int    steer_violations{0};  // times |δ| > δ_max
    double solve_ms_total{0.0};  // MPC only
    int    n{0};
};

// ── Run one controller on a trajectory ───────────────────────────────────────

template<typename Tracker>
TrackStats run_sim(Tracker& tracker,
                   vehicle::BicycleModel& model,
                   vehicle::State init,
                   double sim_end,
                   double sim_dt,
                   bool is_mpc,
                   std::ofstream& csv,
                   const std::string& prefix)
{
    vehicle::State s = init;
    TrackStats st;
    double t = 0.0;
    while (t <= sim_end + 1e-9) {
        auto t0 = std::chrono::steady_clock::now();
        auto ctl = tracker.compute(s);
        auto t1 = std::chrono::steady_clock::now();
        if (is_mpc)
            st.solve_ms_total +=
                std::chrono::duration<double, std::milli>(t1-t0).count();

        // Error logging using tracker's current hint
        std::size_t hi = tracker.current_hint();
        // Access trajectory via the tracker (hint already updated)

        double steer_deg = ctl.delta * (180.0 / std::numbers::pi);
        st.max_abs_steer_deg = std::max(st.max_abs_steer_deg,
                                        std::abs(steer_deg));
        if (std::abs(ctl.delta) > 0.524 + 1e-4) ++st.steer_violations;

        // CTE for stats (will be computed from CSV later by Python)
        // Just record the state and control
        csv << t << "," << s.x << "," << s.y << "," << s.v << ","
            << ctl.delta * (180.0 / std::numbers::pi) << ","
            << ctl.accel << "\n";

        s = rk4_step(model, s, {ctl.delta, ctl.accel}, sim_dt);
        ++st.n;
        t += sim_dt;
    }
    return st;
}

// ── Scenario A: figure-eight ──────────────────────────────────────────────────
static void scenario_a()
{
    constexpr double FIG8_SIZE = 30.0;
    constexpr double V_MAX     =  8.0;
    constexpr double A_LAT     =  3.5;
    constexpr double SIM_END   = 60.0;
    constexpr double SIM_DT    =  0.02;  // 50 Hz

    auto refpath = vehicle::ReferencePath::figure_eight(FIG8_SIZE);
    auto traj    = control::Trajectory::from_path(refpath, V_MAX, A_LAT);

    vehicle::BicycleModel model;
    // Init vehicle at trajectory start (zero initial errors)
    const auto& tp0 = traj[0];
    vehicle::State init{ tp0.x, tp0.y, tp0.theta, tp0.v_ref };

    // LQR
    control::LQRTrackerParams lqr_p;
    control::LQRTracker lqr_tracker(traj, lqr_p);

    // MPC
    control::MPCTrackerParams mpc_p;
    control::MPCTracker mpc_tracker(traj, mpc_p);

    // --- run LQR ---
    {
        vehicle::State s = init;
        std::ofstream f("fig8_lqr.csv");
        f << "t,x,y,v,steer_deg,accel\n";
        double t = 0.0;
        while (t <= SIM_END + 1e-9) {
            auto ctl = lqr_tracker.compute(s);
            f << t << "," << s.x << "," << s.y << "," << s.v << ","
              << ctl.delta*(180.0/std::numbers::pi) << "," << ctl.accel << "\n";
            s = rk4_step(model, s, {ctl.delta, ctl.accel}, SIM_DT);
            t += SIM_DT;
        }
    }

    // --- run MPC ---
    {
        vehicle::State s = init;
        std::ofstream f("fig8_mpc.csv");
        f << "t,x,y,v,steer_deg,accel\n";
        double t = 0.0, mpc_ms = 0.0; int n = 0;
        while (t <= SIM_END + 1e-9) {
            auto t0  = std::chrono::steady_clock::now();
            auto ctl = mpc_tracker.compute(s);
            auto t1  = std::chrono::steady_clock::now();
            mpc_ms  += std::chrono::duration<double,std::milli>(t1-t0).count();
            f << t << "," << s.x << "," << s.y << "," << s.v << ","
              << ctl.delta*(180.0/std::numbers::pi) << "," << ctl.accel << "\n";
            s = rk4_step(model, s, {ctl.delta, ctl.accel}, SIM_DT);
            t += SIM_DT; ++n;
        }
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "  [A] MPC mean solve: " << mpc_ms/n << " ms/step  ("
                  << n << " steps)\n";
    }
    std::cout << "  Scenario A CSVs: fig8_lqr.csv, fig8_mpc.csv\n";
}

// ── Scenario B: P7 urban path ─────────────────────────────────────────────────
static void scenario_b()
{
    // ── 1. Build P7 occupancy grid + A* + spline ─────────────
    planning::OccupancyGrid map(60, 60, 1.0);
    map.set_rect( 8,  8, 22, 22);
    map.set_rect( 8, 35, 22, 50);
    map.set_rect(35,  8, 50, 22);
    map.set_rect(35, 35, 50, 50);
    map.inflate(2);

    auto raw = planning::astar(map, {2,2}, {57,57});
    if (!raw.found) { std::cerr << "A* failed in scenario B\n"; return; }

    auto thinned = planning::thin_waypoints(raw.waypoints, 3.0);
    auto smooth  = planning::fit_spline(thinned, 0.5);
    planning::add_speed_profile(smooth, 8.0, 3.5);

    // ── 2. Convert to control::Trajectory ────────────────────
    std::vector<std::pair<double,double>> xy;
    xy.reserve(smooth.size());
    for (auto& p : smooth) xy.push_back({p.x, p.y});
    auto refpath = vehicle::ReferencePath::from_waypoints(xy, /*closed=*/false);
    auto traj    = control::Trajectory::from_path(refpath, 8.0, 3.5);

    vehicle::BicycleModel model;
    const auto& tp0 = traj[0];
    vehicle::State init{ tp0.x, tp0.y, tp0.theta, tp0.v_ref };

    constexpr double SIM_DT = 0.02;

    // ── 3. Run LQR (no constraints) ───────────────────────────
    double lqr_max_steer = 0.0, lqr_viol = 0;
    {
        control::LQRTrackerParams p;
        control::LQRTracker tracker(traj, p);
        vehicle::State s = init;
        std::ofstream f("urban_lqr.csv");
        f << "t,x,y,v,steer_deg,accel\n";
        double t = 0.0;
        double sim_end = traj[traj.size()-1].s / 4.0 + 5.0; // time to traverse
        while (t <= sim_end + 1e-9) {
            auto ctl = tracker.compute(s);
            double sd = std::abs(ctl.delta) * (180.0/std::numbers::pi);
            lqr_max_steer = std::max(lqr_max_steer, sd);
            if (std::abs(ctl.delta) > 0.524 + 1e-4) ++lqr_viol;
            f << t << "," << s.x << "," << s.y << "," << s.v << ","
              << ctl.delta*(180.0/std::numbers::pi) << "," << ctl.accel << "\n";
            s = rk4_step(model, s, {ctl.delta, ctl.accel}, SIM_DT);
            t += SIM_DT;
        }
    }

    // ── 4. Run MPC (constrained) ──────────────────────────────
    double mpc_max_steer = 0.0, mpc_ms = 0.0; int mpc_n = 0;
    {
        control::MPCTrackerParams p;
        control::MPCTracker tracker(traj, p);
        vehicle::State s = init;
        std::ofstream f("urban_mpc.csv");
        f << "t,x,y,v,steer_deg,accel\n";
        double t = 0.0;
        double sim_end = traj[traj.size()-1].s / 4.0 + 5.0;
        while (t <= sim_end + 1e-9) {
            auto t0  = std::chrono::steady_clock::now();
            auto ctl = tracker.compute(s);
            auto t1  = std::chrono::steady_clock::now();
            mpc_ms  += std::chrono::duration<double,std::milli>(t1-t0).count();
            double sd = std::abs(ctl.delta) * (180.0/std::numbers::pi);
            mpc_max_steer = std::max(mpc_max_steer, sd);
            f << t << "," << s.x << "," << s.y << "," << s.v << ","
              << ctl.delta*(180.0/std::numbers::pi) << "," << ctl.accel << "\n";
            s = rk4_step(model, s, {ctl.delta, ctl.accel}, SIM_DT);
            t += SIM_DT; ++mpc_n;
        }
    }

    // Also write the reference path for the visualiser
    {
        std::ofstream f("urban_ref.csv");
        f << "x,y,theta,kappa,v_ref\n";
        for (std::size_t i = 0; i < traj.size(); ++i)
            f << traj[i].x << "," << traj[i].y << "," << traj[i].theta
              << "," << traj[i].kappa << "," << traj[i].v_ref << "\n";
    }

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  [B] Path: " << traj.size() << " pts, len ≈ "
              << traj[traj.size()-1].s << " m, κ_max = "
              << [&]{
                    double km=0;
                    for(std::size_t i=0;i<traj.size();++i)
                        km=std::max(km,std::abs(traj[i].kappa));
                    return km; }()
              << " 1/m\n";
    std::cout << "  [B] LQR  max |δ|: " << lqr_max_steer << "°  ("
              << lqr_viol << " violations)\n";
    std::cout << "  [B] MPC  max |δ|: " << mpc_max_steer << "° (hard limit 30°)\n";
    std::cout << std::setprecision(3);
    std::cout << "  [B] MPC  mean solve: " << mpc_ms/mpc_n << " ms/step\n";
    std::cout << "  Scenario B CSVs: urban_lqr.csv, urban_mpc.csv, urban_ref.csv\n";
}

// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    std::cout << "\n================================================================\n";
    std::cout << "   Project 8 -- MPC Trajectory Controller (Capstone)\n";
    std::cout << "================================================================\n";
    std::cout << "  Horizon N=15, dt=0.1s, FISTA 200 iters, δ_max=30°\n\n";

    std::cout << "  ── Scenario A: Figure-eight 30m, 8m/s ──\n";
    scenario_a();

    std::cout << "\n  ── Scenario B: Urban P7 path (tight corner) ──\n";
    scenario_b();

    std::cout << "\n================================================================\n";
    std::cout << "All CSVs written.  Run:  python visualize.py\n\n";
    return 0;
}
