// ============================================================
// Project 9 - Mini AV Stack  |  main.cpp
// Full closed-loop AV pipeline:
//
//   P7 (A* + Spline planner)
//     L> P8 (MPC controller)
//           +> P3 (Bicycle plant - ground truth)
//           |       |
//           |       V
//           |   P5 GPS + IMU sensors
//           |       |
//           L---P5 EKF (state estimator)
//                   L> [back to MPC]
//
// Two parallel runs for comparison:
//   Oracle   - MPC receives true vehicle state (no estimation error)
//   EKF+MPC  - MPC receives EKF estimated state (realistic pipeline)
// ============================================================
#include "mpc_tracker.hpp"
#include "lqr_tracker.hpp"
#include "trajectory.hpp"
#include "reference_path.hpp"
#include "kalman_filter.hpp"
#include "sensor_models.hpp"
#include "grid_map.hpp"
#include "astar.hpp"
#include "spline_smoother.hpp"
#include "bicycle_model.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <numbers>
#include <chrono>
#include <algorithm>
#include <limits>
#include <vector>

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

static sensors::IMUMeasurement true_imu(const vehicle::State& s,
                                        const vehicle::Control& u,
                                        double wheelbase)
{
    double omega = (std::abs(s.v) > 1e-3)
                   ? s.v / wheelbase * std::tan(u.delta)
                   : 0.0;
    return { u.a, omega };
}

struct StepLog {
    double t;
    double true_x, true_y, true_v, true_theta;
    double est_x,  est_y,  est_v;
    double est_pos_err;
    double cte;
    double steer_deg, accel;
};

static void run_oracle(const control::Trajectory& traj,
                       const vehicle::State& init,
                       double sim_end, double sim_dt,
                       std::vector<StepLog>& log_out,
                       double& mpc_ms_mean)
{
    vehicle::BicycleModel model;
    vehicle::State s = init;
    control::MPCTrackerParams p;
    control::MPCTracker tracker(traj, p);
    log_out.clear();
    double t = 0.0, mpc_total = 0.0;
    int n = 0;
    const auto& goal = traj[traj.size()-1];
    while (t <= sim_end + 1e-9) {
        auto t0  = std::chrono::steady_clock::now();
        auto ctl = tracker.compute(s);
        auto t1  = std::chrono::steady_clock::now();
        mpc_total += std::chrono::duration<double,std::milli>(t1-t0).count();
        StepLog row{};
        row.t = t; row.true_x = s.x; row.true_y = s.y;
        row.true_v = s.v; row.true_theta = s.theta;
        row.est_x = s.x; row.est_y = s.y; row.est_v = s.v;
        row.est_pos_err = 0.0;
        row.cte = nearest_cte(s, traj);
        row.steer_deg = ctl.delta * (180.0 / std::numbers::pi);
        row.accel = ctl.accel;
        log_out.push_back(row);
        s = model.step(s, {ctl.delta, ctl.accel}, sim_dt);
        t += sim_dt; ++n;
        if (tracker.current_hint() + 1 >= traj.size()) break;
    }
    mpc_ms_mean = (n > 0) ? mpc_total / n : 0.0;
}

static void run_ekf_mpc(const control::Trajectory& traj,
                        const vehicle::State& init,
                        double sim_end, double sim_dt,
                        std::vector<StepLog>& log_out,
                        double& ekf_pred_ms, double& mpc_ms_mean)
{
    sensors::GPS gps(5.0, 3.0, 42);
    sensors::IMU imu_sensor(50.0, 0.3, 0.01, 7);
    estimation::EKF ekf(0.5, 0.05, 3.0);
    ekf.init(init.x, init.y, init.v, init.theta, 1.0, 0.5, 0.1);
    vehicle::BicycleModel model;
    vehicle::State true_state = init;
    vehicle::Control last_ctrl = {0.0, 0.0};
    control::MPCTrackerParams p;
    control::MPCTracker tracker(traj, p);
    log_out.clear();
    double t = 0.0, ekf_total = 0.0, mpc_total = 0.0;
    int n = 0;
    const auto& goal2 = traj[traj.size()-1];
    while (t <= sim_end + 1e-9) {
        auto imu_true = true_imu(true_state, last_ctrl, p.wheelbase);
        auto imu_meas = imu_sensor.measure(imu_true.ax, imu_true.omega);
        auto te0 = std::chrono::steady_clock::now();
        ekf.predict({imu_meas.ax, imu_meas.omega}, sim_dt);
        auto te1 = std::chrono::steady_clock::now();
        ekf_total += std::chrono::duration<double,std::milli>(te1-te0).count();
        sensors::GPSMeasurement gps_meas;
        if (gps.try_measure(t, true_state.x, true_state.y, gps_meas))
            ekf.update(gps_meas.px, gps_meas.py);
        vehicle::State est = ekf_to_vehicle(ekf.state());
        auto tm0 = std::chrono::steady_clock::now();
        auto ctl = tracker.compute(est);
        auto tm1 = std::chrono::steady_clock::now();
        mpc_total += std::chrono::duration<double,std::milli>(tm1-tm0).count();
        StepLog row{};
        row.t = t;
        row.true_x = true_state.x; row.true_y = true_state.y;
        row.true_v = true_state.v; row.true_theta = true_state.theta;
        row.est_x = est.x; row.est_y = est.y; row.est_v = est.v;
        row.est_pos_err = std::hypot(est.x - true_state.x, est.y - true_state.y);
        row.cte = nearest_cte(true_state, traj);
        row.steer_deg = ctl.delta * (180.0 / std::numbers::pi);
        row.accel = ctl.accel;
        log_out.push_back(row);
        true_state = model.step(true_state, {ctl.delta, ctl.accel}, sim_dt);
        last_ctrl = {ctl.delta, ctl.accel};
        t += sim_dt; ++n;
        if (tracker.current_hint() + 1 >= traj.size()) break;
    }
    ekf_pred_ms = (n > 0) ? ekf_total / n : 0.0;
    mpc_ms_mean = (n > 0) ? mpc_total / n : 0.0;
}

static double rms(const std::vector<StepLog>& log, double StepLog::* f) {
    double sq = 0.0;
    for (auto& r : log) sq += (r.*f) * (r.*f);
    return std::sqrt(sq / log.size());
}
static double vmax(const std::vector<StepLog>& log, double StepLog::* f) {
    double m = 0.0;
    for (auto& r : log) m = std::max(m, std::abs(r.*f));
    return m;
}

int main() {
    // P7: Build urban map + A* + spline
    auto tp0 = std::chrono::steady_clock::now();
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
    auto tp1 = std::chrono::steady_clock::now();
    double plan_ms = std::chrono::duration<double,std::milli>(tp1-tp0).count();

    // Convert to control::Trajectory
    std::vector<std::pair<double,double>> xy;
    for (auto& p : smooth) xy.push_back({p.x, p.y});
    auto refpath = vehicle::ReferencePath::from_waypoints(xy, false);
    auto traj    = control::Trajectory::from_path(refpath, 8.0, 3.5);

    const auto& ref0 = traj[0];
    vehicle::State init{ ref0.x, ref0.y, ref0.theta, ref0.v_ref };
    const double SIM_END = traj[traj.size()-1].s / 4.0 + 5.0;
    const double SIM_DT  = 0.02;

    // Run both simulations
    std::vector<StepLog> oracle_log, ekf_log;
    double oracle_mpc_ms = 0.0, ekf_pred_ms = 0.0, ekf_mpc_ms = 0.0;
    run_oracle (traj, init, SIM_END, SIM_DT, oracle_log, oracle_mpc_ms);
    run_ekf_mpc(traj, init, SIM_END, SIM_DT, ekf_log,   ekf_pred_ms, ekf_mpc_ms);

    double o_rms = rms(oracle_log, &StepLog::cte);
    double o_max = vmax(oracle_log, &StepLog::cte);
    double o_st  = vmax(oracle_log, &StepLog::steer_deg);
    int    o_viol = 0;
    for (auto& r : oracle_log) if (std::abs(r.steer_deg) > 30.05) ++o_viol;

    double e_rms = rms(ekf_log, &StepLog::cte);
    double e_max = vmax(ekf_log, &StepLog::cte);
    double e_st  = vmax(ekf_log, &StepLog::steer_deg);
    double e_pos = rms(ekf_log, &StepLog::est_pos_err);
    int    e_viol = 0;
    for (auto& r : ekf_log) if (std::abs(r.steer_deg) > 30.05) ++e_viol;

    std::cout << "\n================================================================\n";
    std::cout << "   Project 9 -- Mini AV Stack\n";
    std::cout << "================================================================\n";
    std::cout << "  Pipeline: P7 Planner -> P5 EKF -> P8 MPC -> P3 Bicycle\n";
    std::cout << "  Urban path: " << traj.size() << " pts, "
              << std::fixed << std::setprecision(1)
              << traj[traj.size()-1].s << " m,  sim " << SIM_END << " s\n\n";

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  Oracle (MPC + true state)\n";
    std::cout << "    RMS CTE      : " << o_rms << " m\n";
    std::cout << "    Max CTE      : " << o_max << " m\n";
    std::cout << "    Max |delta|  : " << o_st  << " deg  (" << o_viol << " violations)\n\n";

    std::cout << "  EKF + MPC (full pipeline)\n";
    std::cout << "    RMS CTE      : " << e_rms << " m\n";
    std::cout << "    Max CTE      : " << e_max << " m\n";
    std::cout << "    Max |delta|  : " << e_st  << " deg  (" << e_viol << " violations)\n";
    std::cout << std::setprecision(2);
    std::cout << "    EKF pos RMSE : " << e_pos << " m  (GPS sigma=3m, "
              << (1.0 - e_pos/3.0)*100.0 << "% noise reduction)\n\n";

    std::cout << std::setprecision(3);
    std::cout << "  Component latency (mean per 20ms step)\n";
    std::cout << "    Planning (A*+spline, once) : " << plan_ms     << " ms\n";
    std::cout << "    EKF predict               : " << ekf_pred_ms  << " ms\n";
    std::cout << "    MPC solve (EKF run)        : " << ekf_mpc_ms   << " ms\n";
    std::cout << "    Total per step             : "
              << ekf_pred_ms + ekf_mpc_ms << " ms  ("
              << std::setprecision(1)
              << (ekf_pred_ms + ekf_mpc_ms) / 20.0 * 100.0
              << "% of 20ms budget)\n";
    std::cout << "================================================================\n\n";

    // Write CSVs
    {
        std::ofstream f("oracle.csv");
        f << "t,x,y,v,theta,cte,steer_deg,accel\n";
        for (auto& r : oracle_log)
            f << r.t << "," << r.true_x << "," << r.true_y << ","
              << r.true_v << "," << r.true_theta << ","
              << r.cte << "," << r.steer_deg << "," << r.accel << "\n";
    }
    {
        std::ofstream f("ekf_mpc.csv");
        f << "t,true_x,true_y,true_v,est_x,est_y,est_v,est_err,cte,steer_deg,accel\n";
        for (auto& r : ekf_log)
            f << r.t << "," << r.true_x << "," << r.true_y << "," << r.true_v << ","
              << r.est_x << "," << r.est_y << "," << r.est_v << ","
              << r.est_pos_err << "," << r.cte << ","
              << r.steer_deg << "," << r.accel << "\n";
    }
    {
        std::ofstream f("ref_path.csv");
        f << "x,y,kappa,v_ref\n";
        for (std::size_t i = 0; i < traj.size(); ++i)
            f << traj[i].x << "," << traj[i].y << ","
              << traj[i].kappa << "," << traj[i].v_ref << "\n";
    }

    std::cout << "CSVs written: oracle.csv, ekf_mpc.csv, ref_path.csv\n";
    std::cout << "Run:  python visualize.py\n\n";
    return 0;
}
