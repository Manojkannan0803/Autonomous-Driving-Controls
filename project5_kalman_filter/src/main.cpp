// =============================================================================
// main.cpp  —  Project 5: Kalman Filter + EKF Sensor Fusion Demo
// =============================================================================
// Scenario: a vehicle drives a figure-eight path for 30 seconds using the
//           bicycle model from Project 3 (open-loop, known steering).
//           Two estimators run in parallel on the same synthetic sensor stream:
//
//   LinearKF  — constant-velocity model, GPS only (no IMU)
//   EKF       — CTRV model, GPS + IMU yaw-rate + acceleration
//
// Ground truth is the exact bicycle model trajectory.
// Sensors: GPS @ 5 Hz with σ=3m, IMU @ 50 Hz with σ_ω=0.01 rad/s.
//
// Output (one row per simulation step):
//   trajectory.csv   — true/KF/EKF positions over time
//   errors.csv       — per-step position error for KF and EKF
//   innovations.csv  — EKF innovation sequence (should be white noise)
//
// Metrics printed to stdout:
//   RMSE position (KF vs EKF) — main accuracy comparison
//   ANEES (Avg Normalised Estimation Error Squared) — tests if P is correct:
//     ANEES ≈ 2  → covariance is well-calibrated (2 measurement dimensions)
//     ANEES >> 2 → filter overconfident (P too small)
//     ANEES << 2 → filter underconfident (P too large / Q too big)
// =============================================================================

#include "kalman_filter.hpp"
#include "sensor_models.hpp"

// Reuse bicycle model from Project 3 as the ground-truth vehicle
#include "bicycle_model.hpp"
#include "reference_path.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

// ── Simulation parameters ─────────────────────────────────────────────────────
static constexpr double SIM_DT    =  0.02;   // 50 Hz simulation step (matches IMU)
static constexpr double SIM_END   = 45.0;    // seconds
static constexpr double FIG8_SIZE = 12.0;    // figure-eight half-size (m)
static constexpr double TARGET_V  = 5.0;     // m/s cruise speed on figure-eight

// ── Open-loop steering controller (drives the figure-eight) ──────────────────
// Pure Pursuit with a fixed lookahead on the reference path.
// We only need the vehicle to drive the path — the controller quality is not
// what we're measuring here.
static double pp_steer(const vehicle::State& s,
                       const vehicle::ReferencePath& path,
                       double wheelbase = 2.7,
                       double lookahead = 6.0) {
    const auto& wps = path.waypoints();
    std::size_t nearest = path.nearest_index(s.x, s.y);
    std::size_t target  = wps.size() - 1;
    for (std::size_t i = nearest; i < wps.size(); ++i) {
        if (std::hypot(wps[i].x - s.x, wps[i].y - s.y) >= lookahead) {
            target = i; break;
        }
    }
    const double dx     = wps[target].x - s.x;
    const double dy     = wps[target].y - s.y;
    const double lx     = -dx * std::sin(s.theta) + dy * std::cos(s.theta);
    const double sinA   = std::clamp(lx / lookahead, -1.0, 1.0);
    return std::atan2(2.0 * wheelbase * sinA, lookahead);
}

// ── RMSE helper ───────────────────────────────────────────────────────────────
static double rmse(const std::vector<double>& errs) {
    double sum = 0.0;
    for (double e : errs) sum += e * e;
    return std::sqrt(sum / static_cast<double>(errs.size()));
}

// ── CSV writer helper ─────────────────────────────────────────────────────────
struct CSV {
    std::ofstream f;
    explicit CSV(const std::string& name) : f(name) {}
    template<typename... Args>
    void row(Args... args) {
        bool first = true;
        ((f << (first ? (first=false,"") : ",") << args), ...);
        f << '\n';
    }
};

// =============================================================================
int main() {
    // ── Reference path ─────────────────────────────────────────────────────
    auto path = vehicle::ReferencePath::figure_eight(FIG8_SIZE, 600);

    // ── Bicycle model (ground truth vehicle) ──────────────────────────────
    vehicle::BicycleParams bparams;
    bparams.wheelbase = 2.7;
    bparams.max_steer = 0.6;
    bparams.max_speed = 20.0;
    vehicle::BicycleModel car(bparams);

    // Start at (0, 0) heading North (θ = π/2) — figure-eight crosses origin
    vehicle::State gt{ 0.0, 0.0, std::numbers::pi / 2.0, TARGET_V };

    // ── Sensors ───────────────────────────────────────────────────────────
    sensors::GPS gps(5.0,  3.0);      // 5 Hz, σ = 3 m
    sensors::IMU imu(50.0, 0.3, 0.01); // 50 Hz, σ_ax=0.3m/s², σ_ω=0.01 rad/s

    // ── Filters ───────────────────────────────────────────────────────────
    // LinearKF: state = [px, py, vx, vy]
    //   q_accel = 2 m/s² (model may miss rapid manoeuvres)
    //   r_gps   = 3 m    (matches sensor noise)
    estimation::LinearKF kf(SIM_DT, /*q_accel=*/2.0, /*r_gps=*/3.0);
    kf.init(gt.x, gt.y,
            gt.v * std::cos(gt.theta),   // initial vx
            gt.v * std::sin(gt.theta),   // initial vy
            5.0, 2.0);                   // initial pos/vel sigma

    // EKF: state = [px, py, v, θ]
    //   q_vel = 1 m/s,  q_yaw = 0.1 rad/s,  r_gps = 3 m
    estimation::EKF ekf(/*q_vel=*/1.0, /*q_yaw=*/0.1, /*r_gps=*/3.0);
    ekf.init(gt.x, gt.y, gt.v, gt.theta, 5.0, 2.0, 0.3);

    // ── CSV files ─────────────────────────────────────────────────────────
    CSV traj("trajectory.csv");
    traj.row("t","true_x","true_y",
             "kf_x","kf_y",
             "ekf_x","ekf_y",
             "gps_x","gps_y","gps_valid");

    CSV errs_csv("errors.csv");
    errs_csv.row("t","kf_err","ekf_err","kf_P_trace","ekf_P_trace");

    CSV innov_csv("innovations.csv");
    innov_csv.row("t","innov_x","innov_y");

    // ── Accumulators for metrics ──────────────────────────────────────────
    std::vector<double> kf_errs, ekf_errs;
    double kf_anees_sum  = 0.0;
    double ekf_anees_sum = 0.0;
    int    anees_count   = 0;

    // ── Simulation loop ───────────────────────────────────────────────────
    std::cout << "Running simulation (" << SIM_END << " s at "
              << 1.0/SIM_DT << " Hz)...\n";

    for (double t = 0.0; t <= SIM_END; t += SIM_DT) {

        // ── Ground truth: drive the figure-eight with open-loop PP ─────────
        const double steer = pp_steer(gt, path);
        const double speed_err = TARGET_V - gt.v;
        const double accel = std::clamp(2.0 * speed_err, -4.0, 4.0);

        const vehicle::Control u{ steer, accel };
        const vehicle::State gt_prev = gt;
        gt = car.step(gt, u, SIM_DT);

        // ── True derivatives (for IMU ground truth) ───────────────────────
        const double true_ax    = (gt.v - gt_prev.v) / SIM_DT;
        const double true_omega = vehicle::wrap_angle(gt.theta - gt_prev.theta) / SIM_DT;

        // ── IMU measurement (always available at simulation rate) ──────────
        const auto imu_meas = imu.measure(true_ax, true_omega);

        // ── EKF predict (every step — IMU rate) ───────────────────────────
        estimation::EKF::IMUInput ekf_u{ imu_meas.ax, imu_meas.omega };
        ekf.predict(ekf_u, SIM_DT);

        // ── KF predict (every step — no IMU input) ────────────────────────
        kf.predict(SIM_DT);

        // ── GPS measurement (5 Hz — only some steps) ──────────────────────
        sensors::GPSMeasurement gps_meas;
        const bool gps_valid = gps.try_measure(t, gt.x, gt.y, gps_meas);

        double innov_x = 0.0, innov_y = 0.0;
        if (gps_valid) {
            // Compute innovation BEFORE updating (for logging)
            auto innov = ekf.last_innovation(gps_meas.px, gps_meas.py);
            innov_x = innov(0,0);
            innov_y = innov(1,0);

            ekf.update(gps_meas.px, gps_meas.py);
            kf.update (gps_meas.px, gps_meas.py);

            innov_csv.row(t, innov_x, innov_y);

            // ANEES at GPS update steps (2 measurement dimensions)
            // ANEES_k = y_k^T · S_k^{-1} · y_k  (normalised squared innovation)
            // Expected value = 2 for a consistent filter (one per measurement dim)
            const auto& ekf_P = ekf.state().P;
            const double sp00 = ekf_P(0,0) + 3.0*3.0;
            const double sp11 = ekf_P(1,1) + 3.0*3.0;
            // Approximate ANEES ignoring off-diagonals (they're small for GPS)
            const double nees = (innov_x*innov_x)/sp00 + (innov_y*innov_y)/sp11;
            ekf_anees_sum += nees;
            ++anees_count;
        }

        // ── Extract position estimates ─────────────────────────────────────
        const double kf_px  = kf.state().x(0,0);
        const double kf_py  = kf.state().x(1,0);
        const double ekf_px = ekf.state().x(0,0);
        const double ekf_py = ekf.state().x(1,0);

        // ── Position errors ────────────────────────────────────────────────
        const double kf_err  = std::hypot(kf_px  - gt.x, kf_py  - gt.y);
        const double ekf_err = std::hypot(ekf_px - gt.x, ekf_py - gt.y);
        kf_errs.push_back(kf_err);
        ekf_errs.push_back(ekf_err);

        // P trace = sum of diagonal = total uncertainty
        double kf_trace  = 0.0;
        double ekf_trace = 0.0;
        for (int i = 0; i < 4; ++i) {
            kf_trace  += kf.state().P(i,i);
            ekf_trace += ekf.state().P(i,i);
        }

        // ── Write CSV rows ─────────────────────────────────────────────────
        traj.row(t, gt.x, gt.y, kf_px, kf_py, ekf_px, ekf_py,
                 gps_meas.px, gps_meas.py, gps_valid ? 1 : 0);
        errs_csv.row(t, kf_err, ekf_err, kf_trace, ekf_trace);
    }

    // ── Print summary ──────────────────────────────────────────────────────
    const double kf_rmse  = rmse(kf_errs);
    const double ekf_rmse = rmse(ekf_errs);
    const double ekf_anees = (anees_count > 0)
                             ? ekf_anees_sum / anees_count
                             : 0.0;

    std::printf("\n");
    std::printf("============================================================\n");
    std::printf("   Project 5 \xe2\x80\x94 Kalman Filter + EKF Sensor Fusion Results     \n");
    std::printf("============================================================\n");
    std::printf("  Duration       : %.0f s   |   Steps: %zu\n",
                SIM_END, kf_errs.size());
    std::printf("  GPS rate       : 5 Hz, σ = 3 m\n");
    std::printf("  IMU rate       : 50 Hz\n");
    std::printf("\n");
    std::printf("  %-12s  RMSE pos: %7.3f m\n", "LinearKF",  kf_rmse);
    std::printf("  %-12s  RMSE pos: %7.3f m   (EKF uses IMU)\n", "EKF", ekf_rmse);
    std::printf("\n");
    std::printf("  EKF ANEES: %.3f  (ideal ≈ 2.0 for 2-D GPS measurement)\n",
                ekf_anees);
    if (ekf_anees > 4.0)
        std::printf("  -> Filter may be OVERCONFIDENT (increase Q or R)\n");
    else if (ekf_anees < 0.5)
        std::printf("  -> Filter may be UNDERCONFIDENT (decrease Q or R)\n");
    else
        std::printf("  -> Filter consistency looks GOOD\n");
    std::printf("\n");
    std::printf("All CSVs written.  Run:  python visualize.py\n");

    return 0;
}
