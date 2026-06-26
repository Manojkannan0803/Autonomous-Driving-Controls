// =============================================================================
// main.cpp  --  Project 6: LQR Trajectory Tracker Demo
// =============================================================================
// Runs two controllers side-by-side on a figure-eight path (60 s / ~3 laps):
//
//   P6 -- LQRTracker: time-varying gains + curvature feedforward + speed profile
//   P4 -- LQRTracker: flat-speed trajectory + no feedforward (replicates P4 style)
//
// Both vehicles are initialised EXACTLY at traj[0] (zero initial errors).
// All performance differences come from controller design, not startup transients.
// =============================================================================

#include "lqr_tracker.hpp"
#include "trajectory.hpp"
#include "bicycle_model.hpp"
#include "reference_path.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numbers>
#include <string>
#include <vector>

static constexpr double SIM_DT    = 0.02;
static constexpr double SIM_END   = 60.0;
static constexpr double FIG8_SIZE = 30.0;
static constexpr double V_MAX     = 8.0;
static constexpr double A_LAT     = 3.5;

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

static double rmse(const std::vector<double>& v) {
    double s = 0.0;
    for (double x : v) s += x*x;
    return std::sqrt(s / static_cast<double>(v.size()));
}

int main() {
    auto ref_path = vehicle::ReferencePath::figure_eight(FIG8_SIZE, 500);

    auto traj_p6 = control::Trajectory::from_path(ref_path, V_MAX, A_LAT);
    auto traj_p4 = control::Trajectory::from_path(ref_path, V_MAX, 1.0e6);

    control::LQRTrackerParams p6_params;
    p6_params.use_feedforward = true;

    control::LQRTrackerParams p4_params = p6_params;
    p4_params.use_feedforward = false;

    control::LQRTracker tracker_p6(traj_p6, p6_params);
    control::LQRTracker tracker_p4(traj_p4, p4_params);

    vehicle::BicycleModel car;

    // Init BOTH vehicles at traj[0] -- zero initial heading/speed errors
    const auto& pt0 = traj_p6[0];
    vehicle::State s_p6 { pt0.x, pt0.y, pt0.theta, pt0.v_ref };
    vehicle::State s_p4 { pt0.x, pt0.y, pt0.theta, traj_p4[0].v_ref };

    // Gain schedule CSV (written once)
    {
        CSV g("gains.csv");
        g.row("idx","s","kappa","v_ref","K00","K01","K10","K12");
        for (std::size_t i = 0; i < traj_p6.size(); ++i) {
            const auto& pt = traj_p6[i];
            const auto& K  = tracker_p6.gain_schedule()[i];
            g.row(i, pt.s, pt.kappa, pt.v_ref, K[0], K[1], K[3], K[5]);
        }
    }

    CSV track("tracking.csv");
    track.row("t",
              "p6_x","p6_y","p6_v","p6_cte","p6_he","p6_ve",
              "p6_steer_deg","p6_accel","p6_v_ref",
              "p4_x","p4_y","p4_v","p4_cte","p4_he","p4_steer_deg");

    std::vector<double> cte_p6, cte_p4, ve_p6;
    double max_p6 = 0.0, max_p4 = 0.0;

    std::printf("Running %.0f s at %.0f Hz on figure-eight (%.0f m)...\n",
                SIM_END, 1.0/SIM_DT, FIG8_SIZE);

    for (double t = 0.0; t <= SIM_END; t += SIM_DT) {

        const auto  u6  = tracker_p6.compute(s_p6);
        const double st6 = std::clamp(u6.delta, -0.524, 0.524);
        const double ac6 = std::clamp(u6.accel, -8.0, 3.0);
        s_p6 = car.step(s_p6, {st6, ac6}, SIM_DT);

        const auto  u4  = tracker_p4.compute(s_p4);
        const double st4 = std::clamp(u4.delta, -0.524, 0.524);
        const double ac4 = std::clamp(u4.accel, -8.0, 3.0);
        s_p4 = car.step(s_p4, {st4, ac4}, SIM_DT);

        // Use tracker hint for accurate error (avoids snap-back on closed loops)
        const std::size_t i6 = tracker_p6.current_hint();
        const std::size_t i4 = tracker_p4.current_hint();

        const double cte6   = traj_p6.cross_track_error(s_p6, i6);
        const double he6    = vehicle::wrap_angle(s_p6.theta - traj_p6[i6].theta);
        const double ve6    = s_p6.v - traj_p6[i6].v_ref;
        const double cte4   = traj_p4.cross_track_error(s_p4, i4);
        const double he4    = vehicle::wrap_angle(s_p4.theta - traj_p4[i4].theta);
        const double v_ref6 = traj_p6[i6].v_ref;

        cte_p6.push_back(std::abs(cte6));
        cte_p4.push_back(std::abs(cte4));
        ve_p6.push_back(std::abs(ve6));
        max_p6 = std::max(max_p6, std::abs(cte6));
        max_p4 = std::max(max_p4, std::abs(cte4));

        track.row(t,
                  s_p6.x, s_p6.y, s_p6.v, cte6, he6, ve6,
                  st6*(180.0/std::numbers::pi), ac6, v_ref6,
                  s_p4.x, s_p4.y, s_p4.v, cte4, he4,
                  st4*(180.0/std::numbers::pi));
    }

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf("   Project 6 -- LQR Trajectory Tracker Results\n");
    std::printf("================================================================\n");
    std::printf("  Figure-eight %.0fm, v_max=%.0fm/s, a_lat=%.1fm/s^2\n",
                FIG8_SIZE, V_MAX, A_LAT);
    std::printf("\n");
    std::printf("  %-30s  RMS CTE: %7.4f m   Max: %7.4f m\n",
                "P6 (feedforward + adaptive v)", rmse(cte_p6), max_p6);
    std::printf("  %-30s  RMS CTE: %7.4f m   Max: %7.4f m\n",
                "P4-style (no feedforward)",     rmse(cte_p4), max_p4);
    std::printf("\n");
    std::printf("  P6 RMS speed error: %.4f m/s\n", rmse(ve_p6));
    std::printf("\n");
    std::printf("All CSVs written.  Run:  python visualize.py\n");

    return 0;
}
