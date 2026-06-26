// =============================================================================
// trajectory.hpp  —  Reference trajectory with curvature + speed profile
// =============================================================================
// A Trajectory is a ReferencePath augmented with:
//   θ    — tangent heading at each waypoint (finite-difference estimate)
//   κ    — signed curvature (1/m):  left turn = positive
//   v_ref— reference speed derived from a lateral-acceleration budget
//   s    — cumulative arc-length from the start
//
// Speed profile: v_ref = min(v_max, sqrt(a_lat_max / max(|κ|, ε)))
//   This enforces  a_lat = v² · |κ|  ≤  a_lat_max.
//   On straights (κ → 0) the car may travel at v_max.
//   On tight curves the car slows to keep lateral acceleration safe.
//   Three passes of 3-point averaging smooth out curvature noise.
//
// Nearest-index lookup uses a forward-biased window search so the index
// never snaps back to a previously visited segment on closed loops.
// =============================================================================
#pragma once

#include "bicycle_model.hpp"
#include "reference_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace control {

// ── Single reference point on the trajectory ─────────────────────────────────
struct TrajectoryPoint {
    double x     = 0.0;   // world-frame position (m)
    double y     = 0.0;
    double theta = 0.0;   // tangent heading (rad)
    double kappa = 0.0;   // curvature (1/m), signed
    double v_ref = 0.0;   // reference speed (m/s)
    double s     = 0.0;   // arc-length from start (m)
};

// ── Trajectory ────────────────────────────────────────────────────────────────
class Trajectory {
public:

    // ── Factory ───────────────────────────────────────────────────────────────
    /// Build from an existing ReferencePath with a kinematic speed profile.
    /// @param v_max       Maximum cruise speed (m/s)
    /// @param a_lat_max   Lateral acceleration budget (m/s²); tighter = slower on curves
    static Trajectory from_path(const vehicle::ReferencePath& path,
                                double v_max,
                                double a_lat_max = 4.0) {
        if (path.size() < 3)
            throw std::invalid_argument("Trajectory::from_path needs ≥ 3 waypoints");

        const auto& wps = path.waypoints();
        const int   N   = static_cast<int>(wps.size());

        Trajectory t;
        t.pts_.resize(N);
        t.closed_ = path.closed();

        // ── Step 1: positions and arc-length ──────────────────────────────
        t.pts_[0].x = wps[0].x;  t.pts_[0].y = wps[0].y;  t.pts_[0].s = 0.0;
        for (int i = 1; i < N; ++i) {
            t.pts_[i].x = wps[i].x;
            t.pts_[i].y = wps[i].y;
            t.pts_[i].s = t.pts_[i-1].s +
                          std::hypot(wps[i].x - wps[i-1].x,
                                     wps[i].y - wps[i-1].y);
        }

        // ── Step 2: tangent heading (central difference) ──────────────────
        for (int i = 0; i < N; ++i) {
            const int prev = std::max(0,   i - 1);
            const int next = std::min(N-1, i + 1);
            t.pts_[i].theta = std::atan2(wps[next].y - wps[prev].y,
                                         wps[next].x - wps[prev].x);
        }

        // ── Step 3: curvature κ = dθ/ds ──────────────────────────────────
        for (int i = 0; i < N; ++i) {
            const int prev = std::max(0,   i - 1);
            const int next = std::min(N-1, i + 1);
            const double ds = t.pts_[next].s - t.pts_[prev].s;
            if (ds < 1.0e-9) { t.pts_[i].kappa = 0.0; continue; }
            const double dtheta = vehicle::wrap_angle(
                t.pts_[next].theta - t.pts_[prev].theta);
            t.pts_[i].kappa = dtheta / ds;
        }

        // ── Step 4: speed profile from lateral acceleration budget ────────
        constexpr double v_min    = 1.0;
        constexpr double kapa_eps = 1.0e-4;
        for (int i = 0; i < N; ++i) {
            const double k = std::abs(t.pts_[i].kappa);
            const double v_curve = (k > kapa_eps)
                                   ? std::sqrt(a_lat_max / k)
                                   : v_max;
            t.pts_[i].v_ref = std::clamp(v_curve, v_min, v_max);
        }

        // ── Step 5: smooth speed profile (3 passes of 3-point average) ───
        // Without smoothing, a single noisy κ spike produces a narrow speed
        // dip that the longitudinal controller can't track at 50 Hz.
        for (int pass = 0; pass < 3; ++pass) {
            std::vector<double> tmp(N);
            for (int i = 0; i < N; ++i) {
                const int prev = std::max(0,   i-1);
                const int next = std::min(N-1, i+1);
                tmp[i] = (t.pts_[prev].v_ref
                        + t.pts_[i   ].v_ref
                        + t.pts_[next].v_ref) / 3.0;
            }
            for (int i = 0; i < N; ++i) t.pts_[i].v_ref = tmp[i];
        }

        return t;
    }

    // ── Nearest-index with forward bias ──────────────────────────────────────
    // Searches a window of 80 points forward from hint so the tracker never
    // snaps back to an earlier lap on closed loops.
    std::size_t nearest_index_forward(double x, double y,
                                      std::size_t hint = 0) const {
        const std::size_t N      = pts_.size();
        const std::size_t window = std::min(N, std::size_t{80});
        std::size_t best  = hint % N;
        double      best_d2 = std::numeric_limits<double>::max();
        for (std::size_t w = 0; w < window; ++w) {
            const std::size_t idx = (hint + w) % N;
            const double dx = x - pts_[idx].x;
            const double dy = y - pts_[idx].y;
            const double d2 = dx*dx + dy*dy;
            if (d2 < best_d2) { best_d2 = d2; best = idx; }
        }
        return best;
    }

    // ── Signed CTE at trajectory index i (positive = vehicle is left of path) ─
    double cross_track_error(const vehicle::State& s, std::size_t i) const {
        const std::size_t j = next_idx(i);
        const double dx  = pts_[j].x - pts_[i].x;
        const double dy  = pts_[j].y - pts_[i].y;
        const double seg = std::hypot(dx, dy);
        if (seg < 1.0e-9) return 0.0;
        const double ex = s.x - pts_[i].x;
        const double ey = s.y - pts_[i].y;
        return (dx * ey - dy * ex) / seg;
    }

    // ── Accessors ─────────────────────────────────────────────────────────────
    const TrajectoryPoint& operator[](std::size_t i) const { return pts_[i]; }
    std::size_t size()   const { return pts_.size(); }
    bool        closed() const { return closed_; }
    const std::vector<TrajectoryPoint>& points() const { return pts_; }

private:
    std::vector<TrajectoryPoint> pts_;
    bool closed_ = false;

    std::size_t next_idx(std::size_t i) const noexcept {
        const std::size_t j = i + 1;
        if (j < pts_.size()) return j;
        return closed_ ? 0 : i;
    }
};

} // namespace control
