// =============================================================================
// reference_path.hpp  —  Reference path and geometric error computation
// =============================================================================
// A ReferencePath is the road the vehicle is supposed to follow.
// It is represented as an ordered list of (x, y) waypoints.
//
// Two key error signals used by all path-following controllers:
//
//   Cross-Track Error (CTE)
//   ─────────────────────────────────────────────────────────────────────────
//   Signed perpendicular distance from the vehicle to the nearest path segment.
//   Positive  = vehicle is LEFT of the path direction.
//   Negative  = vehicle is RIGHT of the path direction.
//
//   Computed via 2-D cross product:
//       CTE = (path_dir × vehicle_offset) / |path_dir|
//
//   Heading Error (ψ_e)
//   ─────────────────────────────────────────────────────────────────────────
//   Difference between vehicle heading and path tangent at nearest point.
//   Positive  = vehicle is turned LEFT relative to path direction.
//   Always wrapped into (-π, π].
//
// Factory methods produce common reference paths used in AV testing:
//   circle()      — constant-radius loop
//   figure_eight()— Lissajous-1:2 curve (stress-tests lateral controllers)
//   s_curve()     — sinusoidal lane change (standard automotive test)
// =============================================================================
#pragma once

#include "bicycle_model.hpp"

#include <cmath>
#include <numbers>
#include <vector>
#include <stdexcept>

namespace vehicle {

// ── Waypoint ──────────────────────────────────────────────────────────────────

struct Waypoint {
    double x = 0.0;
    double y = 0.0;
};

// ── ReferencePath ─────────────────────────────────────────────────────────────

class ReferencePath {
public:

    // ── Factory methods ───────────────────────────────────────────────────────

    /// Circle of given radius centred at the origin.
    /// Vehicle placed at (radius, 0) should trace this path exactly
    /// with heading θ = π/2 and steering δ = arctan(L/radius).
    static ReferencePath circle(double radius, int n = 300) {
        if (radius <= 0.0) throw std::invalid_argument("circle: radius must be positive");
        ReferencePath p;
        p.closed_ = true;
        for (int i = 0; i < n; ++i) {
            const double a = 2.0 * std::numbers::pi * i / n;
            p.pts_.push_back({radius * std::cos(a), radius * std::sin(a)});
        }
        return p;
    }

    /// Lissajous 1:2 figure-eight (smooth, closed, self-intersecting at origin).
    ///    x(t) = size · sin(t)
    ///    y(t) = size · sin(2t) / 2
    static ReferencePath figure_eight(double size, int n = 400) {
        if (size <= 0.0) throw std::invalid_argument("figure_eight: size must be positive");
        ReferencePath p;
        p.closed_ = true;
        for (int i = 0; i < n; ++i) {
            const double t = 2.0 * std::numbers::pi * i / n;
            p.pts_.push_back({size * std::sin(t),
                              size * std::sin(2.0 * t) / 2.0});
        }
        return p;
    }

    /// S-curve lane change: sinusoidal lateral displacement over a straight run.
    /// Standard automotive reference for lateral controller benchmarking.
    ///   x(s) = s · length
    ///   y(s) = amplitude · sin(2π·s),   s ∈ [0, 1]
    static ReferencePath s_curve(double length, double amplitude, int n = 200) {
        if (length <= 0.0) throw std::invalid_argument("s_curve: length must be positive");
        ReferencePath p;
        for (int i = 0; i < n; ++i) {
            const double s = static_cast<double>(i) / (n - 1);
            p.pts_.push_back({s * length,
                              amplitude * std::sin(2.0 * std::numbers::pi * s)});
        }
        return p;
    }

    /// Straight line along the X-axis.
    static ReferencePath straight(double length, int n = 100) {
        ReferencePath p;
        for (int i = 0; i < n; ++i)
            p.pts_.push_back({length * i / (n - 1), 0.0});
        return p;
    }

    // ── Manual construction ───────────────────────────────────────────────────

    void add(Waypoint wp) { pts_.push_back(wp); }

    /// Build from an existing list of (x, y) waypoints.
    static ReferencePath from_waypoints(
        const std::vector<std::pair<double,double>>& wpts,
        bool closed = false)
    {
        ReferencePath p;
        p.closed_ = closed;
        for (auto& [x, y] : wpts) p.pts_.push_back({x, y});
        return p;
    }

    // ── Nearest waypoint (linear search from a hint index) ───────────────────
    // In a production controller this would use a KD-tree or a sliding window.
    // Here we search forward from 'hint' to avoid snapping to a distant loop.
    std::size_t nearest_index(double x, double y,
                               std::size_t hint = 0) const {
        if (pts_.empty()) throw std::runtime_error("ReferencePath is empty");

        // Search the whole path; a sliding window would be O(1) amortised.
        std::size_t best = 0;
        double best_d2   = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < pts_.size(); ++i) {
            const double dx = x - pts_[i].x;
            const double dy = y - pts_[i].y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best = i; }
        }
        (void)hint;  // kept for API compatibility with future KD-tree upgrade
        return best;
    }

    // ── Cross-Track Error (signed perpendicular distance) ─────────────────────
    // Derivation:
    //   Let the nearest segment run from P_i to P_{i+1}.
    //   Path direction unit vector: d̂ = (P_{i+1} - P_i) / |P_{i+1} - P_i|
    //   Vehicle offset from segment start: e = vehicle_pos - P_i
    //   CTE = d̂ × e  (2-D cross product, gives the perpendicular component)
    //       = (dx·ey - dy·ex) / |segment|
    double cross_track_error(const State& s) const {
        const std::size_t i = nearest_index(s.x, s.y);
        const std::size_t j = next_idx(i);

        const double dx = pts_[j].x - pts_[i].x;    // path segment direction
        const double dy = pts_[j].y - pts_[i].y;
        const double seg = std::hypot(dx, dy);
        if (seg < 1.0e-9) return 0.0;                // degenerate segment

        const double ex = s.x - pts_[i].x;          // vehicle offset
        const double ey = s.y - pts_[i].y;

        return (dx * ey - dy * ex) / seg;
    }

    // ── Heading Error ─────────────────────────────────────────────────────────
    // Path tangent at the nearest segment, then difference from vehicle heading.
    double heading_error(const State& s) const {
        const std::size_t i = nearest_index(s.x, s.y);
        const std::size_t j = next_idx(i);

        const double path_heading = std::atan2(pts_[j].y - pts_[i].y,
                                               pts_[j].x - pts_[i].x);
        return wrap_angle(s.theta - path_heading);
    }

    /// Path tangent heading (radians) at waypoint index i.
    double tangent_heading(std::size_t i) const {
        const std::size_t j = next_idx(i);
        return std::atan2(pts_[j].y - pts_[i].y, pts_[j].x - pts_[i].x);
    }

    // ── Accessors ─────────────────────────────────────────────────────────────
    const std::vector<Waypoint>& waypoints() const noexcept { return pts_; }
    std::size_t size()   const noexcept { return pts_.size(); }
    bool        closed() const noexcept { return closed_; }
    const Waypoint& operator[](std::size_t i) const { return pts_[i]; }

private:
    std::vector<Waypoint> pts_;
    bool closed_ = false;

    /// Next index, wrapping if closed path.
    std::size_t next_idx(std::size_t i) const noexcept {
        const std::size_t j = i + 1;
        if (j < pts_.size()) return j;
        return closed_ ? 0 : i;   // open path: clamp at last point
    }
};

} // namespace vehicle
