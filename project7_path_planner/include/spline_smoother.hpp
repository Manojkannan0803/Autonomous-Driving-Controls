#pragma once
// ============================================================
// Project 7 – Path Planner  |  spline_smoother.hpp
// Natural cubic spline through waypoints (C2 continuous)
//
// Challenge Q3: Why does the controller need C2 continuity?
// A: The LQR feedforward is δ_ff = atan(L·κ_ref). If the path
//    is only C1 (slope-continuous but not curvature-continuous),
//    κ jumps discontinuously at every spline join — producing a
//    step change in δ_ff that the actuator can never satisfy
//    instantaneously. C2 guarantees κ(s) is continuous, so
//    δ_ff changes smoothly. The "natural" boundary condition
//    m_0 = m_n = 0 sets κ=0 at both endpoints, which is correct
//    for a vehicle that starts and stops driving straight.
//
// Algorithm overview:
//   1. Chord-length parameterisation: t_i = Σ‖Δp‖  (avoids Runge
//      oscillations that would appear with equally-spaced t)
//   2. Build tridiagonal system for interior second-derivatives m_i
//      (one equation per interior knot, natural BCs m_0=m_n=0)
//   3. Solve with O(n) Thomas algorithm
//   4. Evaluate separately for x(t) and y(t), sample every ds metres
//   5. Compute signed curvature:  κ = (x'y'' − y'x'') / (x'²+y'²)^(3/2)
// ============================================================
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include <cassert>

namespace planning {

// ── Thomas algorithm: solves tridiagonal A·x = rhs in O(n) ───────────
// lower[i]·x[i-1] + diag[i]·x[i] + upper[i]·x[i+1] = rhs[i]
// lower[0] and upper[n-1] are ignored.
static std::vector<double> thomas_solve(
    const std::vector<double>& lower,
    const std::vector<double>& diag,
    const std::vector<double>& upper,
    const std::vector<double>& rhs)
{
    int n = static_cast<int>(diag.size());
    assert(n > 0);
    std::vector<double> cp(n), dp(n), x(n);
    cp[0] = upper[0] / diag[0];
    dp[0] = rhs[0]   / diag[0];
    for (int i = 1; i < n; ++i) {
        double denom = diag[i] - lower[i] * cp[i-1];
        cp[i] = upper[i] / denom;               // cp[n-1] unused — harmless
        dp[i] = (rhs[i] - lower[i] * dp[i-1]) / denom;
    }
    x[n-1] = dp[n-1];
    for (int i = n - 2; i >= 0; --i)
        x[i] = dp[i] - cp[i] * x[i+1];
    return x;
}

// ── Output sample ─────────────────────────────────────────────────────
struct SplinePoint {
    double x{0.0}, y{0.0}, kappa{0.0}, s{0.0}, v_ref{0.0};
};

// ── Thin dense waypoints to ~min_dist metre spacing ───────────────────
// Reduces A* staircase (1 m grid) to ~3-4 m control-point spacing,
// preventing spline oscillations from over-determined knot placement.
inline std::vector<std::pair<double,double>> thin_waypoints(
    const std::vector<std::pair<double,double>>& pts,
    double min_dist = 3.0)
{
    if (pts.size() < 3) return pts;
    std::vector<std::pair<double,double>> out;
    out.push_back(pts.front());
    for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
        double dx = pts[i].first  - out.back().first;
        double dy = pts[i].second - out.back().second;
        if (std::sqrt(dx*dx + dy*dy) >= min_dist)
            out.push_back(pts[i]);
    }
    out.push_back(pts.back());
    return out;
}

// ── Natural cubic spline + dense sampling ─────────────────────────────
// pts: control points (sparse A* waypoints, already thinned).
// ds:  output sample spacing in metres.
// Returns SplinePoints with x, y, kappa, s populated (v_ref = 0 until
// add_speed_profile() is called).
inline std::vector<SplinePoint> fit_spline(
    const std::vector<std::pair<double,double>>& pts,
    double ds = 0.5)
{
    int n = static_cast<int>(pts.size()) - 1;  // number of segments

    // Degenerate: ≤1 segment → linear interpolation
    if (n < 1) {
        std::vector<SplinePoint> out;
        for (auto& [px, py] : pts)
            out.push_back({px, py, 0.0, 0.0, 0.0});
        return out;
    }

    // ── Chord-length parameterisation ─────────────────────────────────
    std::vector<double> t(n + 1, 0.0);
    for (int i = 1; i <= n; ++i) {
        double dx = pts[i].first  - pts[i-1].first;
        double dy = pts[i].second - pts[i-1].second;
        t[i] = t[i-1] + std::sqrt(dx*dx + dy*dy);
    }
    std::vector<double> h(n);
    for (int i = 0; i < n; ++i) h[i] = t[i+1] - t[i];

    // ── Build & solve tridiagonal for second derivatives m[] ──────────
    // Natural BCs: m[0] = m[n] = 0.
    // Interior unknowns: m[1]..m[n-1].
    int ni = n - 1;  // number of interior knots

    auto solve_coord = [&](const std::vector<double>& y) -> std::vector<double> {
        if (ni < 1) {
            // n=1: one segment, linear → m=[0,0]
            return std::vector<double>(n + 1, 0.0);
        }
        std::vector<double> lo(ni), di(ni), up(ni), rhs(ni);
        for (int i = 0; i < ni; ++i) {
            int ii = i + 1;
            lo[i]  = h[ii-1];
            di[i]  = 2.0 * (h[ii-1] + h[ii]);
            up[i]  = h[ii];
            rhs[i] = 6.0 * ((y[ii+1] - y[ii]) / h[ii]
                           - (y[ii]   - y[ii-1]) / h[ii-1]);
        }
        auto m_int = thomas_solve(lo, di, up, rhs);
        std::vector<double> m(n + 1, 0.0);
        for (int i = 0; i < ni; ++i) m[i+1] = m_int[i];
        return m;
    };

    std::vector<double> xs(n+1), ys(n+1);
    for (int i = 0; i <= n; ++i) { xs[i] = pts[i].first; ys[i] = pts[i].second; }
    auto mx = solve_coord(xs);
    auto my = solve_coord(ys);

    // ── Sample spline at step ds ───────────────────────────────────────
    std::vector<SplinePoint> result;
    double total = t[n];
    double s_cur = 0.0;

    while (s_cur <= total + 1e-9) {
        // Locate segment: find largest i such that t[i] <= s_cur
        int seg = n - 1;
        for (int i = 0; i < n; ++i) {
            if (s_cur < t[i+1]) { seg = i; break; }
        }
        // Clamp to last valid segment
        if (seg >= n) seg = n - 1;

        double u  = s_cur - t[seg];
        double hi = h[seg];

        // Cubic Hermite coefficients for coordinate z[] with second-deriv m[]
        // z(u) = a·u³ + b·u² + c·u + d
        //   a = (m[seg+1] - m[seg]) / (6·h)
        //   b =  m[seg] / 2
        //   c = (z[seg+1]-z[seg])/h - h·(2·m[seg] + m[seg+1])/6
        //   d =  z[seg]

        auto eval0 = [&](const std::vector<double>& z,
                         const std::vector<double>& m) {
            double a = (m[seg+1] - m[seg]) / (6.0 * hi);
            double b =  m[seg] / 2.0;
            double c = (z[seg+1] - z[seg]) / hi
                       - hi * (2.0*m[seg] + m[seg+1]) / 6.0;
            double d =  z[seg];
            return ((a*u + b)*u + c)*u + d;
        };
        auto eval1 = [&](const std::vector<double>& z,
                         const std::vector<double>& m) {
            double a = (m[seg+1] - m[seg]) / (6.0 * hi);
            double b =  m[seg] / 2.0;
            double c = (z[seg+1] - z[seg]) / hi
                       - hi * (2.0*m[seg] + m[seg+1]) / 6.0;
            return (3.0*a*u + 2.0*b)*u + c;
        };
        auto eval2 = [&](const std::vector<double>& /*z*/,
                         const std::vector<double>& m) {
            double a = (m[seg+1] - m[seg]) / (6.0 * hi);
            double b =  m[seg] / 2.0;
            return 6.0*a*u + 2.0*b;
        };

        double xi  = eval0(xs, mx);
        double yi  = eval0(ys, my);
        double dxi = eval1(xs, mx);
        double dyi = eval1(ys, my);
        double d2x = eval2(xs, mx);
        double d2y = eval2(ys, my);

        // Signed curvature: κ = (x'y'' − y'x'') / (x'²+y'²)^(3/2)
        double spd2  = dxi*dxi + dyi*dyi;
        double kappa = (spd2 > 1e-12)
                       ? (dxi*d2y - dyi*d2x) / (spd2 * std::sqrt(spd2))
                       : 0.0;

        result.push_back({xi, yi, kappa, s_cur, 0.0});
        s_cur += ds;
    }
    return result;
}

// ── Speed profile from lateral-acceleration budget ────────────────────
// v_ref(s) = clamp( sqrt(a_lat / max(|κ|, ε)), v_min, v_max )
// Followed by 3-pass exponential-smoothing to avoid abrupt transitions.
inline void add_speed_profile(std::vector<SplinePoint>& path,
                              double v_max = 8.0,
                              double a_lat = 3.5,
                              double v_min = 1.0)
{
    const double eps = 1e-4;
    for (auto& p : path)
        p.v_ref = std::clamp(std::sqrt(a_lat / std::max(std::abs(p.kappa), eps)),
                             v_min, v_max);

    // 3 smoothing passes: each interior point takes the min of itself
    // and the mean of its neighbours + a small headroom buffer.
    for (int pass = 0; pass < 3; ++pass)
        for (std::size_t i = 1; i + 1 < path.size(); ++i)
            path[i].v_ref = std::min(path[i].v_ref,
                                     0.5*(path[i-1].v_ref + path[i+1].v_ref) + 0.5);
}

} // namespace planning
