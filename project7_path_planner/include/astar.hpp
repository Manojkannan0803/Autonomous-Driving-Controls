#pragma once
// ============================================================
// Project 7 – Path Planner  |  astar.hpp
// A* search on an 8-connected occupancy grid
//
// Challenge Q2: A* vs Dijkstra vs Greedy Best-First
// A: Dijkstra (h≡0) guarantees optimal paths but expands the entire
//    reachable space — O(V log V) in the worst case. Greedy best-first
//    (no g term, only h) is fast but non-optimal and can loop around
//    obstacles to chasing the heuristic. A* (f = g + h) inherits
//    Dijkstra's optimality guarantee and greedy's directedness —
//    it expands only nodes whose estimated total cost beats the
//    current best found path.
//
// Challenge Q3: Why is Euclidean distance admissible on this grid?
// A: An admissible heuristic never OVERestimates the true cost.
//    Diagonal moves cost √2 ≈ 1.41, cardinal moves cost 1.0.
//    The Euclidean distance equals the crow-flies distance, which
//    is always ≤ any actual step sequence → admissible.
//    Manhattan distance IS NOT admissible on 8-connected grids:
//    one diagonal covers Manhattan distance 2 at cost √2 < 2,
//    so Manhattan can overestimate by up to 2× on diagonal routes.
// ============================================================
#include "grid_map.hpp"
#include <vector>
#include <queue>
#include <array>
#include <tuple>
#include <cmath>
#include <limits>
#include <algorithm>

namespace planning {

struct PathResult {
    std::vector<std::pair<double, double>> waypoints; // world-frame (x,y)
    double  length_m{0.0};
    int     nodes_expanded{0};
    bool    found{false};
};

inline PathResult astar(const OccupancyGrid& map, Cell start, Cell goal) {
    // Trivial case
    if (start.row == goal.row && start.col == goal.col) {
        PathResult r;
        r.found = true;
        r.nodes_expanded = 0;
        auto [wx, wy] = map.cell_to_world(start.row, start.col);
        r.waypoints.push_back({wx, wy});
        return r;
    }

    const int ROWS = map.rows();
    const int COLS = map.cols();
    const double INF = std::numeric_limits<double>::infinity();

    std::vector<double> g_cost(static_cast<std::size_t>(ROWS * COLS), INF);
    std::vector<int>    parent(static_cast<std::size_t>(ROWS * COLS), -1);

    auto key = [&](int r, int c) noexcept { return r * COLS + c; };

    // Euclidean heuristic (admissible + consistent on 8-connected grid)
    auto h = [&](int r, int c) noexcept {
        double dr = r - goal.row, dc = c - goal.col;
        return std::sqrt(dr * dr + dc * dc);
    };

    // 8-connected neighbourhood + exact step costs
    constexpr std::array<std::pair<int,int>, 8> DIRS{{
        {-1,-1}, {-1, 0}, {-1, 1},
        { 0,-1},          { 0, 1},
        { 1,-1}, { 1, 0}, { 1, 1}
    }};
    // √2 for diagonals, 1.0 for cardinals — matches the array order above
    const std::array<double, 8> STEP{
        1.41421356237, 1.0, 1.41421356237,
        1.0,               1.0,
        1.41421356237, 1.0, 1.41421356237
    };

    // Min-heap keyed on f = g + h
    using Node = std::tuple<double, int, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    int sk = key(start.row, start.col);
    g_cost[sk] = 0.0;
    open.push({ h(start.row, start.col), start.row, start.col });

    PathResult result;

    while (!open.empty()) {
        auto [f, r, c] = open.top();
        open.pop();
        ++result.nodes_expanded;

        if (r == goal.row && c == goal.col) {
            result.found = true;
            // Back-trace through parent links
            for (int cur = key(r, c); cur != -1; cur = parent[cur]) {
                int pr = cur / COLS, pc = cur % COLS;
                auto [wx, wy] = map.cell_to_world(pr, pc);
                result.waypoints.push_back({wx, wy});
            }
            std::reverse(result.waypoints.begin(), result.waypoints.end());
            // Compute path length
            for (std::size_t i = 1; i < result.waypoints.size(); ++i) {
                double dx = result.waypoints[i].first  - result.waypoints[i-1].first;
                double dy = result.waypoints[i].second - result.waypoints[i-1].second;
                result.length_m += std::sqrt(dx * dx + dy * dy);
            }
            return result;
        }

        // Skip stale heap entries (lazy deletion)
        double gc = g_cost[key(r, c)];
        if (f > gc + h(r, c) + 1e-9) continue;

        for (int d = 0; d < 8; ++d) {
            int nr = r + DIRS[d].first;
            int nc = c + DIRS[d].second;
            if (map.is_occupied(nr, nc)) continue;

            double ng = gc + STEP[d];
            int    nk = key(nr, nc);
            if (ng < g_cost[nk]) {
                g_cost[nk] = ng;
                parent[nk] = key(r, c);
                open.push({ ng + h(nr, nc), nr, nc });
            }
        }
    }

    return result; // path not found
}

} // namespace planning
