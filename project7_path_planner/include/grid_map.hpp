#pragma once
// ============================================================
// Project 7 – Path Planner  |  grid_map.hpp
// 2D occupancy grid: obstacle storage, inflation, coord conversion
//
// Challenge Q1: Why inflate obstacles before planning?
// A: A* treats the vehicle as a point mass. Inflating every obstacle
//    outward by half the vehicle's half-width transforms the problem
//    so the POINT can plan freely — yet the FULL footprint stays
//    collision-free. This technique is called "configuration-space
//    (C-space) expansion". Over-inflation wastes space; under-inflation
//    risks collisions — so the radius ≈ ceil(half_width / resolution).
// ============================================================
#include <vector>
#include <cmath>
#include <utility>

namespace planning {

// ── Cell index (row = y-axis, col = x-axis) ───────────────────────────
struct Cell { int row{0}, col{0}; };

// ── 2D occupancy grid ─────────────────────────────────────────────────
class OccupancyGrid {
public:
    // rows × cols cells, each res metres square.
    // Origin (ox, oy) is the world position of cell (0,0)'s corner.
    OccupancyGrid(int rows, int cols, double resolution,
                  double origin_x = 0.0, double origin_y = 0.0)
        : rows_(rows), cols_(cols), res_(resolution),
          ox_(origin_x), oy_(origin_y),
          grid_(static_cast<std::size_t>(rows * cols), false) {}

    // Mark a single cell as occupied.
    void set_obstacle(int r, int c) {
        if (in_bounds(r, c)) grid_[idx(r, c)] = true;
    }

    // Mark a filled rectangle [r0..r1] × [c0..c1] as occupied.
    void set_rect(int r0, int c0, int r1, int c1) {
        for (int r = r0; r <= r1; ++r)
            for (int c = c0; c <= c1; ++c)
                set_obstacle(r, c);
    }

    // Inflate every occupied cell outward by 'radius' cells (Chebyshev).
    // Run this once after all obstacles are placed.
    void inflate(int radius) {
        std::vector<bool> orig = grid_;          // snapshot before expansion
        for (int r = 0; r < rows_; ++r)
            for (int c = 0; c < cols_; ++c)
                if (orig[idx(r, c)])
                    for (int dr = -radius; dr <= radius; ++dr)
                        for (int dc = -radius; dc <= radius; ++dc)
                            if (in_bounds(r + dr, c + dc))
                                grid_[idx(r + dr, c + dc)] = true;
    }

    bool is_occupied(int r, int c) const {
        return !in_bounds(r, c) || grid_[idx(r, c)];
    }
    bool is_free(int r, int c) const { return !is_occupied(r, c); }

    // World (x,y)  →  nearest cell
    Cell world_to_cell(double x, double y) const {
        return { static_cast<int>((y - oy_) / res_),
                 static_cast<int>((x - ox_) / res_) };
    }

    // Cell centre  →  world (x,y)
    std::pair<double, double> cell_to_world(int r, int c) const {
        return { ox_ + (c + 0.5) * res_,
                 oy_ + (r + 0.5) * res_ };
    }

    int    rows()       const { return rows_; }
    int    cols()       const { return cols_; }
    double resolution() const { return res_;  }

    // Raw grid data for serialisation (row-major, true = occupied).
    const std::vector<bool>& data() const { return grid_; }

private:
    int    rows_, cols_;
    double res_, ox_, oy_;
    std::vector<bool> grid_;

    bool        in_bounds(int r, int c) const {
        return r >= 0 && r < rows_ && c >= 0 && c < cols_;
    }
    std::size_t idx(int r, int c) const {
        return static_cast<std::size_t>(r * cols_ + c);
    }
};

} // namespace planning
