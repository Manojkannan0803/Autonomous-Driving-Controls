#include <gtest/gtest.h>
#include "grid_map.hpp"
#include "astar.hpp"
#include "spline_smoother.hpp"
#include <cmath>
#include <numeric>

using namespace planning;

// ── OccupancyGrid tests ───────────────────────────────────────────────

TEST(OccupancyGrid, SetObstacle) {
    OccupancyGrid g(10, 10, 1.0);
    EXPECT_TRUE(g.is_free(5, 5));
    g.set_obstacle(5, 5);
    EXPECT_TRUE(g.is_occupied(5, 5));
    EXPECT_TRUE(g.is_free(5, 4));  // neighbour unchanged
}

TEST(OccupancyGrid, OutOfBoundsIsOccupied) {
    OccupancyGrid g(10, 10, 1.0);
    EXPECT_TRUE(g.is_occupied(-1, 0));
    EXPECT_TRUE(g.is_occupied(0, 10));
    EXPECT_TRUE(g.is_occupied(10, 10));
}

TEST(OccupancyGrid, InflateExpandsNeighbours) {
    OccupancyGrid g(10, 10, 1.0);
    g.set_obstacle(5, 5);
    g.inflate(2);
    // All cells within Chebyshev distance 2 must be occupied
    for (int dr = -2; dr <= 2; ++dr)
        for (int dc = -2; dc <= 2; ++dc)
            EXPECT_TRUE(g.is_occupied(5+dr, 5+dc));
    // Cell at Chebyshev distance 3 in a corner direction should still be free
    EXPECT_TRUE(g.is_free(1, 1));
}

TEST(OccupancyGrid, WorldToCell) {
    OccupancyGrid g(10, 10, 2.0, 0.0, 0.0); // 2 m/cell
    auto c = g.world_to_cell(5.0, 3.0);     // x=5→col=2, y=3→row=1
    EXPECT_EQ(c.row, 1);
    EXPECT_EQ(c.col, 2);
}

TEST(OccupancyGrid, CellToWorldRoundtrip) {
    OccupancyGrid g(20, 20, 1.0, 0.0, 0.0);
    auto [wx, wy] = g.cell_to_world(7, 3);  // centre of cell (7,3)
    auto c = g.world_to_cell(wx, wy);
    EXPECT_EQ(c.row, 7);
    EXPECT_EQ(c.col, 3);
}

// ── A* tests ─────────────────────────────────────────────────────────

TEST(AStar, FindsPathOpenGrid) {
    OccupancyGrid g(20, 20, 1.0);
    auto result = astar(g, {0,0}, {19,19});
    EXPECT_TRUE(result.found);
    EXPECT_GT(result.waypoints.size(), 0u);
    EXPECT_GT(result.length_m, 0.0);
}

TEST(AStar, NoPathWhenBlocked) {
    OccupancyGrid g(10, 10, 1.0);
    // Wall across every column of row 5
    for (int c = 0; c < 10; ++c) g.set_obstacle(5, c);
    auto result = astar(g, {0,0}, {9,9});
    EXPECT_FALSE(result.found);
}

TEST(AStar, PathAvoidsObstacles) {
    OccupancyGrid g(20, 20, 1.0);
    g.set_rect(5, 0, 15, 15);  // large block
    auto result = astar(g, {0,0}, {19,19});
    ASSERT_TRUE(result.found);
    // Every waypoint must be in a free cell
    for (auto& [wx, wy] : result.waypoints) {
        auto c = g.world_to_cell(wx, wy);
        EXPECT_TRUE(g.is_free(c.row, c.col));
    }
}

TEST(AStar, StartEqualsGoal) {
    OccupancyGrid g(10, 10, 1.0);
    auto result = astar(g, {3,3}, {3,3});
    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.waypoints.size(), 1u);
    EXPECT_DOUBLE_EQ(result.length_m, 0.0);
}

TEST(AStar, PathStartsAndEndsAtCorrectCells) {
    OccupancyGrid g(20, 20, 1.0);
    Cell start{1,1}, goal{18,18};
    auto result = astar(g, start, goal);
    ASSERT_TRUE(result.found);
    // First waypoint ≈ centre of start cell
    auto [sx, sy] = g.cell_to_world(start.row, start.col);
    EXPECT_NEAR(result.waypoints.front().first,  sx, 1e-9);
    EXPECT_NEAR(result.waypoints.front().second, sy, 1e-9);
    auto [gx, gy] = g.cell_to_world(goal.row, goal.col);
    EXPECT_NEAR(result.waypoints.back().first,   gx, 1e-9);
    EXPECT_NEAR(result.waypoints.back().second,  gy, 1e-9);
}

// ── Spline tests ──────────────────────────────────────────────────────

TEST(Spline, MonotonicS) {
    std::vector<std::pair<double,double>> pts = {
        {0,0},{5,3},{10,0},{15,3},{20,0}};
    auto sp = fit_spline(pts, 0.5);
    ASSERT_GT(sp.size(), 1u);
    for (std::size_t i = 1; i < sp.size(); ++i)
        EXPECT_GT(sp[i].s, sp[i-1].s);
}

TEST(Spline, EndpointsMatchInput) {
    std::vector<std::pair<double,double>> pts = {{0,0},{10,5},{20,0}};
    auto sp = fit_spline(pts, 0.1);
    ASSERT_FALSE(sp.empty());
    EXPECT_NEAR(sp.front().x, 0.0, 0.05);
    EXPECT_NEAR(sp.front().y, 0.0, 0.05);
    EXPECT_NEAR(sp.back().x, 20.0, 0.05);
    EXPECT_NEAR(sp.back().y,  0.0, 0.05);
}

TEST(Spline, CurvatureFinite) {
    std::vector<std::pair<double,double>> pts = {
        {0,0},{5,3},{10,0},{15,-3},{20,0}};
    auto sp = fit_spline(pts, 0.5);
    for (auto& p : sp) {
        EXPECT_TRUE(std::isfinite(p.kappa));
    }
}

TEST(SpeedProfile, VMaxRespected) {
    std::vector<std::pair<double,double>> pts = {
        {0,0},{5,5},{10,0},{15,5},{20,0}};
    auto sp = fit_spline(pts, 0.5);
    add_speed_profile(sp, 6.0, 3.5);
    for (auto& p : sp)
        EXPECT_LE(p.v_ref, 6.0 + 1e-9);
}

TEST(SpeedProfile, StraightSectionGetsVMax) {
    // Perfectly straight line → κ≈0 everywhere → v_ref should reach v_max
    std::vector<std::pair<double,double>> pts;
    for (int i = 0; i <= 20; ++i) pts.push_back({static_cast<double>(i), 0.0});
    auto sp = fit_spline(pts, 0.5);
    add_speed_profile(sp, 8.0, 3.5);
    // Check the middle of the path (away from smoothing boundary)
    std::size_t mid = sp.size() / 2;
    EXPECT_NEAR(sp[mid].v_ref, 8.0, 0.5);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
