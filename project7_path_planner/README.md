# Project 7 — Path Planner (A\* + Cubic Spline)

## Problem Statement

Controllers from P4–P6 assume a reference path already exists.  A real AV must
**generate** that path from scratch given a map with obstacles.  This project implements a
complete motion planning pipeline:

```
Occupancy grid  →  A* graph search  →  Waypoint list
                →  Spline smoothing →  Smooth path with curvature & speed
```

The output feeds directly into P6 (LQR) and P8 (MPC) as the reference trajectory.

Two key challenges are addressed:
1. **Completeness**: A* guarantees finding the shortest path if one exists.
2. **Smoothness**: Raw A* outputs are piecewise-linear (grid-aligned);
   a cubic spline smoother converts them to C² continuous curves suitable for
   a kinematic bicycle model (which has finite turning radius).

---

## Architecture

```
grid_map.hpp  (namespace planning)
│
├── OccupancyGrid { rows, cols, cell_size }
├── set_rect(r0,c0,r1,c1)    — mark rectangular obstacle
├── inflate(radius)          — expand obstacles by robot radius
└── cell_to_world / world_to_cell

astar.hpp  (namespace planning)
│
├── PathResult { waypoints, length_m, nodes_expanded, found }
└── astar(map, start_cell, goal_cell) → PathResult

spline_smoother.hpp  (namespace planning)
│
├── SplinePoint { x, y, kappa, s, v_ref }
├── thin_waypoints(pts, min_spacing)     — decimate dense A* output
├── fit_spline(pts, ds)                  — natural cubic spline, sampled at ds
├── add_speed_profile(pts, v_max, a_lat) — adds κ-limited v_ref to each point
└── evaluate at arc-length

src/main.cpp
├── 60×60 m map with 4 rectangular building obstacles
├── A* from cell(2,2) to cell(57,57)
└── Writes: astar_path.csv, spline_path.csv, map_grid.csv
```

---

## Design & Implementation

### Occupancy Grid

A 60×60 grid with 1 m/cell resolution represents a 60 m × 60 m urban block.
Four 14 m × 14 m buildings are placed in the four quadrants.  The grid is then
**inflated** by 2 m (robot safety radius) — every free cell within 2 m of an obstacle is
marked occupied.  This "configuration-space obstacle" means the vehicle centre can be
planned as a point robot while guaranteeing clearance for a 4 m wide car.

### A\* Search

**8-connected grid**: diagonal moves are allowed (cost √2 ≈ 1.414; cardinal moves cost 1.0).

**Heuristic**: Euclidean distance to goal.  This is **admissible** on an 8-connected grid
(never overestimates true cost) because no actual move sequence can be shorter than the
straight-line distance.  Manhattan distance is *not* admissible on 8-connected grids —
one diagonal covers `dx=1, dy=1` at cost √2 < 2, so Manhattan can overestimate by up to 2×.

**Priority queue with lazy deletion**: nodes are inserted into a `std::priority_queue`
with their `f = g + h` score.  When a better path to a node is found, the old entry is not
removed (expensive); instead it is marked stale via the `g_cost` array and skipped when
popped.

**Performance**: the 60×60 map with 4 inflated buildings expanded 2,125 nodes and found the
94.5 m path in **0.53 ms** on a mid-range laptop.

### Spline Smoothing

**Step 1 — Thinning**: Raw A* outputs one waypoint per grid cell, giving up to 95 colinear
points on straight segments.  `thin_waypoints(raw, 3.0)` keeps every waypoint that changes
direction by > threshold or is > 3 m from the previous kept point.  Reduces ~95 to ~30 control points.

**Step 2 — Natural cubic spline**: Given `n` control points, solve the `n×n` tridiagonal
system (Thomas algorithm, O(n)) for the spline coefficients.  Boundary conditions: zero
second derivative at endpoints (natural spline), ensuring curvature is zero at the start/goal
— critical for the vehicle to start and end with zero steering.

**Step 3 — Resampling at uniform arc-length**: The spline is evaluated at `ds = 0.5 m`
intervals using Newton's method to invert the arc-length integral, producing a path with
equal spacing regardless of curvature.

**Step 4 — Speed profile**: same `v_ref = sqrt(a_lat_max / |κ|)` formula as P6, applied to
the spline curvature.  Max curvature on the demo path is `κ_max = 0.40 1/m`, which limits
speed to `sqrt(3.5/0.40) = 2.96 m/s` at the tightest corner.

---

## Test & Validation

| Test | What it checks |
|---|---|
| `astar_finds_path` | Returns `found=true` for reachable goal |
| `astar_blocked_goal` | Returns `found=false` when goal is inside obstacle |
| `astar_same_cell` | Trivial case: start == goal, one waypoint returned |
| `astar_path_length` | Length matches known shortest path (±5 %) |
| `inflation_blocks_narrow` | Cells within 2 m of obstacle are occupied |
| `spline_c2_continuity` | Curvature is continuous (no jumps > 0.01 1/m) at knots |
| `spline_endpoints_match` | Spline passes through first and last control points |
| `speed_profile_bounded` | v_ref ≤ v_max everywhere |
| `thin_waypoints_reduces` | Output has fewer points than input |
| `arc_length_monotone` | s strictly increases along spline |

---

## Figures & Trend Rationale

### `map_path.png` — Occupancy Grid + A\* + Spline

The figure shows the 60 m × 60 m map with:

- **Grey rectangles**: the four building obstacles (8–22 m and 35–50 m ranges in x and y).
- **Light grey border**: the 2 m inflation region around each building.
- **Blue dots / lines**: raw A\* waypoints — grid-aligned, diagonal, visibly jagged.
- **Red curve**: the natural cubic spline — smooth C² curve threading the same corridor.
- **Green dot (2,2) → Red dot (57,57)**: start and goal.

**Why the path goes around the outside of the buildings**: the 2 m inflation pushes the
corridor between the left-column buildings (columns 8–22) and the right-column buildings
(columns 35–50) below the minimum traversable width.  A\* correctly finds the route that
goes through the centre corridor, making two right-angle turns around the inner corners.

**Why the A\* path looks diagonal in the open areas**: the 8-connected grid allows 45°
diagonal moves, which are shorter than two cardinal moves covering the same displacement.
A\* uses these wherever the path is not constrained by an obstacle.

### `curvature.png` — Path Curvature vs. Arc-Length

The curvature profile shows three distinct regions:

1. **Near-zero curvature** on straight segments (|κ| < 0.05 1/m): A\* found essentially
   straight corridors; the spline reflects this.
2. **Curvature spikes** (|κ| up to 0.40 1/m) at the two building corners where the path
   changes direction ~90°.  These are the tightest turns in the path — a vehicle must slow
   to `sqrt(3.5/0.40) ≈ 3.0 m/s` to keep lateral acceleration < 3.5 m/s².
3. **Smooth transitions** between the spikes: the natural cubic spline produces C² continuity,
   meaning curvature varies continuously with no step discontinuities that would demand
   instantaneous steering changes.

The speed overlay on the same axis shows the anti-correlation between curvature and speed —
every curvature peak is matched by a speed dip.  This is the `κ`-limited speed profile
doing its job: the planner "knows" the path's geometry and pre-schedules the speed before
the controller even runs.
