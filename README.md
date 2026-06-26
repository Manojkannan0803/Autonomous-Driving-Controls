# AV Controls Portfolio — C++20

A progressive nine-project portfolio implementing the core software stack of an
autonomous vehicle: from numerical mathematics up to a full closed-loop planning,
estimation, and control pipeline.  Every project is written in **standard C++20**
with no third-party dependencies (no Eigen, no ROS, no Boost).

---

## Portfolio at a Glance

| # | Project | Key Algorithm | Output |
|---|---|---|---|
| 1 | Numerical Integrators | Euler / RK2 / RK4 | Convergence plots |
| 2 | PID Controller | Anti-windup, D-filter | Step response plots |
| 3 | Bicycle Model | Kinematic 4-state ODE + RK4 | Trajectory simulation |
| 4 | Lateral Controllers | Pure Pursuit · Stanley · LQR | CTE comparison |
| 5 | Kalman Filter / EKF | Linear KF + CTRV EKF, GPS+IMU fusion | RMSE plots |
| 6 | LQR Trajectory Tracker | Time-varying gains + curvature feedforward | Path tracking |
| 7 | Path Planner | A\* on occupancy grid + cubic spline smoother | Urban path plots |
| 8 | MPC Controller | FISTA QP, N=15 horizon, box constraints | Constraint comparison |
| 9 | Mini AV Stack | P3+P5+P7+P8 integrated closed loop | Full pipeline demo |

---

## Prerequisites

| Tool | Version | Notes |
|---|---|---|
| **GCC (MinGW-w64)** | ≥ 13 (C++20) | Required. Installed at `C:\Program Files\mingw64\bin` by default |
| **Python** | ≥ 3.9 | Required for visualizations |
| `matplotlib` | any | `pip install matplotlib` |
| `numpy` | any | `pip install numpy` |
| CMake | ≥ 3.16 | Optional — only needed if you want to build GoogleTest unit tests |

> **Python packages in one command**
> ```
> pip install matplotlib numpy
> ```

---

## Repository Structure

```
c++/
├── build.bat                        ← one-command build script (Windows)
├── CMakeLists.txt                   ← root CMake (optional, for tests)
├── build/                           ← compiled executables (created by build.bat)
│
├── project1_integrators/
│   ├── include/integrators.hpp      ← header-only library
│   ├── src/main.cpp
│   ├── tests/test_integrators.cpp
│   ├── visualize.py
│   └── README.md
│
├── project2_pid_controller/
│   ├── include/pid_controller.hpp
│   ├── include/vehicle_plant.hpp
│   ├── src/main.cpp
│   ├── tests/test_pid.cpp
│   ├── visualize.py
│   └── README.md
│
├── project3_bicycle_model/
│   ├── include/bicycle_model.hpp
│   ├── include/reference_path.hpp
│   ├── src/main.cpp
│   ├── tests/test_bicycle.cpp
│   ├── visualize.py
│   └── README.md
│
├── project4_lateral_controllers/
│   ├── include/controllers.hpp
│   ├── src/main.cpp
│   ├── tests/test_controllers.cpp
│   ├── visualize.py
│   └── README.md
│
├── project5_kalman_filter/
│   ├── include/kalman_filter.hpp
│   ├── include/sensor_models.hpp
│   ├── src/main.cpp
│   ├── tests/test_kalman.cpp
│   ├── visualize.py
│   └── README.md
│
├── project6_lqr_tracker/
│   ├── include/lqr_tracker.hpp
│   ├── include/trajectory.hpp
│   ├── src/main.cpp
│   ├── tests/test_lqr.cpp
│   ├── visualize.py
│   └── README.md
│
├── project7_path_planner/
│   ├── include/grid_map.hpp
│   ├── include/astar.hpp
│   ├── include/spline_smoother.hpp
│   ├── src/main.cpp
│   ├── tests/test_path_planner.cpp
│   ├── visualize.py
│   └── README.md
│
├── project8_mpc_controller/
│   ├── include/mpc_tracker.hpp
│   ├── src/main.cpp
│   ├── tests/test_mpc_tracker.cpp
│   ├── visualize.py
│   └── README.md
│
└── project9_av_stack/
    ├── src/main.cpp
    ├── tests/test_av_stack.cpp
    ├── visualize.py
    └── README.md
```

---

## Quick Start — Build All Projects

Open a **PowerShell** or **Command Prompt** in this folder and run:

```bat
build.bat
```

This compiles all 9 demo executables into `build\`:

```
build\integrators_demo.exe
build\pid_demo.exe
build\bicycle_demo.exe
build\controllers_demo.exe
build\kalman_demo.exe
build\lqr_demo.exe
build\path_planner_demo.exe
build\mpc_demo.exe
build\av_stack_demo.exe
```

> **GCC path**: `build.bat` assumes GCC at `C:\Program Files\mingw64\bin`.
> Edit the `GCC_PATH` variable at the top of `build.bat` if your installation differs.

---

## Running Each Demo

Each demo writes CSV data files to its **own project folder**, then you run
`visualize.py` from that folder to generate PNG plots.

### Run all demos and visualize in sequence

```powershell
# From the repo root

# Project 1
.\build\integrators_demo.exe
cd project1_integrators; python visualize.py; cd ..

# Project 2
.\build\pid_demo.exe
cd project2_pid_controller; python visualize.py; cd ..

# Project 3
.\build\bicycle_demo.exe
cd project3_bicycle_model; python visualize.py; cd ..

# Project 4
.\build\controllers_demo.exe
cd project4_lateral_controllers; python visualize.py; cd ..

# Project 5
.\build\kalman_demo.exe
cd project5_kalman_filter; python visualize.py; cd ..

# Project 6
.\build\lqr_demo.exe
cd project6_lqr_tracker; python visualize.py; cd ..

# Project 7
.\build\path_planner_demo.exe
cd project7_path_planner; python visualize.py; cd ..

# Project 8
.\build\mpc_demo.exe
cd project8_mpc_controller; python visualize.py; cd ..

# Project 9
.\build\av_stack_demo.exe
cd project9_av_stack; python visualize.py; cd ..
```

> **Shortcut**: `visualize.py` in each project auto-runs the demo binary if the
> CSV files are missing, so you can skip the manual demo step and just run:
> ```powershell
> cd project9_av_stack; python visualize.py
> ```

---

## Generated Figures

After running all visualizers, each project folder contains PNG plots:

| Project | Figures generated |
|---|---|
| P1 | `exp_decay.png` · `harmonic.png` · `convergence.png` |
| P2 | `pid_comparison.png` · `speed_change.png` · `bad_weather.png` |
| P3 | `trajectories.png` |
| P4 | `trajectories.png` · `cte_comparison.png` |
| P5 | `trajectory.png` · `errors.png` · `innovations.png` |
| P6 | `trajectories.png` · `cte_comparison.png` · `speed_profile.png` |
| P7 | `map_path.png` · `curvature.png` |
| P8 | `fig8_comparison.png` · `urban_comparison.png` · `summary.png` |
| P9 | `stack_paths.png` · `cte_comparison.png` · `estimation_error.png` · `summary.png` |

---

## Running Unit Tests (GoogleTest)

Unit tests require **CMake ≥ 3.16**.  GoogleTest is fetched automatically via
`FetchContent` during the CMake configure step (internet connection required once).

```powershell
# From the repo root
mkdir build_tests
cd build_tests
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel
ctest --output-on-failure
```

Test executables are named `*_tests.exe` (e.g. `integrators_tests.exe`,
`kalman_tests.exe`) and can also be run directly.

---

## Project Dependency Chain

Each project builds on top of the previous ones:

```
P1 Integrators
    └── P3 Bicycle Model  (uses RK4 from P1, inlined)
            ├── P4 Lateral Controllers
            ├── P5 Kalman Filter + EKF
            │       └── P9 AV Stack
            └── P6 LQR Tracker
                    └── P8 MPC Controller
                            └── P9 AV Stack
P7 Path Planner
    └── P8 MPC Controller
            └── P9 AV Stack
```

P9 is the integration capstone that uses every other project simultaneously.

---

## Key Results Summary

| Metric | Value |
|---|---|
| A\* planning (60×60 urban map) | **0.53 ms** |
| MPC FISTA solve (N=15 horizon) | **0.062 ms / step** |
| Full AV pipeline per 20 ms cycle | **0.4 % CPU budget** |
| MPC steering violations (urban path) | **0** (vs LQR's 412) |
| EKF GPS noise reduction | **31 %** (3.0 m → 2.08 m RMSE) |
| RK4 vs Euler accuracy at same dt | **10,000×** fewer steps needed |

---

## Compiler & Platform Notes

- All code is **C++20** (`-std=c++20`). Requires GCC ≥ 13 or Clang ≥ 14.
- Tested on **Windows 11** with MinGW-w64 GCC 15.1.0.
- No platform-specific code — the sources are portable to Linux/macOS with the
  same `g++` command (replace `build.bat` with a shell script).
- No third-party C++ libraries. All matrix math is hand-rolled with a thin
  `Mat<R,C>` template (`std::array`-backed, stack-allocated, row-major).
