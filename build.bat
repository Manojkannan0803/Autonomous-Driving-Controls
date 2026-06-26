@echo off
:: ============================================================================
:: build.bat  —  Build the AV Controls Portfolio
:: Requires:  MinGW GCC in PATH (or adjust GCC_PATH below)
:: ============================================================================

set GCC_PATH=C:\Program Files\mingw64\bin
set BASE=%~dp0
set BUILD=%BASE%build
set CXX="%GCC_PATH%\g++.exe"
set FLAGS=-std=c++20 -Wall -Wextra -O2

if not exist "%BUILD%" mkdir "%BUILD%"

echo.
echo ============================================================
echo   Building AV Controls Portfolio
echo ============================================================

:: ── Project 1: Numerical Integrators ────────────────────────────────────────
echo.
echo [1/2] project1_integrators ...
%CXX% %FLAGS% ^
    -I "%BASE%project1_integrators\include" ^
    "%BASE%project1_integrators\src\main.cpp" ^
    -o "%BUILD%\integrators_demo.exe"
if errorlevel 1 ( echo   FAILED & goto :end )
echo   OK  ->  build\integrators_demo.exe

:: ── Project 2: PID Controller ────────────────────────────────────────────────
echo.
echo [2/3] project2_pid_controller ...
%CXX% %FLAGS% ^
    -I "%BASE%project2_pid_controller\include" ^
    "%BASE%project2_pid_controller\src\main.cpp" ^
    -o "%BUILD%\pid_demo.exe"
if errorlevel 1 ( echo   FAILED & goto :end )
echo   OK  ->  build\pid_demo.exe

:: ── Project 3: Bicycle Model ─────────────────────────────────────────────────
echo.
echo [3/4] project3_bicycle_model ...
%CXX% %FLAGS% ^
    -I "%BASE%project3_bicycle_model\include" ^
    "%BASE%project3_bicycle_model\src\main.cpp" ^
    -o "%BUILD%\bicycle_demo.exe"
if errorlevel 1 ( echo   FAILED & goto :end )
echo   OK  ->  build\bicycle_demo.exe

:: ── Project 4: Lateral Controllers ──────────────────────────────────────────
echo.
echo [4/5] project4_lateral_controllers ...
%CXX% %FLAGS% ^
    -I "%BASE%project4_lateral_controllers\include" ^
    -I "%BASE%project3_bicycle_model\include" ^
    "%BASE%project4_lateral_controllers\src\main.cpp" ^
    -o "%BUILD%\lateral_demo.exe"
if errorlevel 1 ( echo   FAILED & goto :end )
echo   OK  ->  build\lateral_demo.exe

:: ── Project 5: Kalman Filter + EKF ─────────────────────────────────────────
echo.
echo [5/6] project5_kalman_filter ...
%CXX% %FLAGS% ^
    -I "%BASE%project5_kalman_filter\include" ^
    -I "%BASE%project3_bicycle_model\include" ^
    "%BASE%project5_kalman_filter\src\main.cpp" ^
    -o "%BUILD%\kalman_demo.exe"
if errorlevel 1 ( echo   FAILED & goto :end )
echo   OK  ->  build\kalman_demo.exe

:: ── Project 6: LQR Trajectory Tracker ────────────────────────────────
echo.
echo [6/7] project6_lqr_tracker ...
%CXX% %FLAGS% ^
    -I "%BASE%project6_lqr_tracker\include" ^
    -I "%BASE%project3_bicycle_model\include" ^
    "%BASE%project6_lqr_tracker\src\main.cpp" ^
    -o "%BUILD%\lqr_tracker_demo.exe"
if errorlevel 1 ( echo   FAILED & goto :end )
echo   OK  ->  build\lqr_tracker_demo.exe

:: ── Project 7: Path Planner (A* + Cubic Spline) ────────────────────────────
echo.
echo [7/8] project7_path_planner ...
%CXX% %FLAGS% ^
    -I "%BASE%project7_path_planner\include" ^
    "%BASE%project7_path_planner\src\main.cpp" ^
    -o "%BUILD%\path_planner_demo.exe"
if errorlevel 1 ( echo   FAILED & goto :end )
echo   OK  ->  build\path_planner_demo.exe

:: ── Project 8: MPC Trajectory Controller (Capstone) ────────────────────
echo.
echo [8/9] project8_mpc_controller ...
%CXX% %FLAGS% ^
    -I "%BASE%project8_mpc_controller\include" ^
    -I "%BASE%project7_path_planner\include" ^
    -I "%BASE%project6_lqr_tracker\include" ^
    -I "%BASE%project3_bicycle_model\include" ^
    "%BASE%project8_mpc_controller\src\main.cpp" ^
    -o "%BUILD%\mpc_demo.exe"
if errorlevel 1 ( echo   FAILED & goto :end )
echo   OK  ->  build\mpc_demo.exe
:: ── Project 9: Mini AV Stack (Full Pipeline Integration) ────────────────
echo.
echo [9/9] project9_av_stack ...
%CXX% %FLAGS% ^
    -I "%BASE%project9_av_stack\include" ^
    -I "%BASE%project8_mpc_controller\include" ^
    -I "%BASE%project7_path_planner\include" ^
    -I "%BASE%project6_lqr_tracker\include" ^
    -I "%BASE%project5_kalman_filter\include" ^
    -I "%BASE%project3_bicycle_model\include" ^
    "%BASE%project9_av_stack\src\main.cpp" ^
    -o "%BUILD%\av_stack_demo.exe"
if errorlevel 1 ( echo   FAILED & goto :end )
echo   OK  ->  build\av_stack_demo.exe
echo.
echo ============================================================
echo   Build complete!
echo ============================================================
echo.
echo Run demos (CSVs are written to the build\ folder):
echo   build\integrators_demo.exe
echo   build\pid_demo.exe
echo.
echo Visualise results (run from the build\ folder):
echo   cd build
echo   python ..\project1_integrators\visualize.py
echo   python ..\project2_pid_controller\visualize.py
echo.
echo Unit tests (see project1_integrators\README.md for build steps):
echo   build\integrators_tests.exe

:end
