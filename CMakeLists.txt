cmake_minimum_required(VERSION 3.16)
project(AV_Controls_Portfolio VERSION 1.0 LANGUAGES CXX)

# ── Language standard ──────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Build type default ─────────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ── Compiler warnings ──────────────────────────────────────────────────────────
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
elseif(MSVC)
    add_compile_options(/W4 /WX-)
endif()

# ── GoogleTest (fetched automatically at configure time) ───────────────────────
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ── Tests ──────────────────────────────────────────────────────────────────────
enable_testing()

# ── Sub-projects ──────────────────────────────────────────────────────────────
# Phase 1 — Foundations
add_subdirectory(project1_integrators)
add_subdirectory(project2_pid_controller)
# Phase 2 — Vehicle Modeling + Classical Control
add_subdirectory(project3_bicycle_model)
add_subdirectory(project4_lateral_controllers)
# Phase 3 — State Estimation + Optimal Control
add_subdirectory(project5_kalman_filter)
add_subdirectory(project6_lqr_tracker)
# Phase 4 — Planning
add_subdirectory(project7_path_planner)
# Phase 4 cont. — Optimal Control (Capstone)
add_subdirectory(project8_mpc_controller)
add_subdirectory(project9_av_stack)
