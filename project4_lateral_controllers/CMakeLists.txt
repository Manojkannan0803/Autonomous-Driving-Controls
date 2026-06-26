project(project4_lateral_controllers)

# Project 3 headers are reused directly (bicycle model + reference path)
set(P3_INCLUDE "${CMAKE_SOURCE_DIR}/project3_bicycle_model/include")

# ── Demo / benchmark executable ───────────────────────────────────────────────
add_executable(lateral_demo src/main.cpp)
target_include_directories(lateral_demo PRIVATE include ${P3_INCLUDE})

# ── Unit tests ─────────────────────────────────────────────────────────────────
add_executable(lateral_tests tests/test_controllers.cpp)
target_include_directories(lateral_tests PRIVATE include ${P3_INCLUDE})
target_link_libraries(lateral_tests PRIVATE GTest::gtest_main)

add_test(NAME lateral_tests COMMAND lateral_tests)
