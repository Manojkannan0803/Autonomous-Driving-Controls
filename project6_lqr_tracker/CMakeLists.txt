cmake_minimum_required(VERSION 3.16)
project(project6_lqr_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(P3_INCLUDE "${CMAKE_SOURCE_DIR}/project3_bicycle_model/include")

add_executable(lqr_tracker_demo src/main.cpp)
target_include_directories(lqr_tracker_demo PRIVATE include ${P3_INCLUDE})

add_executable(lqr_tracker_tests tests/test_lqr_tracker.cpp)
target_include_directories(lqr_tracker_tests PRIVATE include ${P3_INCLUDE})
target_link_libraries(lqr_tracker_tests PRIVATE GTest::gtest_main)
