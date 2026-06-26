cmake_minimum_required(VERSION 3.16)
project(project5_kalman_filter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(P3_INCLUDE "${CMAKE_SOURCE_DIR}/project3_bicycle_model/include")

add_executable(kalman_demo src/main.cpp)
target_include_directories(kalman_demo PRIVATE include ${P3_INCLUDE})

add_executable(kalman_tests tests/test_kalman.cpp)
target_include_directories(kalman_tests PRIVATE include ${P3_INCLUDE})
target_link_libraries(kalman_tests PRIVATE GTest::gtest_main)
