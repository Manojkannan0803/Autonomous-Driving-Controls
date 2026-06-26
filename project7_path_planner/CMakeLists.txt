cmake_minimum_required(VERSION 3.16)

set(P7_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(path_planner_demo src/main.cpp)
target_include_directories(path_planner_demo PRIVATE ${P7_INCLUDE})
target_compile_features(path_planner_demo PRIVATE cxx_std_20)

if(TARGET GTest::gtest)
    add_executable(path_planner_tests tests/test_path_planner.cpp)
    target_include_directories(path_planner_tests PRIVATE ${P7_INCLUDE})
    target_link_libraries(path_planner_tests GTest::gtest GTest::gtest_main)
    target_compile_features(path_planner_tests PRIVATE cxx_std_20)
    add_test(NAME path_planner_tests COMMAND path_planner_tests)
endif()
