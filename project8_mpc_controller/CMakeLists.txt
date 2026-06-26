cmake_minimum_required(VERSION 3.16)

set(P3_INCLUDE ${CMAKE_SOURCE_DIR}/project3_bicycle_model/include)
set(P6_INCLUDE ${CMAKE_SOURCE_DIR}/project6_lqr_tracker/include)
set(P7_INCLUDE ${CMAKE_SOURCE_DIR}/project7_path_planner/include)
set(P8_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(mpc_demo src/main.cpp)
target_include_directories(mpc_demo PRIVATE
    ${P8_INCLUDE} ${P7_INCLUDE} ${P6_INCLUDE} ${P3_INCLUDE})
target_compile_features(mpc_demo PRIVATE cxx_std_20)

if(TARGET GTest::gtest)
    add_executable(mpc_tests tests/test_mpc_tracker.cpp)
    target_include_directories(mpc_tests PRIVATE
        ${P8_INCLUDE} ${P6_INCLUDE} ${P3_INCLUDE} ${GTEST_INCLUDE_DIRS})
    target_link_libraries(mpc_tests GTest::gtest GTest::gtest_main)
    target_compile_features(mpc_tests PRIVATE cxx_std_20)
    add_test(NAME mpc_tests COMMAND mpc_tests)
endif()
