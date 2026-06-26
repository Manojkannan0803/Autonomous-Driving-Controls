cmake_minimum_required(VERSION 3.16)

set(P3_INCLUDE ${CMAKE_SOURCE_DIR}/project3_bicycle_model/include)
set(P5_INCLUDE ${CMAKE_SOURCE_DIR}/project5_kalman_filter/include)
set(P6_INCLUDE ${CMAKE_SOURCE_DIR}/project6_lqr_tracker/include)
set(P7_INCLUDE ${CMAKE_SOURCE_DIR}/project7_path_planner/include)
set(P8_INCLUDE ${CMAKE_SOURCE_DIR}/project8_mpc_controller/include)
set(P9_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(av_stack_demo src/main.cpp)
target_include_directories(av_stack_demo PRIVATE
    ${P9_INCLUDE} ${P8_INCLUDE} ${P7_INCLUDE} ${P6_INCLUDE}
    ${P5_INCLUDE} ${P3_INCLUDE})
target_compile_features(av_stack_demo PRIVATE cxx_std_20)

if(TARGET GTest::gtest)
    add_executable(av_stack_tests tests/test_av_stack.cpp)
    target_include_directories(av_stack_tests PRIVATE
        ${P9_INCLUDE} ${P8_INCLUDE} ${P7_INCLUDE} ${P6_INCLUDE}
        ${P5_INCLUDE} ${P3_INCLUDE} ${GTEST_INCLUDE_DIRS})
    target_link_libraries(av_stack_tests GTest::gtest GTest::gtest_main)
    target_compile_features(av_stack_tests PRIVATE cxx_std_20)
    add_test(NAME av_stack_tests COMMAND av_stack_tests)
endif()
