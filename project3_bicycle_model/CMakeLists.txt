project(project3_bicycle_model)

# ── Demo executable ────────────────────────────────────────────────────────────
add_executable(bicycle_demo src/main.cpp)
target_include_directories(bicycle_demo PRIVATE include)

# ── Unit tests ─────────────────────────────────────────────────────────────────
add_executable(bicycle_tests tests/test_bicycle_model.cpp)
target_include_directories(bicycle_tests PRIVATE include)
target_link_libraries(bicycle_tests PRIVATE GTest::gtest_main)

add_test(NAME bicycle_tests COMMAND bicycle_tests)
