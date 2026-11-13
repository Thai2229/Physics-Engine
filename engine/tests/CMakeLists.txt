include(GoogleTest)

add_executable(physics_tests
  math_test.cpp
  shape_test.cpp
  body_test.cpp
  world_test.cpp
  joint_test.cpp
  collision_test.cpp
  contact_test.cpp
)
target_link_libraries(physics_tests PRIVATE physics physics_warnings GTest::gtest_main)
gtest_discover_tests(physics_tests)
