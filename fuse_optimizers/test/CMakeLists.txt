# CORE GTESTS ======================================================================================
ament_add_gtest(test_variable_stamp_index "test_variable_stamp_index.cpp")
target_link_libraries(test_variable_stamp_index ${PROJECT_NAME})


# ROS TESTS (WITH LAUNCH) ==========================================================================
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest_executable(test_fixed_lag_ignition launch_tests/test_fixed_lag_ignition.cpp)
target_link_libraries(test_fixed_lag_ignition ${PROJECT_NAME} ${nav_msgs_TARGETS})

ament_add_pytest_test(
  test_fixed_lag_ignition_py
  "launch_tests/test_fixed_lag_ignition.py"
  WORKING_DIRECTORY
  "${CMAKE_CURRENT_BINARY_DIR}"
)

ament_add_gtest_executable(test_optimizer launch_tests/test_optimizer.cpp)
target_link_libraries(test_optimizer ${PROJECT_NAME} ${nav_msgs_TARGETS})

ament_add_pytest_test(
  test_optimizer_py
  "launch_tests/test_optimizer.py"
  WORKING_DIRECTORY
  "${CMAKE_CURRENT_BINARY_DIR}"
)

ament_add_gtest_executable(test_fixed_lag_smoother launch_tests/test_fixed_lag_smoother.cpp)
target_link_libraries(test_fixed_lag_smoother ${PROJECT_NAME} ${nav_msgs_TARGETS})

ament_add_pytest_test(
  test_fixed_lag_smoother_py
  "launch_tests/test_fixed_lag_smoother.py"
  WORKING_DIRECTORY
  "${CMAKE_CURRENT_BINARY_DIR}"
)

configure_file(
  "launch_tests/config/optimizer_params.yaml"
  "launch_tests/config/optimizer_params.yaml" COPYONLY)
configure_file(
  "launch_tests/config/fixed_lag_ignition_params.yaml"
  "launch_tests/config/fixed_lag_ignition_params.yaml" COPYONLY)
