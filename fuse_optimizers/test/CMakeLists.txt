# CORE GTESTS ======================================================================================
set(TEST_TARGETS
  # test_fixed_lag_ignition
  # test_optimizer
  test_variable_stamp_index
)

foreach(test_name ${TEST_TARGETS})
  ament_add_gtest("${test_name}" "${test_name}.cpp")
  target_link_libraries("${test_name}" ${PROJECT_NAME})
endforeach()


# ROS TESTS (WITH LAUNCH) ==========================================================================
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest_executable(test_fixed_lag_ignition launch_tests/test_fixed_lag_ignition.cpp)
target_link_libraries(test_fixed_lag_ignition ${PROJECT_NAME} ${nav_msgs_TARGETS})

ament_add_pytest_test(fixed_lag_ignition_py "launch_tests/test_fixed_lag_ignition.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

configure_file(
  "launch_tests/config/common_robot_config.yaml"
  "launch_tests/config/common_robot_config.yaml" COPYONLY)
configure_file(
  "launch_tests/config/fixed_lag_params.yaml"
  "launch_tests/config/fixed_lag_params.yaml" COPYONLY)
configure_file(
  "launch_tests/config/noisy_motion_model_config.yaml"
  "launch_tests/config/noisy_motion_model_config.yaml" COPYONLY)
configure_file(
  "launch_tests/config/robot_with_imu_config.yaml"
  "launch_tests/config/robot_with_imu_config.yaml" COPYONLY)
configure_file(
  "launch_tests/config/serialized_publisher_config.yaml"
  "launch_tests/config/serialized_publisher_config.yaml" COPYONLY)
