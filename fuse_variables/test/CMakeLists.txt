# CORE GTESTS ======================================================================================
set(TEST_TARGETS
  test_acceleration_angular_2d_stamped
  test_acceleration_angular_3d_stamped
  test_acceleration_linear_2d_stamped
  test_acceleration_linear_3d_stamped
  test_fixed_size_variable
  test_orientation_2d_stamped
  test_orientation_3d_stamped
  test_point_2d_fixed_landmark
  test_point_2d_landmark
  test_point_3d_fixed_landmark
  test_point_3d_landmark
  test_position_2d_stamped
  test_position_3d_stamped
  test_velocity_angular_2d_stamped
  test_velocity_angular_3d_stamped
  test_velocity_linear_2d_stamped
  test_velocity_linear_3d_stamped
)

foreach(test_name ${TEST_TARGETS})
  ament_add_gtest("${test_name}" "${test_name}.cpp")
  target_link_libraries("${test_name}" ${PROJECT_NAME})
endforeach()


# ROS TESTS (WITH LAUNCH) ==========================================================================
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest_executable(test_load_device_id launch_tests/test_load_device_id.cpp)
target_link_libraries(test_load_device_id ${PROJECT_NAME})

ament_add_pytest_test(test_load_device_id_py "launch_tests/test_load_device_id.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

configure_file(
  "launch_tests/test_load_device_id.yaml"
  "launch_tests/test_load_device_id.yaml"
  COPYONLY
)
