# CORE GTESTS ======================================================================================
set(TEST_TARGETS
  test_absolute_constraint
  test_absolute_orientation_2d_stamped_constraint
  test_absolute_orientation_3d_stamped_constraint
  test_absolute_orientation_3d_stamped_euler_constraint
  test_absolute_pose_2d_stamped_constraint
  test_absolute_pose_3d_stamped_constraint
  test_absolute_pose_3d_stamped_euler_constraint
  test_marginal_constraint
  test_marginalize_variables
  test_normal_delta_pose_2d
  test_normal_prior_pose_2d
  test_normal_prior_pose_3d
  test_normal_prior_pose_3d_euler
  test_relative_constraint
  test_relative_pose_2d_stamped_constraint
  test_relative_pose_3d_stamped_constraint
  test_relative_pose_3d_stamped_euler_constraint
  test_uuid_ordering
  test_variable_constraints
)

foreach(test_name ${TEST_TARGETS})
  ament_add_gtest("${test_name}" "${test_name}.cpp" WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries("${test_name}" ${PROJECT_NAME})
endforeach()
