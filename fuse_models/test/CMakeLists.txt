# CORE GTESTS ======================================================================================
set(TEST_TARGETS
  test_unicycle_2d
  test_unicycle_2d_predict
  test_unicycle_2d_state_cost_function
  test_graph_ignition
  test_unicycle_2d_ignition
  test_omnidirectional_3d
  test_omnidirectional_3d_predict
  test_omnidirectional_3d_state_cost_function
)

foreach(test_name ${TEST_TARGETS})
  ament_add_gtest("${test_name}" "${test_name}.cpp")
  target_link_libraries("${test_name}" ${PROJECT_NAME})
endforeach()

ament_add_gmock(test_sensor_proc "test_sensor_proc.cpp")
target_link_libraries(test_sensor_proc ${PROJECT_NAME})

ament_add_gmock(test_sensor_proc_3d "test_sensor_proc_3d.cpp")
target_link_libraries(test_sensor_proc_3d ${PROJECT_NAME})