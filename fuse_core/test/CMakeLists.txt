# CORE GTESTS ======================================================================================
ament_add_gtest(test_constraint test_constraint.cpp)
target_link_libraries(test_constraint ${PROJECT_NAME})

ament_add_gtest(test_eigen test_eigen.cpp)
target_link_libraries(test_eigen ${PROJECT_NAME})

ament_add_gtest(test_local_parameterization test_local_parameterization.cpp)
target_link_libraries(test_local_parameterization ${PROJECT_NAME})

ament_add_gtest(test_loss test_loss.cpp)
target_link_libraries(test_loss ${PROJECT_NAME})

ament_add_gtest(test_message_buffer test_message_buffer.cpp)
target_link_libraries(test_message_buffer ${PROJECT_NAME})

ament_add_gtest(test_parameter test_parameter.cpp)
target_link_libraries(test_parameter ${PROJECT_NAME})

ament_add_gtest(test_timestamp_manager test_timestamp_manager.cpp)
target_link_libraries(test_timestamp_manager ${PROJECT_NAME})

ament_add_gtest(test_transaction test_transaction.cpp)
target_link_libraries(test_transaction ${PROJECT_NAME})

ament_add_gtest(test_util test_util.cpp)
target_link_libraries(test_util ${PROJECT_NAME})

ament_add_gtest(test_uuid test_uuid.cpp)
target_link_libraries(test_uuid ${PROJECT_NAME})

ament_add_gtest(test_variable test_variable.cpp)
target_link_libraries(test_variable ${PROJECT_NAME})


# ROS TESTS (NO LAUNCH) ============================================================================
ament_add_gtest(test_async_motion_model test_async_motion_model.cpp)
target_link_libraries(test_async_motion_model ${PROJECT_NAME})

ament_add_gtest(test_async_publisher test_async_publisher.cpp)
target_link_libraries(test_async_publisher ${PROJECT_NAME})

ament_add_gtest(test_async_sensor_model test_async_sensor_model.cpp)
target_link_libraries(test_async_sensor_model ${PROJECT_NAME})

ament_add_gtest(test_callback_wrapper test_callback_wrapper.cpp)
target_link_libraries(test_callback_wrapper ${PROJECT_NAME})

find_package(geometry_msgs REQUIRED)
ament_add_gtest(test_throttled_callback test_throttled_callback.cpp)
target_link_libraries(test_throttled_callback ${PROJECT_NAME} ${geometry_msgs_TARGETS})


# ROS TESTS (WITH LAUNCH) ==========================================================================
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest_executable(test_parameters launch_tests/test_parameters.cpp)
target_link_libraries(test_parameters ${PROJECT_NAME})

ament_add_pytest_test(test_parameters_py "launch_tests/test_parameters.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

configure_file("launch_tests/test_parameters.yaml" "launch_tests/test_parameters.yaml" COPYONLY)
