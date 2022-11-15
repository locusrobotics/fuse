# GTESTS ===========================================================================================
ament_add_gtest(test_constraint test_constraint.cpp)
target_link_libraries(test_constraint ${PROJECT_NAME})

ament_add_gtest(test_eigen test_eigen.cpp)
target_link_libraries(test_eigen ${PROJECT_NAME})

# TODO(sloretz) fails to build: can't include ros/node_handle.h
# ament_add_gtest(test_local_parameterization test_local_parameterization.cpp)
# target_link_libraries(test_local_parameterization ${PROJECT_NAME})

ament_add_gtest(test_loss test_loss.cpp)
target_link_libraries(test_loss ${PROJECT_NAME})

ament_add_gtest(test_message_buffer test_message_buffer.cpp)
target_link_libraries(test_message_buffer ${PROJECT_NAME})

ament_add_gtest(test_timestamp_manager test_timestamp_manager.cpp)
target_link_libraries(test_timestamp_manager ${PROJECT_NAME})

ament_add_gtest(test_transaction test_transaction.cpp)
target_link_libraries(test_transaction ${PROJECT_NAME})

# TODO(sloretz) fails to build: can't include ros/node_handle.h
# ament_add_gtest(test_util test_util.cpp)
# target_link_libraries(test_util ${PROJECT_NAME})

ament_add_gtest(test_uuid test_uuid.cpp)
target_link_libraries(test_uuid ${PROJECT_NAME})

ament_add_gtest(test_variable test_variable.cpp)
target_link_libraries(test_variable ${PROJECT_NAME})


# LAUNCH TESTS =====================================================================================
# TODO(CH3): Move this to the test directory and port to ROS 2
# if(CATKIN_ENABLE_TESTING)
#   find_package(roslint REQUIRED)
#   find_package(rostest REQUIRED)

#   # Lint tests
#   set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
#   roslint_cpp()
#   roslint_add_test()

#   # AsyncMotionModel tests
#   add_rostest_gtest(test_async_motion_model
#     async_motion_model.test
#     test_async_motion_model.cpp
#   )
#   add_dependencies(test_async_motion_model
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_async_motion_model
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_async_motion_model
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#   )
#   set_target_properties(test_async_motion_model
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # AsyncPublisher tests
#   add_rostest_gtest(test_async_publisher
#     async_publisher.test
#     test_async_publisher.cpp
#   )
#   add_dependencies(test_async_publisher
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_async_publisher
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_async_publisher
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#   )
#   set_target_properties(test_async_publisher
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # AsyncSensorModel tests
#   add_rostest_gtest(test_async_sensor_model
#     async_sensor_model.test
#     test_async_sensor_model.cpp
#   )
#   add_dependencies(test_async_sensor_model
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_async_sensor_model
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_async_sensor_model
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#   )
#   set_target_properties(test_async_sensor_model
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # CallbackWrapper tests
#   add_rostest_gtest(test_callback_wrapper
#     callback_wrapper.test
#     test_callback_wrapper.cpp
#   )
#   add_dependencies(test_callback_wrapper
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_callback_wrapper
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_callback_wrapper
#     ${catkin_LIBRARIES}
#   )
#   set_target_properties(test_callback_wrapper
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # Parameter tests
#   add_rostest_gtest(test_parameter
#     parameter.test
#     test_parameter.cpp
#   )
#   add_dependencies(test_parameter
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_parameter
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_parameter
#     ${catkin_LIBRARIES}
#   )
#   set_target_properties(test_parameter
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # Throttle callback test
#   add_rostest_gtest(
#     test_throttled_callback
#     throttled_callback.test
#     test_throttled_callback.cpp
#   )
#   target_link_libraries(test_throttled_callback
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#   )
#   set_target_properties(test_throttled_callback
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )
# endif()
