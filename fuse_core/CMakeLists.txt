cmake_minimum_required(VERSION 3.5)
project(fuse_core)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CXX_STANDARD_REQUIRED YES)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fuse_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

include(boost-extras.cmake)

###########
## Build ##
###########

## fuse_core library
add_library(${PROJECT_NAME} SHARED
  src/async_motion_model.cpp
  src/async_publisher.cpp
  src/async_sensor_model.cpp
  src/callback_wrapper.cpp
  # src/ceres_options.cpp
  src/constraint.cpp
  src/graph.cpp
  src/graph_deserializer.cpp
  src/loss.cpp
  src/serialization.cpp
  src/time.cpp
  src/timestamp_manager.cpp
  src/transaction.cpp
  src/transaction_deserializer.cpp
  src/uuid.cpp
  src/variable.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Boost::serialization
  Ceres::ceres
  Eigen3::Eigen
  ${fuse_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  rclcpp_components::component
)

## fuse_echo executable
add_library(fuse_echo_component SHARED src/fuse_echo.cpp)
target_link_libraries(fuse_echo_component PUBLIC ${PROJECT_NAME})
target_link_libraries(fuse_echo_component PRIVATE rclcpp_components::component)

rclcpp_components_register_node(fuse_echo_component
  PLUGIN "fuse_core::FuseEcho"
  EXECUTABLE fuse_echo
  EXECUTOR SingleThreadedExecutor
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # find_package(ament_cmake_gtest REQUIRED)
  # add_subdirectory(test)
endif()

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
#     test/async_motion_model.test
#     test/test_async_motion_model.cpp
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
#     test/async_publisher.test
#     test/test_async_publisher.cpp
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
#     test/async_sensor_model.test
#     test/test_async_sensor_model.cpp
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
#     test/callback_wrapper.test
#     test/test_callback_wrapper.cpp
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

#   # Constraint tests
#   catkin_add_gtest(test_constraint
#     test/test_constraint.cpp
#   )
#   add_dependencies(test_constraint
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_constraint
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${CMAKE_CURRENT_SOURCE_DIR}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_constraint
#     ${PROJECT_NAME}
#   )
#   set_target_properties(test_constraint
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # Eigen tests
#   catkin_add_gtest(test_eigen
#     test/test_eigen.cpp
#   )
#   add_dependencies(test_eigen
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_eigen
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${CMAKE_CURRENT_SOURCE_DIR}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_eigen
#     ${PROJECT_NAME}
#   )
#   set_target_properties(test_eigen
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # Local Parameterization tests
#   catkin_add_gtest(test_local_parameterization
#     test/test_local_parameterization.cpp
#   )
#   target_include_directories(test_local_parameterization
#     PRIVATE
#       include
#   )
#   target_link_libraries(test_local_parameterization
#     ${PROJECT_NAME}
#   )
#   set_target_properties(test_local_parameterization
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # Loss tests
#   catkin_add_gtest(test_loss
#     test/test_loss.cpp
#   )
#   add_dependencies(test_loss
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_loss
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${CMAKE_CURRENT_SOURCE_DIR}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_loss
#     ${PROJECT_NAME}
#   )
#   set_target_properties(test_loss
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # Message Buffer Tests
#   catkin_add_gtest(test_message_buffer
#     test/test_message_buffer.cpp
#   )
#   add_dependencies(test_message_buffer
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_message_buffer
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_message_buffer
#     ${catkin_LIBRARIES}
#   )
#   set_target_properties(test_message_buffer
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # Timestamp Manager Tests
#   catkin_add_gtest(test_timestamp_manager
#     test/test_timestamp_manager.cpp
#   )
#   add_dependencies(test_timestamp_manager
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_timestamp_manager
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_timestamp_manager
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#   )
#   set_target_properties(test_timestamp_manager
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # Transaction tests
#   catkin_add_gtest(test_transaction
#     test/test_transaction.cpp
#   )
#   add_dependencies(test_transaction
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_transaction
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${CMAKE_CURRENT_SOURCE_DIR}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_transaction
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#   )
#   set_target_properties(test_transaction
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # Parameter tests
#   add_rostest_gtest(test_parameter
#     test/parameter.test
#     test/test_parameter.cpp
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
#     test/throttled_callback.test
#     test/test_throttled_callback.cpp
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

#   # Util tests
#   catkin_add_gtest(test_util
#     test/test_util.cpp
#   )
#   add_dependencies(test_util
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_util
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_util
#     ${catkin_LIBRARIES}
#   )
#   set_target_properties(test_util
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # UUID tests
#   catkin_add_gtest(test_uuid
#     test/test_uuid.cpp
#   )
#   add_dependencies(test_uuid
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_uuid
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_uuid
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#   )
#   set_target_properties(test_uuid
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )

#   # Variable tests
#   catkin_add_gtest(test_variable
#     test/test_variable.cpp
#   )
#   add_dependencies(test_variable
#     ${catkin_EXPORTED_TARGETS}
#   )
#   target_include_directories(test_variable
#     PRIVATE
#       include
#       ${Boost_INCLUDE_DIRS}
#       ${catkin_INCLUDE_DIRS}
#       ${CERES_INCLUDE_DIRS}
#       ${CMAKE_CURRENT_SOURCE_DIR}
#       ${EIGEN3_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_variable
#     ${PROJECT_NAME}
#   )
#   set_target_properties(test_variable
#     PROPERTIES
#       CXX_STANDARD 14
#       CXX_STANDARD_REQUIRED YES
#   )
# endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS fuse_echo_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}-export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  fuse_msgs
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_components
  Ceres
  Eigen3
)

ament_package(CONFIG_EXTRAS boost-extras.cmake)
