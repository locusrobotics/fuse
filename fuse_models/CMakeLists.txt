cmake_minimum_required(VERSION 3.16)
project(fuse_models)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CXX_STANDARD_REQUIRED YES)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(covariance_geometry_ros REQUIRED)
find_package(fuse_constraints REQUIRED)
find_package(fuse_core REQUIRED)
find_package(fuse_graphs REQUIRED)
find_package(fuse_msgs REQUIRED)
find_package(fuse_publishers REQUIRED)
find_package(fuse_variables REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_2d REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/acceleration_2d.cpp
  src/graph_ignition.cpp
  src/imu_2d.cpp
  src/imu_3d.cpp
  src/odometry_2d.cpp
  src/odometry_2d_publisher.cpp
  src/odometry_3d.cpp
  src/odometry_3d_publisher.cpp
  src/pose_2d.cpp
  src/transaction.cpp
  src/twist_2d.cpp
  src/unicycle_2d.cpp
  src/unicycle_2d_ignition.cpp
  src/unicycle_2d_state_kinematic_constraint.cpp
  src/omnidirectional_3d.cpp
  src/omnidirectional_3d_ignition.cpp
  src/omnidirectional_3d_state_kinematic_constraint.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME}
  covariance_geometry_ros::covariance_geometry_ros
  Ceres::ceres
  fuse_constraints::fuse_constraints
  fuse_core::fuse_core
  fuse_graphs::fuse_graphs
  ${fuse_msgs_TARGETS}
  fuse_publishers::fuse_publishers
  fuse_variables::fuse_variables
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp_components::component
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_2d::tf2_2d
  ${tf2_geometry_msgs_TARGETS}
  tf2_ros::tf2_ros
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # This is because these two files have unsuitable copyrights
  set(_linter_excludes
    test/test_sensor_proc.cpp
    test/test_unicycle_2d.cpp
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${_linter_excludes})
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
  add_subdirectory(benchmark)
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(fuse_core fuse_plugins.xml)

ament_export_targets(${PROJECT_NAME}-export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake_ros

  covariance_geometry_ros
  fuse_constraints
  fuse_core
  fuse_graphs
  fuse_msgs
  fuse_publishers
  fuse_variables
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp_components
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_2d
  tf2_geometry_msgs
  tf2_ros

  Ceres
  Eigen3
  Boost
)

ament_package()
