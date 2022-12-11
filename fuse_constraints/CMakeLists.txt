cmake_minimum_required(VERSION 3.16)
project(fuse_constraints)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CXX_STANDARD_REQUIRED YES)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(fuse_core REQUIRED)
find_package(fuse_graphs REQUIRED)
find_package(fuse_variables REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(suitesparse-extras.cmake)

###########
## Build ##
###########

# fuse_constraints library
add_library(${PROJECT_NAME}
  src/absolute_constraint.cpp
  src/absolute_orientation_3d_stamped_constraint.cpp
  src/absolute_orientation_3d_stamped_euler_constraint.cpp
  src/absolute_pose_2d_stamped_constraint.cpp
  src/absolute_pose_3d_stamped_constraint.cpp
  src/marginal_constraint.cpp
  src/marginal_cost_function.cpp
  src/marginalize_variables.cpp
  src/normal_delta.cpp
  src/normal_delta_orientation_2d.cpp
  src/normal_delta_pose_2d.cpp
  src/normal_prior_orientation_2d.cpp
  src/normal_prior_pose_2d.cpp
  src/relative_constraint.cpp
  src/relative_orientation_3d_stamped_constraint.cpp
  src/relative_pose_2d_stamped_constraint.cpp
  src/relative_pose_3d_stamped_constraint.cpp
  src/uuid_ordering.cpp
  src/variable_constraints.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Boost::serialization
  Ceres::ceres
  Eigen3::Eigen
  fuse_core::fuse_core
  fuse_graphs::fuse_graphs
  fuse_variables::fuse_variables
  ${geometry_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${SUITESPARSE_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${SUITESPARSE_INCLUDE_DIRS}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
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

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(fuse_core fuse_plugins.xml)

ament_export_targets(${PROJECT_NAME}-export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake_ros
  fuse_core
  fuse_graphs
  fuse_variables
  geometry_msgs
  pluginlib
  rclcpp

  Ceres
  Eigen3
  glog
)

ament_package(CONFIG_EXTRAS suitesparse-extras.cmake)
