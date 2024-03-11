cmake_minimum_required(VERSION 3.16)
project(fuse_optimizers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CXX_STANDARD_REQUIRED YES)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)

find_package(fuse_constraints REQUIRED)
find_package(fuse_core REQUIRED)
find_package(fuse_msgs REQUIRED)
find_package(fuse_graphs REQUIRED)
find_package(fuse_variables REQUIRED)

find_package(std_srvs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

## fuse_optimizers library
add_library(${PROJECT_NAME}
  src/batch_optimizer.cpp
  src/fixed_lag_smoother.cpp
  src/optimizer.cpp
  src/variable_stamp_index.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME}
  fuse_constraints::fuse_constraints
  fuse_core::fuse_core
  fuse_graphs::fuse_graphs
  ${fuse_msgs_TARGETS}
  fuse_variables::fuse_variables
  pluginlib::pluginlib
  ${std_srvs_TARGETS}
)
ament_target_dependencies(${PROJECT_NAME} diagnostic_updater)

## batch_optimizer node
add_executable(batch_optimizer_node src/batch_optimizer_node.cpp)
target_link_libraries(batch_optimizer_node ${PROJECT_NAME})

## fixed_lag_smoother node
add_executable(fixed_lag_smoother_node src/fixed_lag_smoother_node.cpp)
target_link_libraries(fixed_lag_smoother_node ${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  batch_optimizer_node
  fixed_lag_smoother_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}-export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake_ros
  rclcpp
  fuse_constraints
  fuse_core
  fuse_msgs
  fuse_graphs
  fuse_variables
  std_srvs
  diagnostic_updater
  pluginlib
  rclcpp_components

  Ceres
  Eigen3
)

ament_package()
