cmake_minimum_required(VERSION 3.16)
project(fuse_tutorials)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CXX_STANDARD_REQUIRED YES)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(fuse_constraints REQUIRED)
find_package(fuse_core REQUIRED)
find_package(fuse_models REQUIRED)
find_package(fuse_variables REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

###########
## Build ##
###########

## fuse_tutorial library
add_library(${PROJECT_NAME}
  src/beacon_publisher.cpp
  src/range_constraint.cpp
  src/range_sensor_model.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  fuse_constraints::fuse_constraints
  fuse_core::fuse_core
  fuse_models::fuse_models
  fuse_variables::fuse_variables
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# tutorial_sim executable
add_executable(range_sensor_simulator src/range_sensor_simulator.cpp)
target_link_libraries(range_sensor_simulator ${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
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

install(TARGETS
  TARGETS range_sensor_simulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(fuse_core fuse_plugins.xml)

ament_export_targets(${PROJECT_NAME}-export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake_ros
  fuse_constraints
  fuse_core
  fuse_models
  fuse_variables
  nav_msgs
  rclcpp
  sensor_msgs
)

ament_package()
