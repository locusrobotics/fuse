cmake_minimum_required(VERSION 3.16)
project(fuse_core)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CXX_STANDARD_REQUIRED YES)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
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
add_library(${PROJECT_NAME}
  src/async_motion_model.cpp
  src/async_publisher.cpp
  src/async_sensor_model.cpp
  src/callback_wrapper.cpp
  src/ceres_options.cpp
  src/constraint.cpp
  src/graph.cpp
  src/graph_deserializer.cpp
  src/loss.cpp
  src/parameter.cpp
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
  ament_cmake_ros
  fuse_msgs
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_components
  Ceres
  Eigen3
)

ament_package(CONFIG_EXTRAS boost-extras.cmake)
