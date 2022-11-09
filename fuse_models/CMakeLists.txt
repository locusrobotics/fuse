cmake_minimum_required(VERSION 3.5)
project(fuse_models)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(fuse_constraints REQUIRED)
find_package(fuse_core REQUIRED)
find_package(fuse_graphs REQUIRED)
find_package(fuse_msgs REQUIRED)
find_package(fuse_publishers REQUIRED)
find_package(fuse_variables REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_generation REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roslint REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_2d REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# Generate messages
add_service_files(
  DIRECTORY
    srv
  FILES
    SetGraph.srv
    SetPose.srv
    SetPoseDeprecated.srv
)


## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)


#set(ROSLINT_CPP_OPTS "--filter=-build/c++17,-runtime/references")
#roslint_cpp()

###########
## Build ##
###########

include_directories(include)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/acceleration_2d.cpp
  src/graph_ignition.cpp
  src/imu_2d.cpp
  src/odometry_2d.cpp
  src/odometry_2d_publisher.cpp
  src/pose_2d.cpp
  src/transaction.cpp
  src/twist_2d.cpp
  src/unicycle_2d.cpp
  src/unicycle_2d_ignition.cpp
  src/unicycle_2d_state_kinematic_constraint.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${CERES_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  fuse_constraints
  fuse_core
  fuse_graphs
  fuse_msgs
  fuse_publishers
  fuse_variables
  geometry_msgs
  message_generation
  nav_msgs
  rcl_components
  rclcpp
  roslint
  sensor_msgs
  std_srvs
  tf2
  tf2_2d
  tf2_geometry_msgs
  tf2_ros
)


#############
## Install ##
#############


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

## Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION DESTINATION lib
  LIBRARY DESTINATION DESTINATION lib
  RUNTIME DESTINATION DESTINATION bin
)

## Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h"
)

install(
  FILES fuse_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Lint tests
  roslint_add_test()

  # Model tests
  catkin_add_gtest(test_unicycle_2d
    test/test_unicycle_2d.cpp
  )
  target_link_libraries(test_unicycle_2d
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_unicycle_2d
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Prediction tests
  catkin_add_gtest(
    test_unicycle_2d_predict
    test/test_unicycle_2d_predict.cpp
  )
  target_link_libraries(test_unicycle_2d_predict
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_unicycle_2d_predict
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Cost function tests
  catkin_add_gtest(
    test_unicycle_2d_state_cost_function
    test/test_unicycle_2d_state_cost_function.cpp
  )
  target_link_libraries(
    test_unicycle_2d_state_cost_function
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_unicycle_2d_state_cost_function
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Graph Ignition tests
  add_rostest_gtest(
    test_graph_ignition
    test/graph_ignition.test
    test/test_graph_ignition.cpp
  )
  add_dependencies(test_graph_ignition
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_graph_ignition
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  target_link_libraries(test_graph_ignition
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_graph_ignition
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Unicycle2D Ignition tests
  add_rostest_gtest(
    test_unicycle_2d_ignition
    test/unicycle_2d_ignition.test
    test/test_unicycle_2d_ignition.cpp
  )
  target_link_libraries(test_unicycle_2d_ignition
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_unicycle_2d_ignition
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Other tests
  catkin_add_gmock(
    test_sensor_proc
    test/test_sensor_proc.cpp
  )
  target_link_libraries(test_sensor_proc
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_sensor_proc
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Benchmarks
  find_package(benchmark QUIET)

  if(benchmark_FOUND)
    add_executable(benchmark_unicycle_2d_state_cost_function
      benchmark/benchmark_unicycle_2d_state_cost_function.cpp
    )
    if(TARGET benchmark_unicycle_2d_state_cost_function)
      target_link_libraries(
        benchmark_unicycle_2d_state_cost_function
        benchmark
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
      )
      set_target_properties(benchmark_unicycle_2d_state_cost_function
        PROPERTIES
          CXX_STANDARD 14
          CXX_STANDARD_REQUIRED YES
      )
    endif()
  endif()
endif()

ament_package(
  CONFIG_EXTRAS
)
