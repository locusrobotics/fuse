cmake_minimum_required(VERSION 3.0.2)
project(fuse_models)

find_package(
  catkin REQUIRED
  COMPONENTS
    fuse_constraints
    fuse_core
    fuse_graphs
    fuse_msgs
    fuse_publishers
    fuse_variables
    geometry_msgs
    message_generation
    nav_msgs
    pluginlib
    roscpp
    roslint
    sensor_msgs
    std_srvs
    tf2
    tf2_2d
    tf2_geometry_msgs
    tf2_ros)

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

generate_messages(
  DEPENDENCIES
    geometry_msgs
    fuse_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    fuse_constraints
    fuse_core
    fuse_graphs
    fuse_msgs
    fuse_publishers
    fuse_variables
    geometry_msgs
    message_runtime
    nav_msgs
    pluginlib
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_2d
    tf2_geometry_msgs
    tf2_ros
  DEPENDS
    Boost
    CERES
    EIGEN3
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
roslint_cpp()

###########
## Build ##
###########
add_compile_options(-Wall -Werror)

## Declare a C++ library
add_library(${PROJECT_NAME}
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
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  FILES fuse_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
