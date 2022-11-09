cmake_minimum_required(VERSION 3.0.2)
project(fuse_optimizers)

set(build_depends
  diagnostic_updater
  fuse_constraints
  fuse_core
  fuse_graphs
  fuse_variables
  pluginlib
  roscpp
  std_srvs
)

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
)

find_package(ament_cmake REQUIRED)
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

find_package(Boost REQUIRED COMPONENTS serialization)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########
add_compile_options(-Wall -Werror)

## fuse_optimizers library
add_library(${PROJECT_NAME} SHARED
  src/batch_optimizer.cpp
  src/fixed_lag_smoother.cpp
  src/optimizer.cpp
  src/variable_stamp_index.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
)


## batch_optimizer node
add_executable(batch_optimizer_node
src/batch_optimizer_node.cpp
src/batch_optimizer.cpp
src/optimizer.cpp
src/variable_stamp_index.cpp
)

target_include_directories(batch_optimizer_node PUBLIC
include
)
target_link_libraries(batch_optimizer_node
${PROJECT_NAME}
${Boost_LIBRARIES}
#  ${CERES_LIBRARIES}
#  ${GLOG_LIBRARIES}
)
ament_target_dependencies(batch_optimizer_node
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
)

## fixed_lag_smoother node
add_executable(fixed_lag_smoother_node
src/fixed_lag_smoother_node.cpp
src/fixed_lag_smoother.cpp
src/optimizer.cpp
src/variable_stamp_index.cpp
)
target_include_directories(fixed_lag_smoother_node PUBLIC
include
)
target_link_libraries(fixed_lag_smoother_node
${PROJECT_NAME}
${Boost_LIBRARIES}
${CERES_LIBRARIES}
#  ${GLOG_LIBRARIES}
)
ament_target_dependencies(fixed_lag_smoother_node
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
)



#############
## Install ##
#############

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
  
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)
  find_package(fuse_models REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)

  # Lint tests
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
  roslint_cpp()
  roslint_add_test()

  # VariableStampIndex Tests
  catkin_add_gtest(test_variable_stamp_index
    test/test_variable_stamp_index.cpp
  )
  target_include_directories(test_variable_stamp_index
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_variable_stamp_index
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_variable_stamp_index
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Optimizer Tests
  add_rostest_gtest(test_optimizer
    test/optimizer.test
    test/test_optimizer.cpp
  )
  add_dependencies(test_optimizer
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_optimizer
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  target_link_libraries(test_optimizer
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_optimizer
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  add_rostest(test/optimizer.test ARGS config:=list DEPENDENCIES test_optimizer)

  # Fixed-lag Ignition test
  add_rostest_gtest(test_fixed_lag_ignition
    test/fixed_lag_ignition.test
    test/test_fixed_lag_ignition.cpp
  )
  add_dependencies(test_fixed_lag_ignition
    fixed_lag_smoother_node
  )
  target_include_directories(test_fixed_lag_ignition
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${fuse_models_INCLUDE_DIRS}
      ${geometry_msgs_INCLUDE_DIRS}
      ${nav_msgs_INCLUDE_DIRS}
      ${rostest_INCLUDE_DIRS}
  )
  target_link_libraries(test_fixed_lag_ignition
    ${catkin_LIBRARIES}
    ${fuse_models_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${rostest_LIBRARIES}
  )
  set_target_properties(test_fixed_lag_ignition
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )
endif()

ament_package(
  CONFIG_EXTRAS
)
