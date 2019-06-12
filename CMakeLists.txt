cmake_minimum_required(VERSION 2.8.3)
project(fuse_rl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-Wall -Werror)

find_package(
  catkin REQUIRED
  COMPONENTS
    fuse_constraints
    fuse_core
    fuse_graphs
    fuse_publishers
    fuse_variables
    pluginlib
    roscpp
    roslint
    tf2_2d
    tf2_geometry_msgs
    tf2_ros)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    fuse_constraints
    fuse_core
    fuse_graphs
    fuse_publishers
    fuse_variables
    pluginlib
    roscpp
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

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  ${PROJECT_NAME}
    src/acceleration_2d/model.cpp
    src/imu_2d/model.cpp
    src/pose_2d/model.cpp
    src/odometry_2d/model.cpp
    src/odometry_2d/publisher.cpp
    src/twist_2d/model.cpp
    src/unicycle_2d/state_kinematic_constraint.cpp
    src/unicycle_2d/model.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
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
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  # Lint tests
  roslint_add_test()

  # Model tests
  catkin_add_gtest(
    test_unicycle_2d_model
    test/test_unicycle_2d_model.cpp
  )
  if(TARGET test_unicycle_2d_model)
    target_link_libraries(
      test_unicycle_2d_model
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${CERES_LIBRARIES}
    )
  endif()

  # Prediction tests
  catkin_add_gtest(
    test_unicycle_2d_predict
    test/test_unicycle_2d_predict.cpp
  )
  if(TARGET test_unicycle_2d_model)
    target_link_libraries(
      test_unicycle_2d_predict
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${CERES_LIBRARIES}
    )
  endif()
endif()
