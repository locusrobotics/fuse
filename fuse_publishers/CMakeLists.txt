cmake_minimum_required(VERSION 2.8.3)
project(fuse_publishers)

find_package(catkin REQUIRED COMPONENTS
  fuse_core
  fuse_variables
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    fuse_core
    fuse_variables
    geometry_msgs
    nav_msgs
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)

###########
## Build ##
###########

add_compile_options(-std=c++14 -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# fuse_publishers library
add_library(${PROJECT_NAME}
  src/path_2d_publisher.cpp
  src/pose_2d_publisher.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES fuse_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(fuse_constraints REQUIRED)
  find_package(fuse_graphs REQUIRED)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  # Lint tests
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
  roslint_cpp()
  roslint_add_test()

  # Path2DPublisher Tests
  add_rostest_gtest(test_path_2d_publisher
    test/path_2d_publisher.test
    test/test_path_2d_publisher.cpp
  )
  add_dependencies(test_path_2d_publisher
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_path_2d_publisher
    PUBLIC
      ${fuse_constraints_INCLUDE_DIRS}
      ${fuse_graphs_INCLUDE_DIRS}
  )
  target_link_libraries(test_path_2d_publisher
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${fuse_constraints_LIBRARIES}
    ${fuse_graphs_LIBRARIES}
  )

  # Pose2DPublisher Tests
  add_rostest_gtest(test_pose_2d_publisher
    test/pose_2d_publisher.test
    test/test_pose_2d_publisher.cpp
  )
  add_dependencies(test_pose_2d_publisher
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_pose_2d_publisher
    PUBLIC
      ${fuse_constraints_INCLUDE_DIRS}
      ${fuse_graphs_INCLUDE_DIRS}
  )
  target_link_libraries(test_pose_2d_publisher
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${fuse_constraints_LIBRARIES}
    ${fuse_graphs_LIBRARIES}
  )
endif()
