cmake_minimum_required(VERSION 3.5)
project(fuse_publishers)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(fuse_core)
find_package(fuse_variables)
find_package(geometry_msgs)
find_package(nav_msgs)
find_package(pluginlib)
find_package(rclcpp)
find_package(tf2)
find_package(tf2_geometry_msgs)
find_package(tf2_ros)

###########
## Build ##
###########

include_directories(include)

# fuse_publishers library
add_library(${PROJECT_NAME} SHARED
  src/path_2d_publisher.cpp
  src/pose_2d_publisher.cpp
  #src/serialized_publisher.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)
target_link_libraries(${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}
  fuse_core
  fuse_variables
  rclcpp
  geometry_msgs
  nav_msgs
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
)

#############
## Install ##
#############

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION DESTINATION lib
  LIBRARY DESTINATION DESTINATION lib
  RUNTIME DESTINATION DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(FILES fuse_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  set(test_depends
    fuse_constraints
    fuse_graphs
  )

  find_package(catkin REQUIRED COMPONENTS
    ${build_depends}
    ${test_depends}
  )
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
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_path_2d_publisher
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_path_2d_publisher
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
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
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_pose_2d_publisher
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_pose_2d_publisher
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # StampedVariableSynchronizer Tests
  catkin_add_gtest(test_stamped_variable_synchronizer
    test/test_stamped_variable_synchronizer.cpp
  )
  add_dependencies(test_stamped_variable_synchronizer
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_stamped_variable_synchronizer
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_stamped_variable_synchronizer
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_stamped_variable_synchronizer
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )
endif()

ament_package(
  CONFIG_EXTRAS
)
