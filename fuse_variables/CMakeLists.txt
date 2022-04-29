cmake_minimum_required(VERSION 3.5)
project(fuse_variables)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(Ceres REQUIRED)
find_package(fuse_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

###########
## Build ##
###########

include_directories(include)

## fuse_variables library
add_library(${PROJECT_NAME}
  src/acceleration_angular_2d_stamped.cpp
  src/acceleration_angular_3d_stamped.cpp
  src/acceleration_linear_2d_stamped.cpp
  src/acceleration_linear_3d_stamped.cpp
  src/orientation_2d_stamped.cpp
  src/orientation_3d_stamped.cpp
  src/point_2d_fixed_landmark.cpp
  src/point_2d_landmark.cpp
  src/point_3d_fixed_landmark.cpp
  src/point_3d_landmark.cpp
  src/position_2d_stamped.cpp
  src/position_3d_stamped.cpp
  #src/stamped.cpp
  src/velocity_angular_2d_stamped.cpp
  src/velocity_angular_3d_stamped.cpp
  src/velocity_linear_2d_stamped.cpp
  src/velocity_linear_3d_stamped.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
${CERES_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
${CERES_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  fuse_core
  pluginlib
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  # Lint tests
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
  roslint_cpp()
  roslint_add_test()

  # Acceleration Angular 2D Stamped Tests
  catkin_add_gtest(test_acceleration_angular_2d_stamped
    test/test_acceleration_angular_2d_stamped.cpp
  )
  add_dependencies(test_acceleration_angular_2d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_acceleration_angular_2d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_acceleration_angular_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_acceleration_angular_2d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Acceleration Angular 3D Stamped Tests
  catkin_add_gtest(test_acceleration_angular_3d_stamped
    test/test_acceleration_angular_3d_stamped.cpp
  )
  add_dependencies(test_acceleration_angular_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_acceleration_angular_3d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_acceleration_angular_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_acceleration_angular_3d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Acceleration Linear 2D Stamped Tests
  catkin_add_gtest(test_acceleration_linear_2d_stamped
    test/test_acceleration_linear_2d_stamped.cpp
  )
  add_dependencies(test_acceleration_linear_2d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_acceleration_linear_2d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_acceleration_linear_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_acceleration_linear_2d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Acceleration Linear 3D Stamped Tests
  catkin_add_gtest(test_acceleration_linear_3d_stamped
    test/test_acceleration_linear_3d_stamped.cpp
  )
  add_dependencies(test_acceleration_linear_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_acceleration_linear_3d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_acceleration_linear_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_acceleration_linear_3d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Fixed Size Variable Tests
  catkin_add_gtest(test_fixed_size_variable
    test/test_fixed_size_variable.cpp
  )
  add_dependencies(test_fixed_size_variable
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_fixed_size_variable
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_fixed_size_variable
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_fixed_size_variable
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Load Device ID Tests
  add_rostest_gtest(test_load_device_id
    test/load_device_id.test
    test/test_load_device_id.cpp
  )
  add_dependencies(test_load_device_id
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_load_device_id
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_load_device_id
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_load_device_id
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Orientation 2D Stamped Tests
  catkin_add_gtest(test_orientation_2d_stamped
    test/test_orientation_2d_stamped.cpp
  )
  add_dependencies(test_orientation_2d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_orientation_2d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_orientation_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_orientation_2d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Orientation 3D Stamped Tests
  catkin_add_gtest(test_orientation_3d_stamped
    test/test_orientation_3d_stamped.cpp
  )
  add_dependencies(test_orientation_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_orientation_3d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_orientation_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_orientation_3d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Point 2D fixed landmark tests
  catkin_add_gtest(test_point_2d_fixed_landmark
    test/test_point_2d_fixed_landmark.cpp
  )
  add_dependencies(test_point_2d_fixed_landmark
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_point_2d_fixed_landmark
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_point_2d_fixed_landmark
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_point_2d_fixed_landmark
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Point 2D landmark tests
  catkin_add_gtest(test_point_2d_landmark
    test/test_point_2d_landmark.cpp
  )
  add_dependencies(test_point_2d_landmark
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_point_2d_landmark
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_point_2d_landmark
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_point_2d_landmark
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Point 3D fixed landmark tests
  catkin_add_gtest(test_point_3d_fixed_landmark
    test/test_point_3d_fixed_landmark.cpp
  )
  add_dependencies(test_point_3d_fixed_landmark
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_point_3d_fixed_landmark
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_point_3d_fixed_landmark
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_point_3d_fixed_landmark
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Point 3D landmark tests
  catkin_add_gtest(test_point_3d_landmark
    test/test_point_3d_landmark.cpp
  )
  add_dependencies(test_point_3d_landmark
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_point_3d_landmark
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_point_3d_landmark
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_point_3d_landmark
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Position 2D Stamped Tests
  catkin_add_gtest(test_position_2d_stamped
    test/test_position_2d_stamped.cpp
  )
  add_dependencies(test_position_2d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_position_2d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_position_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_position_2d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Position 3D stamped tests
  catkin_add_gtest(test_position_3d_stamped
    test/test_position_3d_stamped.cpp
  )
  add_dependencies(test_position_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_position_3d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_position_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_position_3d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Velocity Angular 2D Stamped Tests
  catkin_add_gtest(test_velocity_angular_2d_stamped
    test/test_velocity_angular_2d_stamped.cpp
  )
  add_dependencies(test_velocity_angular_2d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_velocity_angular_2d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_velocity_angular_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_velocity_angular_2d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Velocity Angular 3D Stamped Tests
  catkin_add_gtest(test_velocity_angular_3d_stamped
    test/test_velocity_angular_3d_stamped.cpp
  )
  add_dependencies(test_velocity_angular_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_velocity_angular_3d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_velocity_angular_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_velocity_angular_3d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Velocity Linear 2D Stamped Tests
  catkin_add_gtest(test_velocity_linear_2d_stamped
    test/test_velocity_linear_2d_stamped.cpp
  )
  add_dependencies(test_velocity_linear_2d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_velocity_linear_2d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_velocity_linear_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_velocity_linear_2d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Velocity Linear 3D Stamped Tests
  catkin_add_gtest(test_velocity_linear_3d_stamped
    test/test_velocity_linear_3d_stamped.cpp
  )
  add_dependencies(test_velocity_linear_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_velocity_linear_3d_stamped
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_velocity_linear_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_velocity_linear_3d_stamped
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )
endif()


ament_package(
  CONFIG_EXTRAS
)

