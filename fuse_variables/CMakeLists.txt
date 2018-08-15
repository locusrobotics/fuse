cmake_minimum_required(VERSION 2.8.3)
project(fuse_variables)

find_package(catkin REQUIRED COMPONENTS
  fuse_core
  roscpp
)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${CERES_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    ${CERES_LIBRARIES}
  CATKIN_DEPENDS
    fuse_core
    roscpp
)

###########
## Build ##
###########

add_compile_options(-std=c++14 -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## fuse_variables library
add_library(${PROJECT_NAME}
  src/acceleration_angular_2d_stamped.cpp
  src/acceleration_angular_3d_stamped.cpp
  src/acceleration_linear_2d_stamped.cpp
  src/acceleration_linear_3d_stamped.cpp
  src/orientation_2d_stamped.cpp
  src/orientation_3d_stamped.cpp
  src/position_2d_stamped.cpp
  src/position_3d_stamped.cpp
  src/stamped.cpp
  src/velocity_angular_2d_stamped.cpp
  src/velocity_angular_3d_stamped.cpp
  src/velocity_linear_2d_stamped.cpp
  src/velocity_linear_3d_stamped.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
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
  target_link_libraries(test_acceleration_angular_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Acceleration Angular 3D Stamped Tests
  catkin_add_gtest(test_acceleration_angular_3d_stamped
    test/test_acceleration_angular_3d_stamped.cpp
  )
  add_dependencies(test_acceleration_angular_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_acceleration_angular_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Acceleration Linear 2D Stamped Tests
  catkin_add_gtest(test_acceleration_linear_2d_stamped
    test/test_acceleration_linear_2d_stamped.cpp
  )
  add_dependencies(test_acceleration_linear_2d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_acceleration_linear_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Acceleration Linear 3D Stamped Tests
  catkin_add_gtest(test_acceleration_linear_3d_stamped
    test/test_acceleration_linear_3d_stamped.cpp
  )
  add_dependencies(test_acceleration_linear_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_acceleration_linear_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Fixed Size Variable Tests
  catkin_add_gtest(test_fixed_size_variable
    test/test_fixed_size_variable.cpp
  )
  add_dependencies(test_fixed_size_variable
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_fixed_size_variable
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # Load Device ID Tests
  add_rostest_gtest(test_load_device_id
    test/load_device_id.test
    test/test_load_device_id.cpp
  )
  add_dependencies(test_load_device_id
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_load_device_id
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # Orientation 2D Stamped Tests
  catkin_add_gtest(test_orientation_2d_stamped
    test/test_orientation_2d_stamped.cpp
  )
  add_dependencies(test_orientation_2d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_orientation_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Orientation 3D Stamped Tests
  catkin_add_gtest(test_orientation_3d_stamped
    test/test_orientation_3d_stamped.cpp
  )
  add_dependencies(test_orientation_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_orientation_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Position 2D Stamped Tests
  catkin_add_gtest(test_position_2d_stamped
    test/test_position_2d_stamped.cpp
  )
  add_dependencies(test_position_2d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_position_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Position 3D stamped tests
  catkin_add_gtest(test_position_3d_stamped
    test/test_position_3d_stamped.cpp
  )
  add_dependencies(test_position_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_position_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Velocity Angular 2D Stamped Tests
  catkin_add_gtest(test_velocity_angular_2d_stamped
    test/test_velocity_angular_2d_stamped.cpp
  )
  add_dependencies(test_velocity_angular_2d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_velocity_angular_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Velocity Angular 3D Stamped Tests
  catkin_add_gtest(test_velocity_angular_3d_stamped
    test/test_velocity_angular_3d_stamped.cpp
  )
  add_dependencies(test_velocity_angular_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_velocity_angular_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Velocity Linear 2D Stamped Tests
  catkin_add_gtest(test_velocity_linear_2d_stamped
    test/test_velocity_linear_2d_stamped.cpp
  )
  add_dependencies(test_velocity_linear_2d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_velocity_linear_2d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Velocity Linear 3D Stamped Tests
  catkin_add_gtest(test_velocity_linear_3d_stamped
    test/test_velocity_linear_3d_stamped.cpp
  )
  add_dependencies(test_velocity_linear_3d_stamped
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_velocity_linear_3d_stamped
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
endif()
