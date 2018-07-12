cmake_minimum_required(VERSION 2.8.3)
project(fuse_constraints)

find_package(catkin REQUIRED COMPONENTS
  fuse_core
  fuse_variables
  roscpp
)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    ${CERES_LIBRARIES}
  CATKIN_DEPENDS
    fuse_core
    fuse_variables
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
  ${EIGEN3_INCLUDE_DIRS}
)

# fuse_constraints library
add_library(${PROJECT_NAME}
  src/absolute_pose_2d_stamped_constraint.cpp
  src/normal_delta.cpp
  src/normal_delta_orientation_2d.cpp
  src/normal_prior_orientation_2d.cpp
  src/relative_pose_2d_stamped_constraint.cpp
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

  # Absolute Constraint Tests
  catkin_add_gtest(test_absolute_constraint
    test/test_absolute_constraint.cpp
  )
  add_dependencies(test_absolute_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_absolute_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Absolute Pose 2D Stamped Constraint Tests
  catkin_add_gtest(test_absolute_pose_2d_stamped_constraint
    test/test_absolute_pose_2d_stamped_constraint.cpp
  )
  add_dependencies(test_absolute_pose_2d_stamped_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_absolute_pose_2d_stamped_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # Relative Constraint Tests
  catkin_add_gtest(test_relative_constraint
    test/test_relative_constraint.cpp
  )
  add_dependencies(test_relative_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_relative_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )

  # Relative Pose 2D Stamped Constraint Tests
  catkin_add_gtest(test_relative_pose_2d_stamped_constraint
    test/test_relative_pose_2d_stamped_constraint.cpp
  )
  add_dependencies(test_relative_pose_2d_stamped_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_relative_pose_2d_stamped_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()
