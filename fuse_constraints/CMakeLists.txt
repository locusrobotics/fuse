cmake_minimum_required(VERSION 3.5)
project(fuse_constraints)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake)
find_package(rclcpp)
find_package(fuse_core)
find_package(fuse_variables)
find_package(pluginlib)
find_package(geometry_msgs REQUIRED)


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED COMPONENTS CCOLAMD)

###########
## Build ##
###########

include_directories(include)

# fuse_constraints library
add_library(${PROJECT_NAME}
  src/absolute_constraint.cpp
  src/absolute_orientation_3d_stamped_constraint.cpp
  src/absolute_orientation_3d_stamped_euler_constraint.cpp
  src/absolute_pose_2d_stamped_constraint.cpp
  src/absolute_pose_3d_stamped_constraint.cpp
  src/marginal_constraint.cpp
  src/marginal_cost_function.cpp
  src/marginalize_variables.cpp
  src/normal_delta.cpp
  src/normal_delta_orientation_2d.cpp
  src/normal_delta_pose_2d.cpp
  src/normal_prior_orientation_2d.cpp
  src/normal_prior_pose_2d.cpp
  src/relative_constraint.cpp
  src/relative_orientation_3d_stamped_constraint.cpp
  src/relative_pose_2d_stamped_constraint.cpp
  src/relative_pose_3d_stamped_constraint.cpp
  src/uuid_ordering.cpp
  src/variable_constraints.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${CERES_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  fuse_core
  fuse_variables
  pluginlib
  geometry_msgs
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
  DIRECTORY include/
  DESTINATION include/
)

#install(
#  FILES fuse_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

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
  target_include_directories(test_absolute_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_absolute_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_absolute_constraint
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Absolute Orientation 3D Stamped Constraint Tests
  catkin_add_gtest(test_absolute_orientation_3d_stamped_constraint
    test/test_absolute_orientation_3d_stamped_constraint.cpp
  )
  add_dependencies(test_absolute_orientation_3d_stamped_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_absolute_orientation_3d_stamped_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_absolute_orientation_3d_stamped_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${EIGEN3_LIBRARIES}
  )
  set_target_properties(test_absolute_orientation_3d_stamped_constraint
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Absolute Orientation 3D Stamped Euler Constraint Tests
    catkin_add_gtest(test_absolute_orientation_3d_stamped_euler_constraint
    test/test_absolute_orientation_3d_stamped_euler_constraint.cpp
  )
  add_dependencies(test_absolute_orientation_3d_stamped_euler_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_absolute_orientation_3d_stamped_euler_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_absolute_orientation_3d_stamped_euler_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${EIGEN3_LIBRARIES}
  )
  set_target_properties(test_absolute_orientation_3d_stamped_euler_constraint
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Absolute Pose 2D Stamped Constraint Tests
  catkin_add_gtest(test_absolute_pose_2d_stamped_constraint
    test/test_absolute_pose_2d_stamped_constraint.cpp
  )
  add_dependencies(test_absolute_pose_2d_stamped_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_absolute_pose_2d_stamped_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_absolute_pose_2d_stamped_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_absolute_pose_2d_stamped_constraint
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Absolute Pose 3D Stamped Constraint Tests
  catkin_add_gtest(test_absolute_pose_3d_stamped_constraint
    test/test_absolute_pose_3d_stamped_constraint.cpp
  )
  add_dependencies(test_absolute_pose_3d_stamped_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_absolute_pose_3d_stamped_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_absolute_pose_3d_stamped_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_absolute_pose_3d_stamped_constraint
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Marginal Constraint Tests
  catkin_add_gtest(test_marginal_constraint
    test/test_marginal_constraint.cpp
  )
  add_dependencies(test_marginal_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_marginal_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_marginal_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_marginal_constraint
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Marginalize Variables Tests
  catkin_add_gtest(test_marginalize_variables
    test/test_marginalize_variables.cpp
  )
  add_dependencies(test_marginalize_variables
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_marginalize_variables
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_marginalize_variables
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_marginalize_variables
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Normal Delta Pose 2D Tests
  catkin_add_gtest(test_normal_delta_pose_2d
    test/test_normal_delta_pose_2d.cpp
  )
  add_dependencies(test_normal_delta_pose_2d
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_normal_delta_pose_2d
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  target_link_libraries(test_normal_delta_pose_2d
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_normal_delta_pose_2d
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Normal Prior Pose 2D Tests
  catkin_add_gtest(test_normal_prior_pose_2d
    test/test_normal_prior_pose_2d.cpp
  )
  add_dependencies(test_normal_prior_pose_2d
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_normal_prior_pose_2d
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  target_link_libraries(test_normal_prior_pose_2d
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_normal_prior_pose_2d
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Relative Constraint Tests
  catkin_add_gtest(test_relative_constraint
    test/test_relative_constraint.cpp
  )
  add_dependencies(test_relative_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_relative_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_relative_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_relative_constraint
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Relative Pose 2D Stamped Constraint Tests
  catkin_add_gtest(test_relative_pose_2d_stamped_constraint
    test/test_relative_pose_2d_stamped_constraint.cpp
  )
  add_dependencies(test_relative_pose_2d_stamped_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_relative_pose_2d_stamped_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_relative_pose_2d_stamped_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_relative_pose_2d_stamped_constraint
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Relative Pose 3D Stamped Constraint Tests
  catkin_add_gtest(test_relative_pose_3d_stamped_constraint
    test/test_relative_pose_3d_stamped_constraint.cpp
  )
  add_dependencies(test_relative_pose_3d_stamped_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_relative_pose_3d_stamped_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_relative_pose_3d_stamped_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_relative_pose_3d_stamped_constraint
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # UuidOrdering Tests
  catkin_add_gtest(test_uuid_ordering
    test/test_uuid_ordering.cpp
  )
  target_include_directories(test_uuid_ordering
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_uuid_ordering
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_uuid_ordering
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # VariableConstraints Tests
  catkin_add_gtest(test_variable_constraints
    test/test_variable_constraints.cpp
  )
  target_include_directories(test_variable_constraints
    PRIVATE
      include
  )
  target_link_libraries(test_variable_constraints
    ${PROJECT_NAME}
  )
  set_target_properties(test_variable_constraints
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Benchmarks
  find_package(benchmark QUIET)

  if(benchmark_FOUND)
    # Normal Delta Pose 2D benchmark
    add_executable(benchmark_normal_delta_pose_2d
      benchmark/benchmark_normal_delta_pose_2d.cpp
    )
    if(TARGET benchmark_normal_delta_pose_2d)
      target_link_libraries(
        benchmark_normal_delta_pose_2d
        benchmark
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
      )
      set_target_properties(benchmark_normal_delta_pose_2d
        PROPERTIES
          CXX_STANDARD 14
          CXX_STANDARD_REQUIRED YES
      )
    endif()

    # Normal Prior Pose 2D benchmark
    add_executable(benchmark_normal_prior_pose_2d
      benchmark/benchmark_normal_prior_pose_2d.cpp
    )
    if(TARGET benchmark_normal_prior_pose_2d)
      target_link_libraries(
        benchmark_normal_prior_pose_2d
        benchmark
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
      )
      set_target_properties(benchmark_normal_prior_pose_2d
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

