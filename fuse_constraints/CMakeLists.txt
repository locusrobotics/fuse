cmake_minimum_required(VERSION 3.0.2)
project(fuse_constraints)

set(build_depends
  fuse_core
  fuse_graphs
  fuse_variables
  geometry_msgs
  pluginlib
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED COMPONENTS CCOLAMD)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
  DEPENDS
    CERES
    EIGEN3
    SUITESPARSE
)

###########
## Build ##
###########

# Disable warnings about array bounds with -Wno-array-bounds until gcc 12 fixes this bug:
# https://gcc.gnu.org/bugzilla/show_bug.cgi?id=106247
#
# Also reported in Eigen, and confirmed to be due to gcc 12:
# https://gitlab.com/libeigen/eigen/-/issues/2506
#
# In file included from include/immintrin.h:43,
#                  from include/eigen3/Eigen/src/Core/util/ConfigureVectorization.h:361,
#                  from include/eigen3/Eigen/Core:22,
#                  from include/fuse_core/fuse_macros.h:63,
#                  from include/fuse_core/loss.h:37,
#                  from include/fuse_core/constraint.h:37,
#                  from src/fuse/fuse_constraints/include/fuse_constraints/relative_orientation_3d_stamped_constraint.h:37,
#                  from src/fuse/fuse_constraints/src/relative_orientation_3d_stamped_constraint.cpp:34:
# In function ‘__m256d _mm256_loadu_pd(const double*)’,
#     inlined from ‘Packet Eigen::internal::ploadu(const typename unpacket_traits<T>::type*) [with Packet = __vector(4) double]’ at include/eigen3/Eigen/src/Core/arch/AVX/PacketMath.h:582:129,
#     inlined from ‘Packet Eigen::internal::ploadt(const typename unpacket_traits<T>::type*) [with Packet = __vector(4) double; int Alignment = 0]’ at include/eigen3/Eigen/src/Core/GenericPacketMath.h:969:26,
#     inlined from ‘PacketType Eigen::internal::evaluator<Eigen::PlainObjectBase<Derived> >::packet(Eigen::Index) const [with int LoadMode = 0; PacketType = __vector(4) double; Derived = Eigen::Matrix<double, 3, 1>]’ at include/eigen3/Eigen/src/Core/CoreEvaluators.h:245:40,
#     inlined from ‘void Eigen::internal::generic_dense_assignment_kernel<DstEvaluatorTypeT, SrcEvaluatorTypeT, Functor, Version>::assignPacket(Eigen::Index) [with int StoreMode = 32; int LoadMode = 0; PacketType = __vector(4) double; DstEvaluatorTypeT = Eigen::internal::evaluator<Eigen::Map<Eigen::Matrix<double, -1, 1> > >; SrcEvaluatorTypeT = Eigen::internal::evaluator<Eigen::Matrix<double, 3, 1> >; Functor = Eigen::internal::assign_op<double, double>; int Version = 0]’ at include/eigen3/Eigen/src/Core/AssignEvaluator.h:681:114,
#     inlined from ‘static void Eigen::internal::dense_assignment_loop<Kernel, 3, 0>::run(Kernel&) [with Kernel = Eigen::internal::generic_dense_assignment_kernel<Eigen::internal::evaluator<Eigen::Map<Eigen::Matrix<double, -1, 1> > >, Eigen::internal::evaluator<Eigen::Matrix<double, 3, 1> >, Eigen::internal::assign_op<double, double>, 0>]’ at include/eigen3/Eigen/src/Core/AssignEvaluator.h:437:75,
#     inlined from ‘void Eigen::internal::call_dense_assignment_loop(DstXprType&, const SrcXprType&, const Functor&) [with DstXprType = Eigen::Map<Eigen::Matrix<double, -1, 1> >; SrcXprType = Eigen::Matrix<double, 3, 1>; Functor = assign_op<double, double>]’ at include/eigen3/Eigen/src/Core/AssignEvaluator.h:785:37,
#     inlined from ‘static void Eigen::internal::Assignment<DstXprType, SrcXprType, Functor, Eigen::internal::Dense2Dense, Weak>::run(DstXprType&, const SrcXprType&, const Functor&) [with DstXprType = Eigen::Map<Eigen::Matrix<double, -1, 1> >; SrcXprType = Eigen::Matrix<double, 3, 1>; Functor = Eigen::internal::assign_op<double, double>; Weak = void]’ at include/eigen3/Eigen/src/Core/AssignEvaluator.h:954:31,
#     inlined from ‘void Eigen::internal::call_assignment_no_alias(Dst&, const Src&, const Func&) [with Dst = Eigen::Map<Eigen::Matrix<double, -1, 1> >; Src = Eigen::Matrix<double, 3, 1>; Func = assign_op<double, double>]’ at include/eigen3/Eigen/src/Core/AssignEvaluator.h:890:49,
#     inlined from ‘void Eigen::internal::call_assignment(Dst&, const Src&, const Func&, typename enable_if<evaluator_assume_aliasing<Src>::value, void*>::type) [with Dst = Eigen::Map<Eigen::Matrix<double, -1, 1> >; Src = Eigen::Product<Eigen::Matrix<double, 3, 3, 1>, Eigen::Map<Eigen::Matrix<double, -1, 1> >, 0>; Func = assign_op<double, double>]’ at include/eigen3/Eigen/src/Core/AssignEvaluator.h:851:27,
#     inlined from ‘void Eigen::internal::call_assignment(Dst&, const Src&) [with Dst = Eigen::Map<Eigen::Matrix<double, -1, 1> >; Src = Eigen::Product<Eigen::Matrix<double, 3, 3, 1>, Eigen::Map<Eigen::Matrix<double, -1, 1> >, 0>]’ at include/eigen3/Eigen/src/Core/AssignEvaluator.h:836:18,
#     inlined from ‘Derived& Eigen::MatrixBase<Derived>::operator=(const Eigen::DenseBase<OtherDerived>&) [with OtherDerived = Eigen::Product<Eigen::Matrix<double, 3, 3, 1>, Eigen::Map<Eigen::Matrix<double, -1, 1> >, 0>; Derived = Eigen::Map<Eigen::Matrix<double, -1, 1> >]’ at include/eigen3/Eigen/src/Core/Assign.h:66:28,
#     inlined from ‘void Eigen::EigenBase<Derived>::applyThisOnTheLeft(Dest&) const [with Dest = Eigen::Map<Eigen::Matrix<double, -1, 1> >; Derived = Eigen::Matrix<double, 3, 3, 1>]’ at include/eigen3/Eigen/src/Core/EigenBase.h:114:9:
# include/avxintrin.h:893:24: error: array subscript ‘__m256d_u[0]’ is partly outside array bounds of ‘Eigen::internal::plain_matrix_type<Eigen::Product<Eigen::Matrix<double, 3, 3, 1>, Eigen::Map<Eigen::Matrix<double, -1, 1> >, 0>, Eigen::Dense>::type [1]’ {aka ‘Eigen::Matrix<double, 3, 1> [1]’} [-Werror=array-bounds]
#   893 |   return *(__m256d_u *)__P;
#       |                        ^~~
# In file included from include/eigen3/Eigen/Core:278:
# include/eigen3/Eigen/src/Core/AssignEvaluator.h: In member function ‘void Eigen::EigenBase<Derived>::applyThisOnTheLeft(Dest&) const [with Dest = Eigen::Map<Eigen::Matrix<double, -1, 1> >; Derived = Eigen::Matrix<double, 3, 3, 1>]’:
# include/eigen3/Eigen/src/Core/AssignEvaluator.h:850:41: note: at offset [0, 16] into object ‘tmp’ of size 24
#   850 |   typename plain_matrix_type<Src>::type tmp(src);
#       |                                         ^~~
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 12.0)
  add_compile_options(-Wall -Werror -Wno-array-bounds)
else()
  add_compile_options(-Wall -Werror)
endif()

# fuse_constraints library
add_library(${PROJECT_NAME}
  src/absolute_constraint.cpp
  src/absolute_orientation_3d_stamped_constraint.cpp
  src/absolute_orientation_3d_stamped_euler_constraint.cpp
  src/absolute_pose_2d_stamped_constraint.cpp
  src/absolute_pose_3d_stamped_constraint.cpp
  src/fixed_3d_landmark_constraint.cpp
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
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
)
target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
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
      CXX_STANDARD 17
      CXX_STANDARD_REQUIRED YES
  )

  # Absolute Orientation 2D Stamped Constraint Tests
  catkin_add_gtest(test_absolute_orientation_2d_stamped_constraint
    test/test_absolute_orientation_2d_stamped_constraint.cpp
  )
  add_dependencies(test_absolute_orientation_2d_stamped_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_absolute_orientation_2d_stamped_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_absolute_orientation_2d_stamped_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${EIGEN3_LIBRARIES}
  )
  set_target_properties(test_absolute_orientation_2d_stamped_constraint
    PROPERTIES
      CXX_STANDARD 17
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
      CXX_STANDARD 17
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
      CXX_STANDARD 17
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
      CXX_STANDARD 17
      CXX_STANDARD_REQUIRED YES
  )

  # Fixed 3D Landmark Constraint Tests
  catkin_add_gtest(test_fixed_3d_landmark_constraint
    test/test_fixed_3d_landmark_constraint.cpp
  )
  add_dependencies(test_fixed_3d_landmark_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_fixed_3d_landmark_constraint
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_fixed_3d_landmark_constraint
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_fixed_3d_landmark_constraint
    PROPERTIES
      CXX_STANDARD 17
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
      CXX_STANDARD 17
      CXX_STANDARD_REQUIRED YES
  )

  # Marginal Constraint Tests
  catkin_add_gtest(test_marginal_constraint
    test/test_marginal_constraint.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
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
      CXX_STANDARD 17
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
      CXX_STANDARD 17
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
      CXX_STANDARD 17
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
      CXX_STANDARD 17
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
      CXX_STANDARD 17
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
      CXX_STANDARD 17
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
      CXX_STANDARD 17
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
      CXX_STANDARD 17
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
      CXX_STANDARD 17
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
          CXX_STANDARD 17
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
          CXX_STANDARD 17
          CXX_STANDARD_REQUIRED YES
      )
    endif()
  endif()
endif()
