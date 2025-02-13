cmake_minimum_required(VERSION 3.16)
project(fuse_constraints)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CXX_STANDARD_REQUIRED YES)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(fuse_core REQUIRED)
find_package(fuse_graphs REQUIRED)
find_package(fuse_variables REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(suitesparse-extras.cmake)

###########
## Build ##
###########
# lint_cmake: -linelength
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
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 12.0)
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
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Boost::serialization
  Ceres::ceres
  Eigen3::Eigen
  fuse_core::fuse_core
  fuse_graphs::fuse_graphs
  fuse_variables::fuse_variables
  ${geometry_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${SUITESPARSE_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${SUITESPARSE_INCLUDE_DIRS}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
  add_subdirectory(benchmark)
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(fuse_core fuse_plugins.xml)

ament_export_targets(${PROJECT_NAME}-export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake_ros
  fuse_core
  fuse_graphs
  fuse_variables
  geometry_msgs
  pluginlib
  rclcpp
  Ceres
  Eigen3
  glog
)

ament_package(CONFIG_EXTRAS suitesparse-extras.cmake)
