cmake_minimum_required(VERSION 3.16)
project(fuse_graphs)

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
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Ceres REQUIRED)

include(boost-extras.cmake)

###########
## Build ##
###########
# lint_cmake: -linelength
# Disable warnings about maybe uninitialized variables with -Wno-maybe-uninitialized until we fix the following error:
#
# In file included from include/c++/12.2.0/functional:59,
#                  from include/eigen3/Eigen/Core:85,
#                  from include/fuse_core/fuse_macros.h:63,
#                  from include/fuse_core/loss.h:37,
#                  from include/fuse_core/constraint.h:37,
#                  from src/fuse/fuse_graphs/include/fuse_graphs/hash_graph.h:38,
#                  from src/fuse/fuse_graphs/src/hash_graph.cpp:34:
# In copy constructor ‘std::function<_Res(_ArgTypes ...)>::function(const std::function<_Res(_ArgTypes ...)>&) [with _Res = const fuse_core::Variable&; _ArgTypes = {const std::pair<const boost::uuids::uuid, std::shared_ptr<fuse_core::Variable> >&}]’,
#     inlined from ‘boost::iterators::transform_iterator<UnaryFunction, Iterator, Reference, Value>::transform_iterator(const Iterator&, UnaryFunc) [with UnaryFunc = std::function<const fuse_core::Variable&(const std::pair<const boost::uuids::uuid, std::shared_ptr<fuse_core::Variable> >&)>; Iterator = std::__detail::_Node_const_iterator<std::pair<const boost::uuids::uuid, std::shared_ptr<fuse_core::Variable> >, false, true>; Reference = boost::use_default; Value = boost::use_default]’ at include/boost/iterator/transform_iterator.hpp:96:21,
#     inlined from ‘boost::iterators::transform_iterator<UnaryFunc, Iterator> boost::iterators::make_transform_iterator(Iterator, UnaryFunc) [with UnaryFunc = std::function<const fuse_core::Variable&(const std::pair<const boost::uuids::uuid, std::shared_ptr<fuse_core::Variable> >&)>; Iterator = std::__detail::_Node_const_iterator<std::pair<const boost::uuids::uuid, std::shared_ptr<fuse_core::Variable> >, false, true>]’ at include/boost/iterator/transform_iterator.hpp:141:61,
#     inlined from ‘virtual fuse_core::Graph::const_variable_range fuse_graphs::HashGraph::getVariables() const’ at fuse/fuse_graphs/src/hash_graph.cpp:284:35:
# /nix/store/b7hvml0m3qmqraz1022fwvyyg6fc1vdy-gcc-12.2.0/include/c++/12.2.0/bits/std_function.h:391:17: error: ‘<anonymous>’ may be used uninitialized [-Werror=maybe-uninitialized]
#   391 |             __x._M_manager(_M_functor, __x._M_functor, __clone_functor);
#       |             ~~~~^~~~~~~~~~
# /nix/store/b7hvml0m3qmqraz1022fwvyyg6fc1vdy-gcc-12.2.0/include/c++/12.2.0/bits/std_function.h: In member function ‘virtual fuse_core::Graph::const_variable_range fuse_graphs::HashGraph::getVariables() const’:
# /nix/store/b7hvml0m3qmqraz1022fwvyyg6fc1vdy-gcc-12.2.0/include/c++/12.2.0/bits/std_function.h:267:7: note: by argument 2 of type ‘const std::_Any_data&’ to ‘static bool std::_Function_handler<_Res(_ArgTypes ...), _Functor>::_M_manager(std::_Any_data&, const std::_Any_data&, std::_Manager_operation) [with _Res = const fuse_core::Variable&; _Functor = fuse_graphs::HashGraph::getVariables() const::<lambda(const std::unordered_map<boost::uuids::uuid, std::shared_ptr<fuse_core::Variable>, boost::hash<boost::uuids::uuid> >::value_type&)>; _ArgTypes = {const std::pair<const boost::uuids::uuid, std::shared_ptr<fuse_core::Variable> >&}]’ declared here
#   267 |       _M_manager(_Any_data& __dest, const _Any_data& __source,
#       |       ^~~~~~~~~~
add_compile_options(-Wall -Werror -Wno-maybe-uninitialized)

## fuse_graphs library
add_library(${PROJECT_NAME}
  src/hash_graph.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Boost::serialization
  Ceres::ceres
  fuse_core::fuse_core
  pluginlib::pluginlib
  rclcpp::rclcpp
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)

  # Benchmarks
  find_package(benchmark QUIET)
  if(benchmark_FOUND)
    # Create Problem benchmark
    add_executable(benchmark_create_problem benchmark/benchmark_create_problem.cpp)
    target_include_directories(benchmark_create_problem PRIVATE "test/")
    target_link_libraries(benchmark_create_problem
      ${PROJECT_NAME}
      benchmark::benchmark
    )
  endif()
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

pluginlib_export_plugin_description_file(fuse_core fuse_plugins.xml)

ament_export_targets(${PROJECT_NAME}-export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake_ros
  fuse_core
  pluginlib
  rclcpp
  Ceres
)

ament_package(CONFIG_EXTRAS boost-extras.cmake)
