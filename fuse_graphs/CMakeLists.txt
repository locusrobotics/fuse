cmake_minimum_required(VERSION 3.0.2)
project(fuse_graphs)

set(build_depends
  fuse_core
  pluginlib
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
  DEPENDS
    Boost
    CERES
)

###########
## Build ##
###########

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
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
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

install(
  DIRECTORY include/${PROJECT_NAME}/
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

  # HashGraph tests
  catkin_add_gtest(test_hash_graph
    test/test_hash_graph.cpp
  )
  add_dependencies(test_hash_graph
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_hash_graph
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  target_link_libraries(test_hash_graph
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  set_target_properties(test_hash_graph
    PROPERTIES
      CXX_STANDARD 17
      CXX_STANDARD_REQUIRED YES
  )

  # Benchmarks
  find_package(benchmark QUIET)

  if(benchmark_FOUND)
    # Create Problem benchmark
    add_executable(benchmark_create_problem
      benchmark/benchmark_create_problem.cpp
    )
    target_include_directories(benchmark_create_problem
      PRIVATE
        include
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(benchmark_create_problem
      benchmark
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
    set_target_properties(benchmark_create_problem
      PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
    )
  endif()
endif()
