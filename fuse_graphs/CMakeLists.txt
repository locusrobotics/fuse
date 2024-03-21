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
# disabling until deprecated ceres::Problem::Options::local_parameterization_ownership is resolved
add_compile_options(-Wall)  # -Werror)

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
