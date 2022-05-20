cmake_minimum_required(VERSION 3.5)
project(fuse_graphs)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)
find_package(fuse_core)
find_package(pluginlib)
find_package(rclcpp)

###########
## Build ##
###########

include_directories(include)

## fuse_graphs library
add_library(${PROJECT_NAME}
  src/hash_graph.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
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
  DIRECTORY include/
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
      CXX_STANDARD 14
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
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED YES
    )
  endif()
endif()

ament_package(
  CONFIG_EXTRAS
)
