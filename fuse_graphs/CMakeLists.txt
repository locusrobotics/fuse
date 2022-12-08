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
