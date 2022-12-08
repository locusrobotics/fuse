cmake_minimum_required(VERSION 3.16)
project(fuse_variables)

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

find_package(Ceres REQUIRED)

###########
## Build ##
###########

## fuse_variables library
add_library(${PROJECT_NAME}
  src/acceleration_angular_2d_stamped.cpp
  src/acceleration_angular_3d_stamped.cpp
  src/acceleration_linear_2d_stamped.cpp
  src/acceleration_linear_3d_stamped.cpp
  src/orientation_2d_stamped.cpp
  src/orientation_3d_stamped.cpp
  src/point_2d_fixed_landmark.cpp
  src/point_2d_landmark.cpp
  src/point_3d_fixed_landmark.cpp
  src/point_3d_landmark.cpp
  src/position_2d_stamped.cpp
  src/position_3d_stamped.cpp
  src/stamped.cpp
  src/velocity_angular_2d_stamped.cpp
  src/velocity_angular_3d_stamped.cpp
  src/velocity_linear_2d_stamped.cpp
  src/velocity_linear_3d_stamped.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Ceres::ceres
  fuse_core::fuse_core
  pluginlib::pluginlib
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
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
  Ceres
)

ament_package()
