cmake_minimum_required(VERSION 3.16)
project(fuse_viz)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CXX_STANDARD_REQUIRED YES)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(fuse_constraints REQUIRED)
find_package(fuse_core REQUIRED)
find_package(fuse_msgs REQUIRED)
find_package(fuse_variables REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

###########
## Build ##
###########

qt5_wrap_cpp(moc_files
  include/fuse_viz/mapped_covariance_property.hpp
  include/fuse_viz/pose_2d_stamped_property.hpp
  include/fuse_viz/relative_pose_2d_stamped_constraint_property.hpp
  include/fuse_viz/serialized_graph_display.hpp
)

add_library(${PROJECT_NAME} SHARED
  src/mapped_covariance_property.cpp
  src/mapped_covariance_visual.cpp
  src/pose_2d_stamped_property.cpp
  src/pose_2d_stamped_visual.cpp
  src/relative_pose_2d_stamped_constraint_property.cpp
  src/relative_pose_2d_stamped_constraint_visual.cpp
  src/serialized_graph_display.cpp
  ${moc_files}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${fuse_msgs_TARGETS}
  Eigen3::Eigen
  fuse_constraints::fuse_constraints
  fuse_core::fuse_core
  fuse_variables::fuse_variables
  ${geometry_msgs_TARGETS}
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  tf2_geometry_msgs::tf2_geometry_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
