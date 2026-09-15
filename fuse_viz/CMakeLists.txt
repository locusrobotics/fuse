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
find_package(gtest_vendor)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Core)

message("Compiling with Qt${QT_VERSION_MAJOR}")

###########
## Build ##
###########

if (${QT_VERSION_MAJOR} GREATER "5")
  qt_standard_project_setup()
  qt_wrap_cpp(moc_files
    include/fuse_viz/mapped_covariance_property.hpp
    include/fuse_viz/pose_2d_stamped_property.hpp
    include/fuse_viz/relative_pose_2d_stamped_constraint_property.hpp
    include/fuse_viz/serialized_graph_display.hpp
  )
else()
  qt5_wrap_cpp(moc_files
    include/fuse_viz/mapped_covariance_property.hpp
    include/fuse_viz/pose_2d_stamped_property.hpp
    include/fuse_viz/relative_pose_2d_stamped_constraint_property.hpp
    include/fuse_viz/serialized_graph_display.hpp
  )
endif()

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
  Qt${QT_VERSION_MAJOR}::Widgets
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
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
