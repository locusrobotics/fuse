cmake_minimum_required(VERSION 3.0.2)
project(fuse_viz)

set(build_depends
  fuse_constraints
  fuse_core
  fuse_msgs
  fuse_variables
  geometry_msgs
  rviz
  tf2_geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(QT_INCLUDE_DIRS
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)
set(QT_LIBRARIES
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

add_definitions(-DQT_NO_KEYWORDS)

###########
## Build ##
###########
# add_compile_options(-Wall -Werror)
add_compile_options(-Wall)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
  DEPENDS
    Boost
    EIGEN3
    QT
)

qt5_wrap_cpp(moc_files
  include/fuse_viz/mapped_covariance_property.h
  include/fuse_viz/pose_2d_stamped_property.h
  include/fuse_viz/relative_pose_2d_stamped_constraint_property.h
  include/fuse_viz/serialized_graph_display.h
)

set(source_files
  src/mapped_covariance_property.cpp
  src/mapped_covariance_visual.cpp
  src/pose_2d_stamped_property.cpp
  src/pose_2d_stamped_visual.cpp
  src/relative_pose_2d_stamped_constraint_property.cpp
  src/relative_pose_2d_stamped_constraint_visual.cpp
  src/serialized_graph_display.cpp
  ${moc_files}
)

add_library(${PROJECT_NAME}
  ${source_files}
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
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${QT_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  # Lint tests
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
  roslint_cpp()
  roslint_add_test()
endif()
