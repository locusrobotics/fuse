cmake_minimum_required(VERSION 3.0.2)
project(fuse_tutorials)

find_package(catkin REQUIRED COMPONENTS
  fuse_constraints
  fuse_core
  fuse_models
  fuse_variables
  nav_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    fuse_constraints
    fuse_core
    fuse_models
    fuse_variables
    nav_msgs
    roscpp
    sensor_msgs
)

###########
## Build ##
###########
add_compile_options(-Wall -Werror)

## fuse_tutorial library
add_library(${PROJECT_NAME}
  src/beacon_publisher.cpp
  src/range_constraint.cpp
  src/range_sensor_model.cpp
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
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
)

# tutorial_sim executable
add_executable(range_sensor_simulator
  src/range_sensor_simulator.cpp
)
add_dependencies(range_sensor_simulator
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(range_sensor_simulator
  PUBLIC
    include
)
target_include_directories(range_sensor_simulator
  SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(range_sensor_simulator
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
set_target_properties(range_sensor_simulator
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
  TARGETS range_sensor_simulator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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

  # Lint tests
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
  roslint_cpp()
  roslint_add_test()
endif()
