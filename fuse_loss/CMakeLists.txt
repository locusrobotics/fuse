cmake_minimum_required(VERSION 3.16)
project(fuse_loss)

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

## fuse_loss library
add_library(${PROJECT_NAME}
  src/arctan_loss.cpp
  src/cauchy_loss.cpp
  src/composed_loss.cpp
  src/dcs_loss.cpp
  src/fair_loss.cpp
  src/geman_mcclure_loss.cpp
  src/huber_loss.cpp
  src/loss_function.cpp
  src/scaled_loss.cpp
  src/softlone_loss.cpp
  src/tolerant_loss.cpp
  src/trivial_loss.cpp
  src/tukey_loss.cpp
  src/welsch_loss.cpp
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
