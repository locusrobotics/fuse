cmake_minimum_required(VERSION 3.5)
project(fuse_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/SerializedGraph.msg"
  "msg/SerializedTransaction.msg"
)

set(srv_files
  "srv/SetGraph.srv"
  "srv/SetPose.srv"
  "srv/SetPoseDeprecated.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files}
  DEPENDENCIES
    std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
