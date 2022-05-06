cmake_minimum_required(VERSION 3.5)
project(fuse_msgs)

#find_package(find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#list message files to generate
set(msg_files
  "msg/SerializedGraph.msg"
  "msg/SerializedTransaction.msg"
)

## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package(
  CONFIG_EXTRAS
)
