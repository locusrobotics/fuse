cmake_minimum_required(VERSION 2.8.3)
project(fuse_test)

set(build_depends
  fuse_constraints
  fuse_core
  fuse_msgs
  fuse_variables
  pluginlib
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
)

catkin_package()

add_compile_options(-std=c++14 -Wall -Werror)

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

  # Serialization tests
  catkin_add_gtest(test_serialization
    test/test_serialization.cpp
  )
  add_dependencies(test_serialization
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_serialization
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_serialization
    ${catkin_LIBRARIES}
  )

endif()
