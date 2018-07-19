cmake_minimum_required(VERSION 2.8.3)
project(fuse_core)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${CERES_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    ${CERES_LIBRARIES}
  CATKIN_DEPENDS
    roscpp
  DEPENDS
    Boost
)

###########
## Build ##
###########

add_compile_options(-std=c++14 -Wall -Werror)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## fuse_core library
add_library(${PROJECT_NAME}
  src/async_sensor_model.cpp
  src/constraint.cpp
  src/graph.cpp
  src/sensor_model.cpp
  src/transaction.cpp
  src/variable.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
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
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)
  include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # Lint tests
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
  roslint_cpp()
  roslint_add_test()

  # Constraint tests
  catkin_add_gtest(test_constraint
    test/test_constraint.cpp
  )
  add_dependencies(test_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_constraint
    ${PROJECT_NAME}
  )

  # Transaction tests
  catkin_add_gtest(test_transaction
    test/test_transaction.cpp
  )
  add_dependencies(test_transaction
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_transaction
    ${PROJECT_NAME}
  )

  # UUID tests
  catkin_add_gtest(test_uuid
    test/test_uuid.cpp
  )
  add_dependencies(test_uuid
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_uuid
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # Variable tests
  catkin_add_gtest(test_variable
    test/test_variable.cpp
  )
  add_dependencies(test_variable
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(test_variable
    ${PROJECT_NAME}
  )
endif()
