cmake_minimum_required(VERSION 2.8.3)
project(fuse_core)

set(build_depends
  fuse_msgs
  pluginlib
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
  DEPENDS
    Boost
    CERES
    EIGEN3
)

###########
## Build ##
###########

add_compile_options(-std=c++14 -Wall -Werror)

## fuse_core library
add_library(${PROJECT_NAME}
  src/async_motion_model.cpp
  src/async_publisher.cpp
  src/async_sensor_model.cpp
  src/ceres_options.cpp
  src/constraint.cpp
  src/graph.cpp
  src/graph_deserializer.cpp
  src/serialization.cpp
  src/timestamp_manager.cpp
  src/transaction.cpp
  src/transaction_deserializer.cpp
  src/uuid.cpp
  src/variable.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

# fuse_echo executable
add_executable(fuse_echo
  src/fuse_echo.cpp
)
add_dependencies(fuse_echo
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(fuse_echo
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(fuse_echo
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
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
  TARGETS fuse_echo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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

  # Lint tests
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
  roslint_cpp()
  roslint_add_test()

  # AsyncMotionModel tests
  add_rostest_gtest(test_async_motion_model
    test/async_motion_model.test
    test/test_async_motion_model.cpp
  )
  add_dependencies(test_async_motion_model
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_async_motion_model
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_async_motion_model
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # AsyncPublisher tests
  add_rostest_gtest(test_async_publisher
    test/async_publisher.test
    test/test_async_publisher.cpp
  )
  add_dependencies(test_async_publisher
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_async_publisher
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_async_publisher
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # AsyncSensorModel tests
  add_rostest_gtest(test_async_sensor_model
    test/async_sensor_model.test
    test/test_async_sensor_model.cpp
  )
  add_dependencies(test_async_sensor_model
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_async_sensor_model
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_async_sensor_model
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # CallbackWrapper tests
  add_rostest_gtest(test_callback_wrapper
    test/callback_wrapper.test
    test/test_callback_wrapper.cpp
  )
  add_dependencies(test_callback_wrapper
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_callback_wrapper
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_callback_wrapper
    ${catkin_LIBRARIES}
  )

  # Constraint tests
  catkin_add_gtest(test_constraint
    test/test_constraint.cpp
  )
  add_dependencies(test_constraint
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_constraint
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_constraint
    ${PROJECT_NAME}
  )

  # Local Parameterization tests
  catkin_add_gtest(test_local_parameterization
    test/test_local_parameterization.cpp
  )
  target_include_directories(test_local_parameterization
    PRIVATE
      include
  )
  target_link_libraries(test_local_parameterization
    ${PROJECT_NAME}
  )

  # Message Buffer Tests
  catkin_add_gtest(test_message_buffer
    test/test_message_buffer.cpp
  )
  add_dependencies(test_message_buffer
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_message_buffer
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_message_buffer
    ${catkin_LIBRARIES}
  )

  # Timestamp Manager Tests
  catkin_add_gtest(test_timestamp_manager
    test/test_timestamp_manager.cpp
  )
  add_dependencies(test_timestamp_manager
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_timestamp_manager
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_timestamp_manager
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # Transaction tests
  catkin_add_gtest(test_transaction
    test/test_transaction.cpp
  )
  add_dependencies(test_transaction
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_transaction
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_transaction
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # UUID tests
  catkin_add_gtest(test_uuid
    test/test_uuid.cpp
  )
  add_dependencies(test_uuid
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_uuid
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
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
  target_include_directories(test_variable
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_variable
    ${PROJECT_NAME}
  )
endif()
