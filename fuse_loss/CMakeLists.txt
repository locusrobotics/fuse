cmake_minimum_required(VERSION 3.0.2)
project(fuse_loss)

set(build_depends
  fuse_core
  pluginlib
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${build_depends}
)

find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
  DEPENDS
    CERES
)

###########
## Build ##
###########

add_compile_options(-Wall -Werror)

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
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 14
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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
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

  find_package(Qt5 REQUIRED COMPONENTS
    Core
    Widgets)

  # Find Qwt using FindQwt.cmake copied from:
  # https://gitlab.kitware.com/cmake/community/-/wikis/contrib/modules/FindQwt
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  find_package(Qwt REQUIRED)

  # Lint tests
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
  roslint_cpp()
  roslint_add_test()

  # Loss function Tests
  catkin_add_gtest(test_loss_function
    test/test_loss_function.cpp
  )
  add_dependencies(test_loss_function
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_loss_function
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_loss_function
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_loss_function
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Composed Loss Tests
  catkin_add_gtest(test_composed_loss
    test/test_composed_loss.cpp
  )
  add_dependencies(test_composed_loss
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_composed_loss
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_composed_loss
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_composed_loss
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Huber Loss Tests
  catkin_add_gtest(test_huber_loss
    test/test_huber_loss.cpp
  )
  add_dependencies(test_huber_loss
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_huber_loss
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_huber_loss
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_huber_loss
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Tukey Loss Tests
  catkin_add_gtest(test_tukey_loss
    test/test_tukey_loss.cpp
  )
  add_dependencies(test_tukey_loss
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_tukey_loss
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_tukey_loss
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_tukey_loss
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )

  # Qwt Loss Plot Tests
  option(BUILD_WITH_PLOT_TESTS "Build with plot tests. These test might fail to run in headless systems." OFF)
  if(BUILD_WITH_PLOT_TESTS)
    catkin_add_gtest(test_qwt_loss_plot
      test/test_qwt_loss_plot.cpp
    )
    add_dependencies(test_qwt_loss_plot
      ${catkin_EXPORTED_TARGETS}
    )
    target_include_directories(test_qwt_loss_plot
      PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
        ${QWT_INCLUDE_DIRS}
    )
    target_link_libraries(test_qwt_loss_plot
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${CERES_LIBRARIES}
      ${Qt5Core_LIBRARIES}
      ${Qt5Widgets_LIBRARIES}
      ${QWT_LIBRARIES}
    )
    set_target_properties(test_qwt_loss_plot
      PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED YES
    )

    option(BUILD_WITH_INTERACTIVE_TESTS "Build with interactive tests" OFF)
    if(BUILD_WITH_INTERACTIVE_TESTS)
      target_compile_definitions(test_qwt_loss_plot PUBLIC INTERACTIVE_TESTS)
    endif(BUILD_WITH_INTERACTIVE_TESTS)
  endif(BUILD_WITH_PLOT_TESTS)

  # Scaled Loss Tests
  catkin_add_gtest(test_scaled_loss
    test/test_scaled_loss.cpp
  )
  add_dependencies(test_scaled_loss
    ${catkin_EXPORTED_TARGETS}
  )
  target_include_directories(test_scaled_loss
    PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_scaled_loss
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
  )
  set_target_properties(test_scaled_loss
    PROPERTIES
      CXX_STANDARD 14
      CXX_STANDARD_REQUIRED YES
  )
endif()
