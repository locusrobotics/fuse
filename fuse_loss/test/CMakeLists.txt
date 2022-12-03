# CORE GTESTS ======================================================================================
ament_add_gtest(test_loss_function test_loss_function.cpp)
target_link_libraries(test_loss_function ${PROJECT_NAME})

ament_add_gtest(test_composed_loss test_composed_loss.cpp)
target_link_libraries(test_composed_loss ${PROJECT_NAME})

ament_add_gtest(test_huber_loss test_huber_loss.cpp)
target_link_libraries(test_huber_loss ${PROJECT_NAME})

ament_add_gtest(test_tukey_loss test_tukey_loss.cpp)
target_link_libraries(test_tukey_loss ${PROJECT_NAME})

ament_add_gtest(test_scaled_loss test_scaled_loss.cpp)
target_link_libraries(test_scaled_loss ${PROJECT_NAME})

# Qwt Loss Plot Tests
option(BUILD_WITH_PLOT_TESTS "Build with plot tests. These test might fail to run in headless systems." OFF)
if(BUILD_WITH_PLOT_TESTS)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

  # Find Qwt using FindQwt.cmake copied from:
  # https://gitlab.kitware.com/cmake/community/-/wikis/contrib/modules/FindQwt
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
  find_package(QWT REQUIRED)

  ament_add_gtest(test_qwt_loss_plot test_qwt_loss_plot.cpp)
  target_link_libraries(test_qwt_loss_plot ${PROJECT_NAME})
  target_include_directories(test_qwt_loss_plot
    PRIVATE
      ${Qt5Core_INCLUDE_DIRS}
      ${Qt5Widgets_INCLUDE_DIRS}
      ${QWT_INCLUDE_DIRS}
  )
  target_link_libraries(test_qwt_loss_plot
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${QWT_LIBRARIES}
  )

  option(BUILD_WITH_INTERACTIVE_TESTS "Build with interactive tests" OFF)
  if(BUILD_WITH_INTERACTIVE_TESTS)
    target_compile_definitions(test_qwt_loss_plot PUBLIC INTERACTIVE_TESTS)
  endif()
endif()
