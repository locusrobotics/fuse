# Benchmarks
find_package(benchmark QUIET)

if(benchmark_FOUND)
  add_executable(benchmark_unicycle_2d_state_cost_function
    benchmark_unicycle_2d_state_cost_function.cpp
  )
  if(TARGET benchmark_unicycle_2d_state_cost_function)
    target_link_libraries(
      benchmark_unicycle_2d_state_cost_function
      benchmark::benchmark
      Ceres::ceres
      ${PROJECT_NAME}
    )
    set_target_properties(benchmark_unicycle_2d_state_cost_function
      PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
    )
  endif()
endif()
