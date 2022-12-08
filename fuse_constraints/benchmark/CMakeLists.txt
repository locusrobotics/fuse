# Benchmarks
find_package(benchmark QUIET)
if(benchmark_FOUND)
  # Normal Delta Pose 2D benchmark
  add_executable(benchmark_normal_delta_pose_2d benchmark_normal_delta_pose_2d.cpp)
  if(TARGET benchmark_normal_delta_pose_2d)
    target_link_libraries(benchmark_normal_delta_pose_2d
      benchmark::benchmark
      ${PROJECT_NAME}
    )
  endif()

  # Normal Prior Pose 2D benchmark
  add_executable(benchmark_normal_prior_pose_2d benchmark_normal_prior_pose_2d.cpp)
  if(TARGET benchmark_normal_prior_pose_2d)
    target_link_libraries(benchmark_normal_prior_pose_2d
      benchmark::benchmark
      ${PROJECT_NAME}
    )
  endif()
endif()
