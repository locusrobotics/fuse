find_package(fuse_constraints REQUIRED)
find_package(fuse_graphs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# CORE GTESTS ======================================================================================
ament_add_gtest("test_stamped_variable_synchronizer" "test_stamped_variable_synchronizer.cpp")
target_link_libraries("test_stamped_variable_synchronizer"
  ${PROJECT_NAME}
  fuse_constraints::fuse_constraints
  fuse_graphs::fuse_graphs
)

ament_add_gtest("test_path_2d_publisher" "test_path_2d_publisher.cpp")
target_link_libraries("test_path_2d_publisher"
  ${PROJECT_NAME}
  fuse_constraints::fuse_constraints
  fuse_graphs::fuse_graphs
)

ament_add_gtest("test_pose_2d_publisher" "test_pose_2d_publisher.cpp")
target_link_libraries("test_pose_2d_publisher"
  ${PROJECT_NAME}
  fuse_constraints::fuse_constraints
  fuse_graphs::fuse_graphs
)
