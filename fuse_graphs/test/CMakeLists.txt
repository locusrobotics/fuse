# CORE GTESTS ======================================================================================
ament_add_gtest(test_hash_graph test_hash_graph.cpp)
target_link_libraries(test_hash_graph ${PROJECT_NAME})
