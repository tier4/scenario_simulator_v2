
ament_add_gtest(test_distance test_distance.cpp)
target_link_libraries(test_distance traffic_simulator)

ament_add_gtest(test_pose test_pose.cpp)
target_link_libraries(test_pose traffic_simulator)
