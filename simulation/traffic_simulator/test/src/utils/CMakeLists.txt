
ament_add_gtest(test_distance test_distance.cpp)
target_link_libraries(test_distance traffic_simulator)

ament_add_gtest(test_pose test_pose.cpp)
target_link_libraries(test_pose traffic_simulator)

ament_add_gtest(test_junit5 test_junit5.cpp)
target_link_libraries(test_junit5 traffic_simulator)
target_compile_definitions(test_junit5 PRIVATE
  TEST_JUNIT5_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test_junit5")
