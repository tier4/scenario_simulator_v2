ament_add_gtest(test_collision test_collision.cpp)
target_link_libraries(test_collision geometry)

ament_add_gtest(test_intersection test_intersection.cpp)
target_link_libraries(test_intersection geometry)
