ament_add_gtest(test_line_segment test_line_segment.cpp)
target_link_libraries(test_line_segment geometry)

ament_add_gtest(test_polygon test_polygon.cpp)
target_link_libraries(test_polygon geometry)
