add_subdirectory(src/traffic_lights)
add_subdirectory(src/helper)
add_subdirectory(src/entity)
add_subdirectory(src/behavior)

ament_add_gtest(test_hdmap_utils src/test_hdmap_utils.cpp)
target_link_libraries(test_hdmap_utils traffic_simulator)
