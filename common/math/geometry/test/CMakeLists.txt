add_subdirectory(src/intersection)
add_subdirectory(src/polygon)
add_subdirectory(src/quaternion)
add_subdirectory(src/solver)
add_subdirectory(src/spline)
add_subdirectory(src/vector3)

ament_add_gtest(test_distance src/test_distance.cpp)
target_link_libraries(test_distance geometry)

ament_add_gtest(test_bounding_box src/test_bounding_box.cpp)
target_link_libraries(test_bounding_box geometry)

ament_add_gtest(test_transform src/test_transform.cpp)
target_link_libraries(test_transform geometry)
