ament_add_gtest(test_vehicle_entity test_vehicle_entity.cpp)
target_link_libraries(test_vehicle_entity traffic_simulator)

ament_add_gtest(test_misc_object_entity test_misc_object_entity.cpp)
target_link_libraries(test_misc_object_entity traffic_simulator)
