ament_add_gtest(test_box test_box.cpp)
target_link_libraries(test_box simple_sensor_simulator_component)

ament_add_gtest(test_vertex test_vertex.cpp)
target_link_libraries(test_vertex simple_sensor_simulator_component)

ament_add_gtest(test_primitive test_primitive.cpp)
target_link_libraries(test_primitive simple_sensor_simulator_component ${Protobuf_LIBRARIES})
