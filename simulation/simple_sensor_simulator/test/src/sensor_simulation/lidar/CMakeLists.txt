find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

ament_add_gtest(test_raycaster test_raycaster.cpp)
target_link_libraries(test_raycaster simple_sensor_simulator_component ${Protobuf_LIBRARIES})
