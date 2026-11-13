# AGNOCAST is explicitly disabled for this module because the tests create both a publisher and a subscriber
# in the same process. This setup causes issues with AGNOCAST memory handling.
# As a temporary workaround, unit tests for lidar_sensor are only run for the ROS 2-based use case.
set(ENV{ENABLE_AGNOCAST_SIMULATOR} "0" CACHE STRING "Set ENABLE_AGNOCAST_SIMULATOR to 0 for testing" FORCE)

ament_add_gtest(test_raycaster test_raycaster.cpp)
agnocast_wrapper_setup(test_raycaster)
target_link_libraries(test_raycaster simple_sensor_simulator_component ${Protobuf_LIBRARIES})

ament_add_gtest(test_lidar_sensor test_lidar_sensor.cpp)
agnocast_wrapper_setup(test_lidar_sensor)
target_link_libraries(test_lidar_sensor simple_sensor_simulator_component ${Protobuf_LIBRARIES})

set_tests_properties(test_lidar_sensor
  PROPERTIES
  ENVIRONMENT "LD_PRELOAD=/opt/ros/humble/lib/libagnocast_heaphook.so:;AGNOCAST_MEMPOOL_SIZE=134217728"
)
