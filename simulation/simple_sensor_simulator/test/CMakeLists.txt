find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

add_subdirectory(src/sensor_simulation/lidar)
add_subdirectory(src/sensor_simulation/primitives)
add_subdirectory(src/sensor_simulation/occupancy_grid)
