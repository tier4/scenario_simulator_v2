cmake_minimum_required(VERSION 3.5)
project(customized_rvo2_examples)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
#find_package(OpenMP REQUIRED)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
find_package(Boost REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(circle_node src/customized_rvo2_examples/circle_example_node.cpp)
ament_auto_add_executable(block_node src/customized_rvo2_examples/block_example_node.cpp)
ament_auto_add_executable(intersection_node src/customized_rvo2_examples/intersection_example_node.cpp)
ament_auto_add_executable(curve_node src/customized_rvo2_examples/curve_example_node.cpp)
ament_auto_add_executable(test_single_agent_node src/customized_rvo2_examples/single_agent_avoid_node.cpp)
ament_auto_add_executable(test_multiple_agent_node src/customized_rvo2_examples/multiple_agent_avoid_node.cpp)
ament_auto_add_executable(road_demo src/customized_rvo2_examples/road_demo_node.cpp)
ament_auto_add_executable(agent_ignore_node src/customized_rvo2_examples/agent_ignore_node.cpp)
ament_auto_add_executable(critical_corner_case src/customized_rvo2_examples/critical_corner_case.cpp)
ament_auto_add_executable(grid_obstacle_node src/customized_rvo2_examples/grid_obstacle_node.cpp)
ament_auto_add_executable(dynamic_velocity_node src/customized_rvo2_examples/dynamic_velocity_node.cpp)
include_directories(PRIVATE ${GENERATED_PATH})
include_directories(include)
link_libraries(${Boost_LIBRARIES})

# workaround to allow deprecated header to build on both galactic and humble
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(circle_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(block_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(intersection_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(curve_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(test_single_agent_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(test_multiple_agent_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(road_demo PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(agent_ignore_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(critical_corner_case PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(grid_obstacle_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(dynamic_velocity_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_examples test/src/test_examples.cpp)
  ament_target_dependencies(test_examples customized_rvo2)
  if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
    target_compile_definitions(test_examples PUBLIC
      USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
    )
  endif()
endif()

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_auto_package()
