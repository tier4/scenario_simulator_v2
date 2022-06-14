ament_auto_add_executable(get_longitudinal_distance
  get_longitudinal_distance.cpp
)
target_link_libraries(get_longitudinal_distance cpp_scenario_node)

ament_auto_add_executable(get_distance_to_lane_bound
  get_distance_to_lane_bound.cpp
)
target_link_libraries(get_distance_to_lane_bound cpp_scenario_node)

install(TARGETS
  get_longitudinal_distance
  get_distance_to_lane_bound
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "get_longitudinal_distance" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "get_distance_to_lane_bound" "15.0")
endif()
