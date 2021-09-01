ament_auto_add_executable(get_longitudinal_distance
  get_longitudinal_distance.cpp
)
target_link_libraries(get_longitudinal_distance cpp_scenario_node)

install(TARGETS
  get_longitudinal_distance
  DESTINATION lib/cpp_mock_scenarios
)
