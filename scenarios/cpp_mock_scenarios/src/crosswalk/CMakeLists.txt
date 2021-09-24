ament_auto_add_executable(stop_at_crosswalk
  stop_at_crosswalk.cpp
)
target_link_libraries(stop_at_crosswalk cpp_scenario_node)

install(TARGETS
  stop_at_crosswalk
  DESTINATION lib/cpp_mock_scenarios
)
