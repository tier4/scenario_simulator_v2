ament_auto_add_executable(lanechange_left
  lanechange_left.cpp
)
target_link_libraries(lanechange_left cpp_scenario_node)

install(TARGETS
  lanechange_left
  DESTINATION lib/cpp_mock_scenarios
)
