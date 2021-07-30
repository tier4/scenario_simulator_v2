ament_auto_add_executable(lanechange_right
  lanechange_right.cpp
)
target_link_libraries(lanechange_right cpp_scenario_node)

install(TARGETS
  lanechange_right
  DESTINATION lib/cpp_mock_scenarios
)
