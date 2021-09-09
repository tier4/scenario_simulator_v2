ament_auto_add_executable(walk_straight
  walk_straight.cpp
)
target_link_libraries(walk_straight cpp_scenario_node)

install(TARGETS
  walk_straight
  DESTINATION lib/cpp_mock_scenarios
)
