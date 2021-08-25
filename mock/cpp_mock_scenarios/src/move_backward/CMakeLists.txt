ament_auto_add_executable(move_backward
  move_backward.cpp
)
target_link_libraries(move_backward cpp_scenario_node)

install(TARGETS
  move_backward
  DESTINATION lib/cpp_mock_scenarios
)
