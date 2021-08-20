ament_auto_add_executable(phase_control
  phase_control.cpp
)
target_link_libraries(phase_control cpp_scenario_node)

install(TARGETS
  phase_control
  DESTINATION lib/cpp_mock_scenarios
)
