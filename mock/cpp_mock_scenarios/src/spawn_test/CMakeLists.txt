ament_auto_add_executable(spawn_test
  spawn_test.cpp
)
target_link_libraries(spawn_test cpp_scenario_node)

install(TARGETS
  spawn_test
  DESTINATION lib/cpp_mock_scenarios
)
