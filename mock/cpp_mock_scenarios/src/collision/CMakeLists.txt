ament_auto_add_executable(crashing_npc
  crashing_npc.cpp
)
target_link_libraries(crashing_npc cpp_scenario_node)

ament_auto_add_executable(spawn_with_offset
  spawn_with_offset.cpp
)
target_link_libraries(spawn_with_offset cpp_scenario_node)

install(TARGETS
  crashing_npc
  spawn_with_offset
  DESTINATION lib/cpp_mock_scenarios
)
