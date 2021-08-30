ament_auto_add_executable(idiot_npc
  idiot_npc.cpp
)
target_link_libraries(idiot_npc cpp_scenario_node)

ament_auto_add_executable(spwan_with_offset_scenario
  spwan_with_offset_scenario.cpp
)
target_link_libraries(spwan_with_offset_scenario cpp_scenario_node)

install(TARGETS
  idiot_npc
  spwan_with_offset_scenario
  DESTINATION lib/cpp_mock_scenarios
)
