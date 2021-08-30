ament_auto_add_executable(idiot_npc
  idiot_npc.cpp
)
target_link_libraries(idiot_npc cpp_scenario_node)

ament_auto_add_executable(spwan_with_offset
  spwan_with_offset.cpp
)
target_link_libraries(spwan_with_offset cpp_scenario_node)

install(TARGETS
  idiot_npc
  spwan_with_offset
  DESTINATION lib/cpp_mock_scenarios
)
