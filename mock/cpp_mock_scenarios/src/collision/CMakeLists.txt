ament_auto_add_executable(idiot_npc
  idiot_npc.cpp
)
target_link_libraries(idiot_npc cpp_scenario_node)

install(TARGETS
  idiot_npc
  DESTINATION lib/cpp_mock_scenarios
)
