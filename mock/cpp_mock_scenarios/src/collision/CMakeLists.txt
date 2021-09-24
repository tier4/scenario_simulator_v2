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

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "crashing_npc" "5.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "spawn_with_offset" "15.0")
endif()
