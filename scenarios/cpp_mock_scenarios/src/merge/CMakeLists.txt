ament_auto_add_executable(merge_left
  merge_left.cpp
)
target_link_libraries(merge_left cpp_scenario_node)

install(TARGETS
  merge_left
  DESTINATION lib/cpp_mock_scenarios
)
