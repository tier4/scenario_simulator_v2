ament_auto_add_executable(accelerate_and_follow
  accelerate_and_follow.cpp
)
target_link_libraries(accelerate_and_follow cpp_scenario_node)

install(TARGETS
  accelerate_and_follow
  DESTINATION lib/cpp_mock_scenarios
)
