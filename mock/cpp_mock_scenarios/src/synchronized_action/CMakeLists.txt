ament_auto_add_executable(synchronized_action
  synchronized_action.cpp
)
ament_auto_add_executable(synchronized_action_with_speed
  synchronized_action_with_speed.cpp
)

target_link_libraries(synchronized_action cpp_scenario_node)
target_link_libraries(synchronized_action_with_speed cpp_scenario_node)

install(TARGETS
  synchronized_action
  synchronized_action_with_speed
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "synchronized_action" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "synchronized_action_with_speed" "15.0")
endif()
