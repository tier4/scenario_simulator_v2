ament_auto_add_executable(teleport_action
  teleport_action.cpp
)
target_link_libraries(teleport_action cpp_scenario_node)

install(TARGETS
  teleport_action
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "teleport_action" "20.0")
endif()
