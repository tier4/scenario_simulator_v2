ament_auto_add_executable(phase_control
  phase_control.cpp
)
target_link_libraries(phase_control cpp_scenario_node)

install(TARGETS
  phase_control
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "phase_control" "15")
endif()
