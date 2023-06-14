ament_auto_add_executable(load_do_nothing_plugin
  load_do_nothing_plugin.cpp
)
target_link_libraries(load_do_nothing_plugin cpp_scenario_node)

install(TARGETS
  load_do_nothing_plugin
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "load_do_nothing_plugin" "5.0")
endif()
