ament_auto_add_executable(follow_polyline_trajectory_with_do_nothing_plugin
follow_polyline_trajectory_with_do_nothing_plugin.cpp
)
target_link_libraries(follow_polyline_trajectory_with_do_nothing_plugin cpp_scenario_node)

install(TARGETS
  follow_polyline_trajectory_with_do_nothing_plugin
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "follow_polyline_trajectory_with_do_nothing_plugin" "15.0")
endif()
