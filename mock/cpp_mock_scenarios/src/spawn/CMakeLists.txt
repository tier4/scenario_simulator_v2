ament_auto_add_executable(spawn_in_map_frame
  spawn_in_map_frame.cpp
)
target_link_libraries(spawn_in_map_frame cpp_scenario_node)

install(TARGETS
  spawn_in_map_frame
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "spawn_in_map_frame" "5.0")
endif()
