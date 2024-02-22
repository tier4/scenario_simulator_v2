ament_auto_add_executable(respawn_ego
  respawn_ego.cpp
)
target_link_libraries(respawn_ego cpp_scenario_node)

install(TARGETS
  respawn_ego
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "respawn_ego" "40.0")
endif()
