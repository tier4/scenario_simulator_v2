ament_auto_add_executable(walk_straight
  walk_straight.cpp
)
target_link_libraries(walk_straight cpp_scenario_node)

install(TARGETS
  walk_straight
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "walk_straight" "15.0")
endif()
