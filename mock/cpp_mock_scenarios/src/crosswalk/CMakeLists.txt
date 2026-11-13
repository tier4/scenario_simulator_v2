ament_auto_add_executable(stop_at_crosswalk
  stop_at_crosswalk.cpp
)
target_link_libraries(stop_at_crosswalk cpp_scenario_node)
ament_auto_add_executable(parked_at_crosswalk
  parked_at_crosswalk.cpp
)
target_link_libraries(parked_at_crosswalk cpp_scenario_node)


install(TARGETS
  stop_at_crosswalk
  parked_at_crosswalk
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "stop_at_crosswalk" "20.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "parked_at_crosswalk" "20.0")
endif()
