ament_auto_add_executable(auto_sink_vehicle
  auto_sink_vehicle.cpp
)
target_link_libraries(auto_sink_vehicle cpp_scenario_node)

install(TARGETS
  auto_sink_vehicle
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "auto_sink_vehicle" "5.0")
endif()
