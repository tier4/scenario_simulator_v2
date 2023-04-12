ament_auto_add_executable(traveled_distance
  traveled_distance.cpp
)
target_link_libraries(traveled_distance cpp_scenario_node)

install(TARGETS
  traveled_distance
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "traveled_distance" "15.0")
endif()
