ament_auto_add_executable(request_speed_change
  request_speed_change.cpp
)
target_link_libraries(request_speed_change cpp_scenario_node)

ament_auto_add_executable(request_speed_change_relative
  request_speed_change_relative.cpp
)
target_link_libraries(request_speed_change_relative cpp_scenario_node)

install(TARGETS
  request_speed_change request_speed_change_relative
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "request_speed_change" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "request_speed_change_relative" "5.0")
endif()
