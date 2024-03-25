ament_auto_add_executable(request_speed_change
  request_speed_change.cpp
)
target_link_libraries(request_speed_change cpp_scenario_node)

ament_auto_add_executable(request_speed_change_relative
  request_speed_change_relative.cpp
)
target_link_libraries(request_speed_change_relative cpp_scenario_node)

ament_auto_add_executable(request_speed_change_step
  request_speed_change_step.cpp
)
target_link_libraries(request_speed_change_step cpp_scenario_node)

ament_auto_add_executable(request_speed_change_with_limit
  request_speed_change_with_limit.cpp
)
target_link_libraries(request_speed_change_with_limit cpp_scenario_node)

ament_auto_add_executable(request_speed_change_with_time_constraint_linear
  request_speed_change_with_time_constraint_linear.cpp
)
target_link_libraries(request_speed_change_with_time_constraint_linear cpp_scenario_node)

ament_auto_add_executable(request_speed_change_continuous_false
  request_speed_change_continuous_false.cpp
)
target_link_libraries(request_speed_change_continuous_false cpp_scenario_node)

ament_auto_add_executable(request_speed_change_with_time_constraint
  request_speed_change_with_time_constraint.cpp
)
target_link_libraries(request_speed_change_with_time_constraint cpp_scenario_node)

ament_auto_add_executable(request_speed_change_with_time_constraint_relative
  request_speed_change_with_time_constraint_relative.cpp
)
target_link_libraries(request_speed_change_with_time_constraint_relative cpp_scenario_node)

install(TARGETS
  request_speed_change
  request_speed_change_relative
  request_speed_change_step
  request_speed_change_with_limit
  request_speed_change_with_time_constraint_linear
  request_speed_change_continuous_false
  request_speed_change_with_time_constraint
  request_speed_change_with_time_constraint_relative
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "request_speed_change" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "request_speed_change_relative" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "request_speed_change_step" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "request_speed_change_with_limit" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "request_speed_change_continuous_false" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "request_speed_change_with_time_constraint" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "request_speed_change_with_time_constraint_relative" "5.0")

  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "request_speed_change_with_time_constraint_linear" "5.0")
endif()
