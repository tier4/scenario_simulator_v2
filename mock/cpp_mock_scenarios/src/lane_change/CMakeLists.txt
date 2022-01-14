ament_auto_add_executable(lanechange_left
  lanechange_left.cpp
)
target_link_libraries(lanechange_left cpp_scenario_node)

ament_auto_add_executable(lanechange_left_with_id
  lanechange_left_with_id.cpp
)
target_link_libraries(lanechange_left_with_id cpp_scenario_node)

ament_auto_add_executable(lanechange_right
  lanechange_right.cpp
)
target_link_libraries(lanechange_right cpp_scenario_node)

ament_auto_add_executable(lanechange_right_with_id
  lanechange_right_with_id.cpp
)
target_link_libraries(lanechange_right_with_id cpp_scenario_node)

ament_auto_add_executable(lanechange_linear
  lanechange_linear.cpp
)
target_link_libraries(lanechange_linear cpp_scenario_node)

ament_auto_add_executable(lanechange_linear_time
  lanechange_linear_time.cpp
)
target_link_libraries(lanechange_linear_time cpp_scenario_node)

ament_auto_add_executable(lanechange_linear_lateral_velocity
  lanechange_linear_lateral_velocity.cpp
)
target_link_libraries(lanechange_linear_lateral_velocity cpp_scenario_node)

ament_auto_add_executable(lanechange_time
  lanechange_time.cpp
)
target_link_libraries(lanechange_time cpp_scenario_node)

install(TARGETS
  lanechange_left
  lanechange_left_with_id
  lanechange_linear
  lanechange_linear_lateral_velocity
  lanechange_linear_time
  lanechange_right
  lanechange_right_with_id
  lanechange_time
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "lanechange_left_with_id" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "lanechange_left" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "lanechange_linear_lateral_velocity" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "lanechange_linear_time" "30.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "lanechange_linear" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "lanechange_right_with_id" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "lanechange_right" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "lanechange_time" "30.0")
endif()
