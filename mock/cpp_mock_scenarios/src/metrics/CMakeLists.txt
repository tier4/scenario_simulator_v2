ament_auto_add_executable(traveled_distance
  traveled_distance.cpp
)
target_link_libraries(traveled_distance cpp_scenario_node)

ament_auto_add_executable(momentary_stop
  momentary_stop.cpp
)
target_link_libraries(momentary_stop cpp_scenario_node)

ament_auto_add_executable(reaction_time
  reaction_time.cpp
)
target_link_libraries(reaction_time cpp_scenario_node)

ament_auto_add_executable(out_of_range
  out_of_range.cpp
)
target_link_libraries(out_of_range cpp_scenario_node)

ament_auto_add_executable(standstill_duration
  standstill_duration.cpp
)
target_link_libraries(standstill_duration cpp_scenario_node)

ament_auto_add_executable(collision
  collision.cpp
)
target_link_libraries(collision cpp_scenario_node)

install(TARGETS
  traveled_distance
  momentary_stop
  reaction_time
  out_of_range
  standstill_duration
  collision
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "traveled_distance" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "momentary_stop" "20.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "reaction_time" "10.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "out_of_range" "7.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "collision" "5.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "standstill_duration" "15.0")
endif()
