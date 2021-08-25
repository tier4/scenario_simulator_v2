ament_auto_add_executable(traveled_distance
  traveled_distance.cpp
)
target_link_libraries(traveled_distance cpp_scenario_node)

ament_auto_add_executable(momentary_stop
  momentary_stop.cpp
)
target_link_libraries(momentary_stop cpp_scenario_node)

install(TARGETS
  traveled_distance
  momentary_stop
  DESTINATION lib/cpp_mock_scenarios
)
