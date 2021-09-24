ament_auto_add_executable(acquire_position_in_world_frame
  acquire_position_in_world_frame.cpp
)
target_link_libraries(acquire_position_in_world_frame cpp_scenario_node)

ament_auto_add_executable(assign_route_in_world_frame
  assign_route_in_world_frame.cpp
)
target_link_libraries(assign_route_in_world_frame cpp_scenario_node)

ament_auto_add_executable(cancel_request
  cancel_request.cpp
)
target_link_libraries(cancel_request cpp_scenario_node)

install(TARGETS
  acquire_position_in_world_frame
  assign_route_in_world_frame
  cancel_request
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "acquire_position_in_world_frame" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "assign_route_in_world_frame" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "cancel_request" "15.0")
endif()
