ament_auto_add_executable(merge_left
  merge_left.cpp
)
target_link_libraries(merge_left cpp_scenario_node)

install(TARGETS
  merge_left
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "merge_left" "15")
endif()
