ament_auto_add_executable(random001
  random001.cpp
)
target_link_libraries(random001 cpp_scenario_node)

install(TARGETS
  random001
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  # add_cpp_mock_scenario_test(${PROJECT_NAME} "random001" "10.0")
endif()
