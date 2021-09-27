ament_auto_add_executable(accelerate_and_follow
  accelerate_and_follow.cpp
)
target_link_libraries(accelerate_and_follow cpp_scenario_node)

ament_auto_add_executable(decelerate_and_follow
  decelerate_and_follow.cpp
)
target_link_libraries(decelerate_and_follow cpp_scenario_node)

install(TARGETS
  accelerate_and_follow
  decelerate_and_follow
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "accelerate_and_follow" "15.0")
  add_cpp_mock_scenario_test(${PROJECT_NAME} "decelerate_and_follow" "15.0")
endif()
