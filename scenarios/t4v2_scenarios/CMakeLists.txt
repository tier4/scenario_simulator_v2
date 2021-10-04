cmake_minimum_required(VERSION 3.8)
project(t4v2_scenarios)

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  include(cmake/add_t4v2_scenario_test.cmake)
  add_t4v2_scenario_test(
    "workflow_example"
    ${PROJECT_NAME}
    "$(find-pkg-share ${PROJECT_NAME})/workflow/workflow_example.yaml"
    "20"
    "300")
endif()

install(
  DIRECTORY workflow scenario
  DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake")
