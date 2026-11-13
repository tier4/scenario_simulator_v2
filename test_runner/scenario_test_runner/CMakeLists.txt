cmake_minimum_required(VERSION 3.16.3) # Ubuntu 20.04 default CMake version

project(scenario_test_runner)

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/scenario_test_runner.py
           ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/result_checker.py
           ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/lifecycle_controller.py
           ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/scenario.py
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config
            ${CMAKE_CURRENT_SOURCE_DIR}/launch
            ${CMAKE_CURRENT_SOURCE_DIR}/scenario
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(replay_testing REQUIRED)
  add_replay_test(test/test_traffic_signals.py)
endif()

ament_package()
