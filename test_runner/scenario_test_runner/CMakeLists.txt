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
           ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/shutdown_once.py
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/launch/scenario_test_runner.launch.py
           ${CMAKE_CURRENT_SOURCE_DIR}/launch/replay_bag.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config
            ${CMAKE_CURRENT_SOURCE_DIR}/launch
            ${CMAKE_CURRENT_SOURCE_DIR}/scenario
  DESTINATION share/${PROJECT_NAME})

ament_package()
