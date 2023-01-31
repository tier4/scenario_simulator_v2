cmake_minimum_required(VERSION 3.16.3) # Ubuntu 20.04 default CMake version

project(scenario_test_runner)

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config
            ${CMAKE_CURRENT_SOURCE_DIR}/launch
            ${CMAKE_CURRENT_SOURCE_DIR}/scenario
  DESTINATION share/${PROJECT_NAME})

ament_package()
