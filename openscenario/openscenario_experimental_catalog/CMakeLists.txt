cmake_minimum_required(VERSION 3.16.3) # Ubuntu 20.04 default CMake version

project(openscenario_experimental_catalog)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/vehicle
  DESTINATION share/${PROJECT_NAME})

ament_package()
