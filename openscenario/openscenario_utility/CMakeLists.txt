cmake_minimum_required(VERSION 3.16.3) # Ubuntu 20.04 default CMake version

project(openscenario_utility)

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/conversion.py
           ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/validation.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
