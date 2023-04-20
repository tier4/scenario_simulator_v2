cmake_minimum_required(VERSION 3.16.3) # Ubuntu 20.04 default CMake version

project(openscenario_validator VERSION 0.0.0)

find_package(ament_cmake_auto REQUIRED)
find_package(XercesC          REQUIRED)

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/configure/OpenSCENARIO-1.2.xsd ${PROJECT_NAME}_OPENSCENARIO_1_2_XSD)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configure/schema.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/schema.cpp)

ament_auto_add_library(${PROJECT_NAME} STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/schema.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/validator.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_EXTENSIONS OFF
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON)

ament_auto_package()
