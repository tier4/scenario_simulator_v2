cmake_minimum_required(VERSION 3.16.3)

project(openscenario_validator)

find_package(ament_cmake REQUIRED)
find_package(XercesC REQUIRED)

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/configure/OpenSCENARIO-1.3.xsd ${PROJECT_NAME}_OPENSCENARIO_1_3_XSD)

configure_file(configure/schema.cpp src/schema.cpp)

add_library(${PROJECT_NAME} STATIC
  src/schema.cpp
  src/validator.cpp)

target_link_libraries(${PROJECT_NAME}
  XercesC::XercesC)

include_directories(${XercesC_INCLUDE_DIRS} include)

add_executable(validate
  src/validator_command.cpp)

target_link_libraries(validator
  ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_EXTENSIONS OFF
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON)

install(TARGETS ${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME})

install(TARGETS validator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
