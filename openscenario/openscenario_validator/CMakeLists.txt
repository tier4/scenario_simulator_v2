cmake_minimum_required(VERSION 3.16.3)

project(openscenario_validator)

find_package(ament_cmake REQUIRED)
find_package(XercesC REQUIRED)

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/configure/OpenSCENARIO-1.3.xsd ${PROJECT_NAME}_OPENSCENARIO_1_3_XSD)

configure_file(configure/schema.cpp src/schema.cpp)

include_directories(${XercesC_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME} STATIC src/schema.cpp)

target_link_libraries(${PROJECT_NAME} ${XercesC_LIBRARIES})

add_executable(validate
  src/validator_command.cpp)

target_link_libraries(validate ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME})

install(TARGETS validate
  DESTINATION lib/${PROJECT_NAME})

ament_package()
