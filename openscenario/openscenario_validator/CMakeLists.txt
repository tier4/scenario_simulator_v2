cmake_minimum_required(VERSION 3.16.3)

project(openscenario_validator)

find_package(ament_cmake REQUIRED)
find_package(XercesC REQUIRED)

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/configure/OpenSCENARIO-1.3.xsd ${PROJECT_NAME}_OPENSCENARIO_1_3_XSD)

configure_file(configure/schema.cpp src/schema.cpp)

add_library(${PROJECT_NAME} STATIC src/schema.cpp)

target_link_libraries(${PROJECT_NAME} ${XercesC_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(validate
  src/validator_command.cpp)

target_link_libraries(validate ${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS validate
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME})

ament_package()
