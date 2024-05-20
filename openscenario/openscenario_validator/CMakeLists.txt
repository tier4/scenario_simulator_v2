cmake_minimum_required(VERSION 3.16.3)

project(openscenario_validator)

find_package(ament_cmake REQUIRED)
find_package(XercesC REQUIRED)

add_executable(validate
  src/validator_command.cpp)

target_link_libraries(validate ${XercesC_LIBRARIES})
target_include_directories(validate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_include_directories(include)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS validate
  DESTINATION lib/${PROJECT_NAME})

ament_package()
