cmake_minimum_required(VERSION 3.5)
project(awapi_accessor)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

execute_process(
  COMMAND grep -e "<depend>" ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
  COMMAND sed -E "s/(.*)<(.*)>(.*)<\\/\\2>/\\3;/g"
  COMMAND tr -d "\n"
  OUTPUT_VARIABLE ${PROJECT_NAME}_DEPENDENCIES)

foreach(PACKAGE IN LISTS ${PROJECT_NAME}_DEPENDENCIES)
  if(PACKAGE)
    find_package(${PACKAGE} REQUIRED)
  endif()
endforeach()

# ------------------------------------------------------------------------------
#  libawapi_accessor
# ------------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/awapi_accessor.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_DEPENDENCIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_interfaces(${PROJECT_NAME}-config)
ament_export_dependencies(${${PROJECT_NAME}_DEPENDENCIES})

# ------------------------------------------------------------------------------
#  make install
# ------------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-config
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ------------------------------------------------------------------------------
#  test
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
