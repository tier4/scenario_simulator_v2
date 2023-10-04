cmake_minimum_required(VERSION 3.16.3) # Ubuntu 20.04 default CMake version

project(map_fragment)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-local-typedefs -DBOOST_ALLOW_DEPRECATED_HEADERS)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

file(GLOB ${PROJECT_NAME}_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

foreach(EACH IN LISTS ${PROJECT_NAME}_SOURCES)
  get_filename_component(STEM ${EACH} NAME_WE)
  ament_auto_add_executable(${STEM} ${EACH} ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
